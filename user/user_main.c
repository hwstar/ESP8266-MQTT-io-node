/* main.c -- ESP8266 MQTT relay node
*
* Provides addressable relay contact, and button sense functions.
* Button can be configured for local control, or isolated for use
* as a separate resource.
* 
* Configuration parameters set with the Makefile using a Python patching
* utility which is avalable on my github site. This allows the configurations
* to differ between nodes and also protects the WIFI login credentials by
* removing them from the source.
*
* Copyright (C) 2015, Stephen Rodgers <steve at rodgers 619 dot com>
* Copyright (c) 2014-2015, Tuan PM <tuanpm at live dot com>
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 
* Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* Neither the name of Redis nor the names of its contributors may be used
* to endorse or promote products derived from this software without
* specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

#include "ets_sys.h"
#include "driver/uart.h"
#include "osapi.h"
#include "mqtt.h"
#include "wifi.h"
#include "config.h"
#include "debug.h"
#include "gpio.h"
#include "user_interface.h"
#include "mem.h"

#define ON 1
#define OFF 0

#define RELAY_GPIO 2
#define RELAY_GPIO_MUX PERIPHS_IO_MUX_GPIO2_U
#define RELAY_GPIO_FUNC FUNC_GPIO2

#define BUTTON_GPIO 0
#define BUTTON_GPIO_MUX PERIPHS_IO_MUX_GPIO0_U
#define BUTTON_GPIO_FUNC FUNC_GPIO0


#define MAX_INFO_ELEMENTS 16
#define INFO_BLOCK_MAGIC 0x3F2A6C17
#define INFO_BLOCK_SIG "ESP8266HWSTARSR"
#define CONFIG_FLD_REQD 0x01

struct config_info_element_tag{
	uint8_t flags;
	uint8_t key[15];
	uint8_t value[80];
}  __attribute__((__packed__));

typedef struct config_info_element_tag config_info_element;

struct config_info_block_tag{
	uint8_t signature[16];
	uint32_t magic;
	uint8_t numelements;
	uint8_t recordLength;
	uint8_t pad[10];
	config_info_element e[MAX_INFO_ELEMENTS];
}  __attribute__((__packed__));

typedef struct config_info_block_tag config_info_block;

enum {WIFISSID=0, WIFIPASS, MQTTHOST, MQTTPORT, MQTTSECUR, MQTTDEVID, MQTTCLNT, MQTTPASS, MQTTKPALIV, MQTTTOPIC, MQTTSTOPIC, MQTTBTLOCAL};

 
/* Configuration block */

LOCAL config_info_block configInfoBlock = {
	.signature = INFO_BLOCK_SIG,
	.magic = INFO_BLOCK_MAGIC,
	.numelements = MAX_INFO_ELEMENTS,
	.recordLength = sizeof(config_info_element),
	.e[WIFISSID] = {.flags = CONFIG_FLD_REQD, .key = "WIFISSID", .value="your_ssid_here"},
	.e[WIFIPASS] = {.flags = CONFIG_FLD_REQD, .key = "WIFIPASS", .value="its_a_secret"},
	.e[MQTTHOST] = {.flags = CONFIG_FLD_REQD, .key = "MQTTHOST", .value="your_mqtt_broker_hostname_here"}, // May also be an IP address
	.e[MQTTPORT] = {.key = "MQTTPORT", .value="1880"}, // destination Port for mqtt broker
	.e[MQTTSECUR] = {.key = "MQTTSECUR",.value="0"}, // Security 0 - no encryption
	.e[MQTTDEVID] = {.key = "MQTTDEVID", .value="your_mqtt_device_id_here"}, // Only relevant if MQTTSECUR is other than 0
	.e[MQTTCLNT] = {.key = "MQTTCLNT", .value="your_mqtt_client_name_here"}, // Only relevant if MQTTSECUR is other than 0
	.e[MQTTPASS] = {.key = "MQTTPASS", .value="its_a_secret"},// Only relevant if MQTTSECUR is other than 0
	.e[MQTTKPALIV] = {.key = "MQTTKPALIV", .value="120"}, // Keepalive interval
	.e[MQTTTOPIC] = {.flags = CONFIG_FLD_REQD, .key = "MQTTTOPIC", .value = "/your/topic/here"},
	.e[MQTTSTOPIC] = {.key = "MQTTSTOPIC", .value = "tbd"}, // Optional topic to send state info to
	.e[MQTTBTLOCAL] = {.key = "MQTTBTLOCAL", .value = "1"} // Optional local toggle control using GPIO0
};
	

LOCAL int relay_state = OFF;
LOCAL int button_state = 1;
LOCAL int localControl = 1;

LOCAL os_timer_t pulse_timer, button_timer;

MQTT_Client mqttClient;


/*
 * Crude case insensitive string matching function
 */
 
int userStrnMatchi(const char *s1, const char *s2, int len)
{
	int i;
	for(i = 0; *s1 && *s2 && i < len; i++){
		if(tolower(*s1) != tolower(*s2))
			break;
	}
	if(i == len)
		return 0; /* Match by length */
	else
		return 1; /* No match */	
}

/*
 * Send relay state update message
 */
 
int updateRelayState(int s)
{
	char *t = configInfoBlock.e[MQTTSTOPIC].value;
	char *state = s ? "ON" : "OFF";
	char result[16];
	os_strcpy(result,"RELAYSTATE:");
	os_strcat(result, state);
	if('/' == *t){
		INFO("MQTT: New Relay State: %s\r\n", state);
		MQTT_Publish(&mqttClient, t, result, os_strlen(result), 0, 0);
	}
	else
		INFO("MQTT: State topic not set!\r\n");
}

/*
 * Toggle the relay output
 */

void relayToggle(void)
{
	relay_state = (relay_state) ? 0 : 1;
	GPIO_OUTPUT_SET(RELAY_GPIO, relay_state);
	updateRelayState(relay_state);
}

/*
 * Update the button state
 */

int updateButtonState(int s)
{
	char *t = configInfoBlock.e[MQTTSTOPIC].value;
	char *state = s ? "RELEASED" : "DEPRESSED";
	char result[25];
	os_strcpy(result,"BUTTONSTATE:");
	os_strcat(result, state);
	if('/' == *t){
		INFO("MQTT: New Button State: %s\r\n", state);
		MQTT_Publish(&mqttClient, t, result, os_strlen(result), 0, 0);
	}
	else
		INFO("MQTT: State topic not set!\r\n");
}

/*
 * WIFI connect call back
 */
 

void wifiConnectCb(uint8_t status)
{
	if(status == STATION_GOT_IP){
		MQTT_Connect(&mqttClient);
	}
}

/*
 * MQTT Connect call back
 */
 
void mqttConnectedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Connected\r\n");
	MQTT_Subscribe(client, configInfoBlock.e[MQTTTOPIC].value, 0);
	updateRelayState(relay_state);

}

/*
 * MQTT Disconnect call back
 */
 

void mqttDisconnectedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Disconnected\r\n");
}

/*
 * MQTT published call back
 */

void mqttPublishedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Published\r\n");
}

/*
 * MQTT Data call back
 */

void mqttDataCb(uint32_t *args, const char* topic, uint32_t topic_len, const char *data, uint32_t data_len)
{
	char *topicBuf = (char*)os_zalloc(topic_len+1),
			*dataBuf = (char*)os_zalloc(data_len+1);

	MQTT_Client* client = (MQTT_Client*)args;

	os_memcpy(topicBuf, topic, topic_len);
	topicBuf[topic_len] = 0;

	os_memcpy(dataBuf, data, data_len);
	dataBuf[data_len] = 0;

	INFO("Receive topic: %s, data: %s \r\n", topicBuf, dataBuf);
	
	if (os_strcmp(topicBuf, configInfoBlock.e[MQTTTOPIC].value) == 0){
		if(!userStrnMatchi(dataBuf, "on", 2)){
			GPIO_OUTPUT_SET(RELAY_GPIO, 1);
			INFO("MQTT: Relay ON\r\n");
			relay_state = ON;
			updateRelayState(relay_state);
	

		}
		else if(!userStrnMatchi(dataBuf, "off", 3)){
			GPIO_OUTPUT_SET(RELAY_GPIO, 0);
			INFO("MQTT: Relay OFF\r\n");
			relay_state = OFF;
			updateRelayState(relay_state);
		}
		else if(!userStrnMatchi(dataBuf, "toggle", 3)){
			INFO("MQTT: Relay toggle: %s\r\n", (relay_state) ? "ON" : "OFF");
			relayToggle();
		}
		else if(!userStrnMatchi(dataBuf, "pulse:", 6)){
			int pulse_duration = atoi(dataBuf+6);
			if(pulse_duration){
				GPIO_OUTPUT_SET(RELAY_GPIO, 1);
				INFO("MQTT: Relay pulse\r\n");
				relay_state = ON;
				updateRelayState(relay_state);
				os_timer_arm(&pulse_timer, pulse_duration, 0);
			}			
		}
		else if(!userStrnMatchi(dataBuf, "query:state", 11)){
			updateRelayState(relay_state);
		}			
	}
				
	os_free(topicBuf);
	os_free(dataBuf);
}

/*
 * Pulse timer callback function
 */
 
void pulseTmerExpireCb(void *arg)
{
		GPIO_OUTPUT_SET(RELAY_GPIO, 0);
		relay_state = OFF;
		updateRelayState(relay_state);
}


/*
 * Check for button state change
 */
 
void buttonTimerCb(void *arg)
{
	int newstate =  GPIO_INPUT_GET(BUTTON_GPIO);
	if(newstate != button_state){
		if(newstate){
			char result[32];
			
			if(localControl) // If local control enabled
				relayToggle(); // Toggle the relay state
			INFO("Button released\r\n");
		}
		else{
			INFO("Button pressed\r\n");
		}	
		button_state = newstate;
		updateButtonState(button_state);
	}
	
}

/*
 * User initialization
 */

void user_init(void)
{
	gpio_init();
		
	// Initialize relay GPIO as an output
	PIN_FUNC_SELECT(RELAY_GPIO_MUX, RELAY_GPIO_FUNC);
	GPIO_OUTPUT_SET(RELAY_GPIO, 0); // Relay off initially

	// Uart init
	uart0_init(BIT_RATE_115200);

	os_delay_us(1000000);

	/* Initialize MQTT connection */
	
	uint8_t *host = configInfoBlock.e[MQTTHOST].value;
	uint32_t port = (uint32_t) atoi(configInfoBlock.e[MQTTPORT].value);
	
	MQTT_InitConnection(&mqttClient, host, port,
	(uint8_t) atoi(configInfoBlock.e[MQTTSECUR].value));

	MQTT_InitClient(&mqttClient, configInfoBlock.e[MQTTDEVID].value, 
	configInfoBlock.e[MQTTCLNT].value, configInfoBlock.e[MQTTPASS].value,
	atoi(configInfoBlock.e[MQTTKPALIV].value), 1);


	MQTT_InitLWT(&mqttClient, "/lwt", "offline", 0, 0);
	MQTT_OnConnected(&mqttClient, mqttConnectedCb);
	MQTT_OnDisconnected(&mqttClient, mqttDisconnectedCb);
	MQTT_OnPublished(&mqttClient, mqttPublishedCb);
	MQTT_OnData(&mqttClient, mqttDataCb);

	os_timer_disarm(&pulse_timer);
	os_timer_setfn(&pulse_timer, (os_timer_func_t *)pulseTmerExpireCb, (void *)0);
	os_timer_disarm(&button_timer);
	os_timer_setfn(&button_timer, (os_timer_func_t *)buttonTimerCb, (void *)0);

	/* Attempt WIFI connection */
	
	uint8_t *ssid = configInfoBlock.e[WIFISSID].value;
	uint8_t *wifipass = configInfoBlock.e[WIFIPASS].value;
	
	INFO("Attempting connection with: %s\r\n", ssid);
	INFO("Main topic: %s\r\n", configInfoBlock.e[MQTTTOPIC].value);
	INFO("Status topic: %s\r\n", configInfoBlock.e[MQTTSTOPIC].value);
	
	WIFI_Connect(ssid, wifipass, wifiConnectCb);
	
	// Set local control option based on config
	localControl = atoi(configInfoBlock.e[MQTTBTLOCAL].value);
	
	os_timer_arm(&button_timer, 100, 1);
	
	INFO("\r\nSystem started ...\r\n");

}
