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

#include "util.h"
#include "kvstore.h"

#define ON 1
#define OFF 0

#define RELAY_GPIO 12
#define RELAY_GPIO_MUX PERIPHS_IO_MUX_MTDI_U
#define RELAY_GPIO_FUNC FUNC_GPIO12
#define RELAY_ON 0
#define RELAY_OFF 1

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


typedef union {
	char str[16];
	unsigned u;
	int i;
} pu;

typedef struct {
	const char *command;
	uint8_t type;
	pu p;
} command_element;


typedef struct config_info_block_tag config_info_block;

enum {WIFISSID=0, WIFIPASS, MQTTHOST, MQTTPORT, MQTTSECUR, MQTTDEVID, MQTTCLNT, MQTTPASS, MQTTKPALIV, MQTTRTOPIC, MQTTBTLOCAL};
enum {CP_NONE= 0, CP_INT, CP_BOOL};

 
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
	.e[MQTTRTOPIC] = {.flags = CONFIG_FLD_REQD, .key = "MQTTRTOPIC", .value = "/home/lab/relay"}, // Root topic
	.e[MQTTBTLOCAL] = {.key = "MQTTBTLOCAL", .value = "1"} // Optional local toggle control using GPIO0
};

/* Command elements */
 
enum {CMD_OFF = 0, CMD_ON, CMD_TOGGLE, CMD_PULSE, CMD_MQTTBTLOCAL};

LOCAL command_element commandElements[] = {
	{.command = "OFF", .type = CP_NONE},
	{.command = "ON", .type = CP_NONE},
	{.command = "TOGGLE", .type = CP_NONE},
	{.command = "PULSE", .type = CP_INT},
	{.command = "MQTTBTLOCAL", .type = CP_INT},
	{.command = ""} /* End marker */
};
	

LOCAL const char *infoString = "root:%s;ip4:%d.%d.%d.%d;schema:hwstar.relaynode";	

LOCAL int relay_state = OFF;
LOCAL int button_state = 1;
LOCAL char *commandTopic, *statusTopic;
LOCAL flash_handle_s *configHandle;
LOCAL os_timer_t pulse_timer, button_timer;

MQTT_Client mqttClient;

/*
 * Send relay state update message
 */
 
LOCAL void ICACHE_FLASH_ATTR updateRelayState(int s)
{
	char *state = s ? "ON" : "OFF";
	char result[16];
	os_strcpy(result,"RELAYSTATE:");
	os_strcat(result, state);
	INFO("MQTT: New Relay State: %s\r\n", state);
	MQTT_Publish(&mqttClient, statusTopic, result, os_strlen(result), 0, 0);

}


/*
 * Set new relay state
 */
 
LOCAL void ICACHE_FLASH_ATTR relaySet(bool new_state)
{
	relay_state = new_state;
	GPIO_OUTPUT_SET(RELAY_GPIO, ((relay_state) ? RELAY_ON : RELAY_OFF));
	updateRelayState(relay_state);
}
	


/*
 * Toggle the relay output
 */

LOCAL void ICACHE_FLASH_ATTR relayToggle(void)
{
	bool new_relay_state = (relay_state) ? OFF : ON;
	relaySet(new_relay_state);
}

/*
 * Update the button state
 */

LOCAL void ICACHE_FLASH_ATTR updateButtonState(int s)
{

	
	char *state = s ? "RELEASED" : "DEPRESSED";
	char result[25];
	os_strcpy(result,"BUTTONSTATE:");
	os_strcat(result, state);
	INFO("MQTT: New Button State: %s\r\n", state);
	MQTT_Publish(&mqttClient, statusTopic, result, os_strlen(result), 0, 0);
	
}

/*
 * WIFI connect call back
 */
 

LOCAL void ICACHE_FLASH_ATTR wifiConnectCb(uint8_t status)
{
	if(status == STATION_GOT_IP){
		MQTT_Connect(&mqttClient);
	}
}

/*
 * MQTT Connect call back
 */
 
LOCAL void ICACHE_FLASH_ATTR mqttConnectedCb(uint32_t *args)
{
	char *buf = util_zalloc(os_strlen(configInfoBlock.e[MQTTRTOPIC].value) + os_strlen(infoString) + 1);
	struct ip_info ipConfig;
	MQTT_Client* client = (MQTT_Client*)args;
	
	INFO("MQTT: Connected\r\n");
	
	// Publish who we are and where we live
	wifi_get_ip_info(STATION_IF, &ipConfig);
	
	os_sprintf(buf, infoString,
			configInfoBlock.e[MQTTRTOPIC].value,
			*((uint8_t *) &ipConfig.ip.addr),
			*((uint8_t *) &ipConfig.ip.addr + 1),
			*((uint8_t *) &ipConfig.ip.addr + 2),
			*((uint8_t *) &ipConfig.ip.addr + 3));

	INFO("MQTT Node info: %s\r\n", buf);

	MQTT_Publish(client, "/node/info", buf, os_strlen(buf), 0, 0);
	
	// Subscribe to command topic
	MQTT_Subscribe(client, commandTopic, 0);
	// Publish relay state
	updateRelayState(relay_state); 
	// Free the buffer
	util_free(buf);
}

/*
 * MQTT Disconnect call back
 */
 

LOCAL void ICACHE_FLASH_ATTR mqttDisconnectedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Disconnected\r\n");
}

/*
 * MQTT published call back
 */

LOCAL void ICACHE_FLASH_ATTR mqttPublishedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Published\r\n");
}

/*
 * MQTT Data call back
 */

LOCAL void ICACHE_FLASH_ATTR mqttDataCb(uint32_t *args, const char* topic, uint32_t topic_len, const char *data, uint32_t data_len)
{
	char *topicBuf, *dataBuf;
	uint8_t i;

	MQTT_Client* client = (MQTT_Client*)args;

	topicBuf = util_strndup(topic, topic_len);
	dataBuf = util_strndup(data, data_len);
	
	INFO("Receive topic: %s, data: %s \r\n", topicBuf, dataBuf);
	
	
	if (!os_strcmp(topicBuf, commandTopic)){
		// Decode command
		for(i = 0; commandElements[i].command[0]; i++){
			command_element *ce = &commandElements[i];
			uint8_t cmdlen = os_strlen(ce->command);
			//INFO("Trying %s\r\n", ce->command);
			if(util_match_stringi(dataBuf, ce->command, cmdlen)){
				if(CP_NONE == ce->type){ // Parameterless command
					switch(i){
						case CMD_OFF:
							relaySet(OFF);
							break;
					
						case CMD_ON:
							relaySet(ON);
							break;
					
						case CMD_TOGGLE:
							relayToggle();
							break;
								
						default:
							util_assert(0, "Unsupported command: %d", i);
					}
					break;
				}
			}
			
			if((CP_INT == ce->type) || (CP_BOOL == ce->type)){ // Integer/bool parameter
				if(util_parse_command_int(dataBuf, ce->command, &ce->p.i)){
					if(CMD_PULSE == i){ /* Pulse relay */
						relaySet(ON);
						os_timer_arm(&pulse_timer, ce->p.i, 0);
						break;
					}
					if(CMD_MQTTBTLOCAL == i){ /* option: local button toggles relay */
						ce->p.i = (ce->p.i) ? 1: 0;
						kvstore_update_number(configHandle, ce->command, ce->p.i);
						break;
						
					}
					
				}
							
			}		
		} /* END for */
	} /* END if topic test */
				
	util_free(topicBuf);
	util_free(dataBuf);
}

/*
 * Pulse timer callback function
 */
 
LOCAL void ICACHE_FLASH_ATTR pulseTmerExpireCb(void *arg)
{
		relaySet(OFF);
}


/*
 * Check for button state change
 */
 
LOCAL void ICACHE_FLASH_ATTR buttonTimerCb(void *arg)
{
	int newstate =  GPIO_INPUT_GET(BUTTON_GPIO);
	
	if(newstate != button_state){
		if(newstate){
			char result[32];
			
			if(commandElements[CMD_MQTTBTLOCAL].p.i) // If local control enabled
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

LOCAL void ICACHE_FLASH_ATTR relayInit(void)
{
	gpio_init();
		
	// Initialize relay GPIO as an output
	PIN_FUNC_SELECT(RELAY_GPIO_MUX, RELAY_GPIO_FUNC);
	GPIO_OUTPUT_SET(RELAY_GPIO, RELAY_OFF); // Relay off initially
	
	
	// Uart init
	uart0_init(BIT_RATE_115200);

	os_delay_us(2000000); // To allow gtkterm to come up
	

	// Read in the config sector from flash
	configHandle = kvstore_open(KVS_DEFAULT_LOC);
	
	// Check for default configuration overrides
	if(!kvstore_exists(configHandle, commandElements[CMD_MQTTBTLOCAL].command)){
		kvstore_put(configHandle, commandElements[CMD_MQTTBTLOCAL].command, configInfoBlock.e[CMD_MQTTBTLOCAL].value);
	}

	// Get the configurations we need from the KVS
	
	kvstore_get_integer(configHandle,  commandElements[CMD_MQTTBTLOCAL].command, &commandElements[CMD_MQTTBTLOCAL].p.i);

	// Write the KVS back out to flash	
	
	kvstore_flush(configHandle);
	
	// Initialize MQTT connection 
	
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
	
	// Subtopics
	commandTopic = util_make_sub_topic(configInfoBlock.e[MQTTRTOPIC].value, "command");
	statusTopic = util_make_sub_topic(configInfoBlock.e[MQTTRTOPIC].value, "status");
	

	// Timers
	os_timer_disarm(&pulse_timer);
	os_timer_setfn(&pulse_timer, (os_timer_func_t *)pulseTmerExpireCb, (void *)0);
	os_timer_disarm(&button_timer);
	os_timer_setfn(&button_timer, (os_timer_func_t *)buttonTimerCb, (void *)0);

	// Attempt WIFI connection
	
	char *ssid = configInfoBlock.e[WIFISSID].value;
	char *wifipass = configInfoBlock.e[WIFIPASS].value;
	
	INFO("Attempting connection with: %s\r\n", ssid);
	INFO("Command subtopic: %s\r\n", commandTopic);
	INFO("Status subtopic: %s\r\n", statusTopic);
	
	// Attempt to connect to AP
	WIFI_Connect(ssid, wifipass, wifiConnectCb);
	
	// Sample button every 100 mSec
	
	os_timer_arm(&button_timer, 100, 1);
	
	INFO("\r\nSystem started ...\r\n");

}

/*
 * Called from startup
 */
 
void user_init(void)
{
	relayInit();
}

