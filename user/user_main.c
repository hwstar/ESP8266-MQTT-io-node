/* main.c -- MQTT client example
*
* Copyright (c) 2014-2015, Tuan PM <tuanpm at live dot com>
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of Redis nor the names of its contributors may be used
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

enum {WIFISSID=0, WIFIPASS, MQTTHOST, MQTTPORT, MQTTSECUR, MQTTDEVID, MQTTCLNT, MQTTPASS, MQTTKPALIV,MQTTTOPIC};


/* Configuration block */

LOCAL config_info_block configInfoBlock = {
	.signature = INFO_BLOCK_SIG,
	.magic = INFO_BLOCK_MAGIC,
	.numelements = MAX_INFO_ELEMENTS,
	.recordLength = sizeof(config_info_element),
	.e[WIFISSID] = {.flags = CONFIG_FLD_REQD, .key = "WIFISSID", .value="tbd"},
	.e[WIFIPASS] = {.flags = CONFIG_FLD_REQD, .key = "WIFIPASS", .value="tbd"},
	.e[MQTTHOST] = {.flags = CONFIG_FLD_REQD, .key = "MQTTHOST", .value="tbd"},
	.e[MQTTPORT] = {.key = "MQTTPORT", .value="1880"},
	.e[MQTTSECUR] = {.key = "MQTTSECUR",.value="0"},
	.e[MQTTDEVID] = {.key = "MQTTDEVID", .value="DEVID"},
	.e[MQTTCLNT] = {.key = "MQTTCLNT", .value="MYUSER"},
	.e[MQTTPASS] = {.key = "MQTTPASS", .value="MYPASS"},
	.e[MQTTKPALIV] = {.key = "MQTTKPALIV", .value="120"},
	.e[MQTTTOPIC] = {.flags = CONFIG_FLD_REQD, .key = "MQTTTOPIC", .value = "tbd"}
};
	

LOCAL int relay_state = OFF;

MQTT_Client mqttClient;

void wifiConnectCb(uint8_t status)
{
	if(status == STATION_GOT_IP){
		MQTT_Connect(&mqttClient);
	}
}
void mqttConnectedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Connected\r\n");
	MQTT_Subscribe(client, configInfoBlock.e[MQTTTOPIC].value, 0);

}

void mqttDisconnectedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Disconnected\r\n");
}

void mqttPublishedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Published\r\n");
}

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
		if((os_strcmp(dataBuf, "on") == 0) || (os_strcmp(dataBuf, "ON") == 0)){
			GPIO_OUTPUT_SET(RELAY_GPIO, 1);
			relay_state = ON;
			INFO("MQTT: Relay on\r\n");

		}
		else if((os_strcmp(dataBuf, "off") == 0) || (os_strcmp(dataBuf, "OFF") == 0)){
			GPIO_OUTPUT_SET(RELAY_GPIO, 0);
			relay_state = OFF;
			INFO("MQTT: Relay off\r\n");
			
		}
	}
				
	os_free(topicBuf);
	os_free(dataBuf);
}


void user_init(void)
{
	gpio_init();

	
			
	// Initialize relay GPIO
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


	/* Attempt WIFI connection */
	
	uint8_t *ssid = configInfoBlock.e[WIFISSID].value;
	uint8_t *wifipass = configInfoBlock.e[WIFIPASS].value;
	
	INFO("Attempting connection with: %s\r\n", ssid);
	WIFI_Connect(ssid, wifipass, wifiConnectCb);

	INFO("\r\nSystem started ...\r\n");
}
