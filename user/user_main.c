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

// API includes
#include "ets_sys.h"
#include "osapi.h"
#include "debug.h"
#include "gpio.h"
#include "user_interface.h"
#include "mem.h"
// Project includes
#include "driver/uart.h"
#include "mqtt.h"
#include "wifi.h"
#include "easygpio.h"
#include "util.h"
#include "kvstore.h"

/* Operational modes */

#define STANDARD 0		// Single output
#define LATCHING 1		// Pulsed dual output (latching relay use)

#ifndef MODE			// In initially udefined,
#define MODE STANDARD	// Set  mode of operation
						// either STANDARD OR LATCHING
#endif

// Include connection state LED code if defined

#define WITH_LED		


#if STANDARD==MODE // GPIO setup for standard

#define RELAY_GPIO 12				// GPIO for relay control output

#define BUTTON_GPIO 0				// GPIO to use for button

#define LED_GPIO 14					// GPIO to use for LED

#define RELAY_ON 0		// Low true relay outputs
#define RELAY_OFF 1


#endif // End standard setup

#if LATCHING==MODE // Gpio setup for latching

#define RELAY_SET_GPIO 13			// GPIO for set coil on bistable relay
#define RELAY_CLEAR_GPIO 12			// GPIO for clear coil on bistable relay

#define BUTTON_GPIO 0				// GPIO to use for button

#define LED_GPIO 14					// GPIO to use for LED

#define RELAY_ON 0		// Low true relay outputs
#define RELAY_OFF 1


#endif // End latching setup


#ifdef WITH_LED // LED configuration

#define LED_ON 0		// Low true LED output
#define LED_OFF 1

#define LED_FLASH_COUNT 2 // 400mSec LED flash period (n*100)*2

#endif // End WITH_LED

/* General definitions */

#define ON 1
#define OFF 0

#define MAX_INFO_ELEMENTS 16			// Patcher number of elements
#define INFO_BLOCK_MAGIC 0x3F2A6C17		// Patcher magic
#define INFO_BLOCK_SIG "ESP8266HWSTARSR"// Patcher pattern
#define CONFIG_FLD_REQD 0x01			// Patcher field required flag

// Definition for a patcher config element

struct config_info_element_tag{
	uint8_t flags;
	uint8_t key[15];
	uint8_t value[80];
}  __attribute__((__packed__));

typedef struct config_info_element_tag config_info_element;

// Definition for a patcher config element

struct config_info_block_tag{
	uint8_t signature[16];
	uint32_t magic;
	uint8_t numelements;
	uint8_t recordLength;
	uint8_t pad[10];
	config_info_element e[MAX_INFO_ELEMENTS];
}  __attribute__((__packed__));

// Definition of a common element for MQTT command parameters

typedef union {
	char *sp;
	unsigned u;
	int i;
} pu;

// Definition of an MQTT command element

typedef struct {
	const char *command;
	uint8_t type;
	pu p;
} command_element;


typedef struct config_info_block_tag config_info_block;

// Definition of command codes and types
enum {WIFISSID=0, WIFIPASS, MQTTHOST, MQTTPORT, MQTTSECUR, MQTTDEVID, MQTTCLNT, MQTTPASS, MQTTKPALIV, MQTTDEVPATH, MQTTBTLOCAL};
enum {CP_NONE= 0, CP_INT, CP_BOOL, CP_QSTRING};
 
/* Local storage */

// Patcher configuration information


LOCAL config_info_block configInfoBlock = {
	.signature = INFO_BLOCK_SIG,
	.magic = INFO_BLOCK_MAGIC,
	.numelements = MAX_INFO_ELEMENTS,
	.recordLength = sizeof(config_info_element),
	.e[WIFISSID] = {.flags = CONFIG_FLD_REQD, .key = "WIFISSID", .value="your_ssid_here"},
	.e[WIFIPASS] = {.flags = CONFIG_FLD_REQD, .key = "WIFIPASS", .value="its_a_secret"},
	.e[MQTTHOST] = {.flags = CONFIG_FLD_REQD, .key = "MQTTHOST", .value="your_mqtt_broker_hostname_here"}, // May also be an IP address
	.e[MQTTPORT] = {.key = "MQTTPORT", .value="1883"}, // destination Port for mqtt broker
	.e[MQTTSECUR] = {.key = "MQTTSECUR",.value="0"}, // Security 0 - no encryption
	.e[MQTTDEVID] = {.key = "MQTTDEVID", .value="your_mqtt_device_id_here"}, // Only relevant if MQTTSECUR is other than 0
	.e[MQTTCLNT] = {.key = "MQTTCLNT", .value="your_mqtt_client_name_here"}, // Only relevant if MQTTSECUR is other than 0
	.e[MQTTPASS] = {.key = "MQTTPASS", .value="its_a_secret"},// Only relevant if MQTTSECUR is other than 0
	.e[MQTTKPALIV] = {.key = "MQTTKPALIV", .value="120"}, // Keepalive interval
	.e[MQTTDEVPATH] = {.flags = CONFIG_FLD_REQD, .key = "MQTTDEVPATH", .value = "/home/lab/relay"}, // Device path
	.e[MQTTBTLOCAL] = {.key = "MQTTBTLOCAL", .value = "1"} // Optional local toggle control using GPIO0
};

// Command elements 
// Additional commands are added here
 
enum {CMD_OFF = 0, CMD_ON, CMD_TOGGLE, CMD_PULSE, CMD_BTLOCAL, CMD_QUERY, CMD_SURVEY, CMD_SSID, CMD_RESTART, CMD_WIFIPASS, CMD_CYCLE};

LOCAL command_element commandElements[] = {
	{.command = "off", .type = CP_NONE},
	{.command = "on", .type = CP_NONE},
	{.command = "toggle", .type = CP_NONE},
	{.command = "pulse", .type = CP_INT},
	{.command = "btlocal", .type = CP_INT},
	{.command = "query", .type = CP_NONE},
	{.command = "survey", .type = CP_NONE},
	{.command = "ssid", .type = CP_QSTRING},
	{.command = "restart",.type = CP_NONE},
	{.command = "wifipass",.type = CP_QSTRING},
	{.command = "cycle",.type = CP_INT},
	{.command = ""} /* End marker */
};
	
// Misc Local variables 

LOCAL int relayState = OFF;

#if MODE==LATCHING
typedef enum {LR_IDLE = 0, LR_SET, LR_CLEAR, LR_DONE} lrstatetype;
LOCAL lrstatetype lrState;
#endif

#ifdef WITH_LED
typedef enum {LC_OFF = 0, LC_ON, LC_FLASH, LC_FLASH_OFF} lcstatetype;
LOCAL lcstatetype lcState;
#endif

LOCAL int buttonState = 1;
LOCAL char *commandTopic, *statusTopic;
LOCAL char *controlTopic = "/node/control";
LOCAL char *infoTopic = "/node/info";
LOCAL flash_handle_s *configHandle;
LOCAL os_timer_t pulseTimer, buttonTimer;

MQTT_Client mqttClient;			// Control block used by MQTT functions


/**
 * Publish connection info
 */
LOCAL void ICACHE_FLASH_ATTR publishConnInfo(MQTT_Client *client)
{
	struct ip_info ipConfig;
	char *buf = util_zalloc(256);	
		
	// Publish who we are and where we live
	wifi_get_ip_info(STATION_IF, &ipConfig);
	os_sprintf(buf, "muster{connstate:online,device:%s,ip4:%d.%d.%d.%d,schema:hwstar_relaynode,ssid:%s}",
			configInfoBlock.e[MQTTDEVPATH].value,
			*((uint8_t *) &ipConfig.ip.addr),
			*((uint8_t *) &ipConfig.ip.addr + 1),
			*((uint8_t *) &ipConfig.ip.addr + 2),
			*((uint8_t *) &ipConfig.ip.addr + 3),
			commandElements[CMD_SSID].p.sp);

	INFO("MQTT Node info: %s\r\n", buf);

	// Publish
	MQTT_Publish(client, infoTopic, buf, os_strlen(buf), 0, 0);
	
	// Free the buffer
	util_free(buf);
	
}



/**
 * Handle qstring command
 */
 
LOCAL void ICACHE_FLASH_ATTR handleQstringCommand(char *new_value, command_element *ce)
{
	char *buf = util_zalloc(128);
	
	
	if(!new_value){
		const char *cur_value = kvstore_get_string(configHandle, ce->command);
		os_sprintf(buf, "%s:%s", ce->command, cur_value);
		util_free(cur_value);
		INFO("Query Result: %s\r\n", buf );
		MQTT_Publish(&mqttClient, statusTopic, buf, os_strlen(buf), 0, 0);
	}
	else{
		util_free(ce->p.sp); // Free old value
		ce->p.sp = util_strdup(new_value); // Copy new value to new string
		kvstore_put(configHandle, ce->command, ce->p.sp);
	}

	util_free(buf);

}

/**
 * Send MQTT message to update relay state
 */
 
LOCAL void ICACHE_FLASH_ATTR updateRelayState(int s)
{
	char *state = s ? "on" : "off";
	char result[16];
	os_strcpy(result,"relaystate:");
	os_strcat(result, state);
	INFO("MQTT: New Relay State: %s\r\n", state);
	MQTT_Publish(&mqttClient, statusTopic, result, os_strlen(result), 0, 0);

}

/**
 * Set new relay state
 */
 
LOCAL void ICACHE_FLASH_ATTR relaySet(bool new_state)
{
	relayState = new_state; // Save new state
	#if STANDARD==MODE
	GPIO_OUTPUT_SET(RELAY_GPIO, ((relayState) ? RELAY_ON : RELAY_OFF)); // Set relay GPIO
	#endif
	#if LATCHING==MODE
		if(!lrState)
			lrState = (new_state) ? LR_SET : LR_CLEAR; // Get new latching relay state
		else
			INFO("Overlapped latching relay commands not supported yet. Command ignored while busy\r\n");
	#endif	
	updateRelayState(relayState); // Send MQTT message indicating new state
}

/**
 * Toggle the relay output
 */

LOCAL void ICACHE_FLASH_ATTR relayToggle(void)
{
	bool new_relayState = (relayState) ? OFF : ON;
	relaySet(new_relayState);
}

/**
 * Update the button state
 */

LOCAL void ICACHE_FLASH_ATTR updateButtonState(int s)
{

	
	char *state = s ? "released" : "depressed";
	char result[25];
	os_strcpy(result,"buttonstate:");
	os_strcat(result, state);
	INFO("MQTT: New Button State: %s\r\n", state);
	MQTT_Publish(&mqttClient, statusTopic, result, os_strlen(result), 0, 0);
	
}

/**
 * WIFI connect call back
 */
 

LOCAL void ICACHE_FLASH_ATTR wifiConnectCb(uint8_t status)
{
	if(status == STATION_GOT_IP){
		#ifdef WITH_LED
		lcState = LC_FLASH; // Start flashing LED
		#endif
		MQTT_Connect(&mqttClient);
	}
	#ifdef WITH_LED
	else
		lcState = LC_OFF; // LED off
	#endif
	
}

/**
 * Survey complete,
 * publish results
 */


LOCAL void ICACHE_FLASH_ATTR
surveyCompleteCb(void *arg, STATUS status)
{
	struct bss_info *bss = arg;
	
	#define SURVEY_CHUNK_SIZE 128
	
	if(status == OK){
		uint8_t i;
		char *buf = util_zalloc(SURVEY_CHUNK_SIZE);
		bss = bss->next.stqe_next; //ignore first
		for(i = 2; (bss); i++){
			os_sprintf(strlen(buf)+ buf, "ap:%s;chan:%d;rssi:%d\r\n", bss->ssid, bss->channel, bss->rssi);
			bss = bss->next.stqe_next;
			buf = util_str_realloc(buf, i * SURVEY_CHUNK_SIZE); // Grow buffer
		}
		INFO("Survey Results:\r\n", buf);
		INFO(buf);
		MQTT_Publish(&mqttClient, statusTopic, buf, os_strlen(buf), 0, 0);
		util_free(buf);
	}

}


/**
 * MQTT Connect call back
 */
 
LOCAL void ICACHE_FLASH_ATTR mqttConnectedCb(uint32_t *args)
{
	
	MQTT_Client* client = (MQTT_Client*)args;


	
	INFO("MQTT: Connected\r\n");
	#ifdef WITH_LED
	lcState = LC_ON; // LED on solid
	#endif

	publishConnInfo(client);
	
	// Subscribe to the control topic
	MQTT_Subscribe(client, controlTopic, 0);
	// Subscribe to command topic
	MQTT_Subscribe(client, commandTopic, 0);
	// Publish relay state
	updateRelayState(relayState); 

}

/**
 * MQTT Disconnect call back
 */
 

LOCAL void ICACHE_FLASH_ATTR mqttDisconnectedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Disconnected\r\n");
	#ifdef WITH_LED
	lcState = LC_FLASH;
	#endif
}

/**
 * MQTT published call back
 */

LOCAL void ICACHE_FLASH_ATTR mqttPublishedCb(uint32_t *args)
{
	MQTT_Client* client = (MQTT_Client*)args;
	INFO("MQTT: Published\r\n");
}

/**
 * MQTT Data call back
 * Commands are decoded and acted upon here
 */

LOCAL void ICACHE_FLASH_ATTR 
mqttDataCb(uint32_t *args, const char* topic, uint32_t topic_len, 
const char *data, uint32_t data_len)
{
	char *topicBuf, *dataBuf;
	uint8_t i;

	MQTT_Client* client = (MQTT_Client*)args; // Pointer to MQTT control block passed in as args

	// Save local copies of the topic and data
	topicBuf = util_strndup(topic, topic_len);
	dataBuf = util_strndup(data, data_len);
	
	INFO("Receive topic: %s, data: %s \r\n", topicBuf, dataBuf);
	
	// Control Message?
	if(!os_strcmp(topicBuf, controlTopic)){
		if(util_match_stringi(dataBuf, "muster", 6)){
			publishConnInfo(&mqttClient);
		}
	}
	
	// Command Message?
	else if (!os_strcmp(topicBuf, commandTopic)){ // Check for match to command topic
		// Decode command
		for(i = 0; commandElements[i].command[0]; i++){
			command_element *ce = &commandElements[i];
			uint8_t cmdlen = os_strlen(ce->command);
			//INFO("Trying %s\r\n", ce->command);
			if(CP_NONE == ce->type){ // Parameterless command
				if(util_match_stringi(dataBuf, ce->command, cmdlen)){
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
						
						case CMD_QUERY:
							updateRelayState(relayState);
							break;
							
						case CMD_SURVEY:
							wifi_station_scan(NULL, surveyCompleteCb);
							break;
							
						case CMD_RESTART:
							util_restart();
							break;
							
						default:
							util_assert(FALSE, "Unsupported command: %d", i);
					}
					break;
				}
			}
			
			if((CP_INT == ce->type) || (CP_BOOL == ce->type)){ // Integer/bool parameter
				int arg;
				if(util_parse_command_int(dataBuf, ce->command, &arg)){
					switch(i){
						case CMD_PULSE: // Pulse rely on then off for a specific time in mSec
							relaySet(ON);
							os_timer_arm(&pulseTimer, arg, 0);
							break;
			
						case CMD_BTLOCAL: // Link or break button control from relay
							ce->p.i = (arg) ? 1: 0;
							kvstore_update_number(configHandle, ce->command, ce->p.i);
							break;
							
						case CMD_CYCLE: // Cycle relay
							if(arg && (arg < 500))
								arg = 500; // Clip to 1/2 sec half cycle
							arg = arg/100; // Convert 1ms to 100ms count
							ce->p.i = arg;
							if(!arg) // If paramter is 0, this is a request to stop cycling.
								relaySet(OFF);
							break;
							
						default:
							util_assert(FALSE, "Unsupported command: %d", i);
						}
					break;
				}				
			}
			if(CP_QSTRING == ce->type){ // Query strings
				char *val;
				if(util_parse_command_qstring(dataBuf, ce->command,  &val)){
					if((CMD_SSID == i) || (CMD_WIFIPASS == i)){ // SSID or WIFIPASS?
						handleQstringCommand(val, ce);
					}
				}
			}
			
		} /* END for */
		kvstore_flush(configHandle); // Flush any changes back to the kvs
	} /* END if topic test */
				
	// Free local copies of the topic and data strings
	util_free(topicBuf);
	util_free(dataBuf);
}

/**
 * Pulse timer callback function. This is used to time
 * the length of the relay on state after a pulse:n command
 * is received.
 */
 
LOCAL void ICACHE_FLASH_ATTR pulseTmerExpireCb(void *arg)
{
		relaySet(OFF);
}


/**
 * 100 millisecond timer callback
 * This function handles button sampling, 
 * latching relay control, and LED control
 */
 
LOCAL void ICACHE_FLASH_ATTR nodeTimerCb(void *arg)
{
	int newstate =  GPIO_INPUT_GET(BUTTON_GPIO);
	static int relayCycleTimer;
	
	if(newstate != buttonState){
		if(newstate){
			char result[32];
			
			if(commandElements[CMD_BTLOCAL].p.i) // If local control enabled
				relayToggle(); // Toggle the relay state
			INFO("Button released\r\n");
		}
		else{
			INFO("Button pressed\r\n");
		}	
		buttonState = newstate;
		updateButtonState(buttonState);
	}
	
	#if LATCHING==MODE // Handle latching relay as state machine
	if(lrState){
		switch(lrState){
	
			case LR_SET:
				GPIO_OUTPUT_SET(RELAY_SET_GPIO, RELAY_ON);
				lrState = LR_DONE;
				break;
			
			case LR_CLEAR:
				//INFO("lr_clear\r\n");
				GPIO_OUTPUT_SET(RELAY_CLEAR_GPIO, RELAY_ON);
				lrState = LR_DONE;
				break;
				
			case LR_DONE:
				//INFO("lr_done\r\n");
				GPIO_OUTPUT_SET(RELAY_SET_GPIO, RELAY_OFF);
				GPIO_OUTPUT_SET(RELAY_CLEAR_GPIO, RELAY_OFF);
				lrState = LR_IDLE;
				break;	
		
			default:
				util_assert(FALSE, "Bad latching relay state reached: %d", lrState);
			}
	}
	#endif // End LATCHING
	
	#ifdef WITH_LED
	
	switch(lcState){
		static uint8_t flashCount;
			
		case LC_OFF:
			flashCount = 0;
			GPIO_OUTPUT_SET(LED_GPIO, LED_OFF);
			break;
			
		case LC_ON:
			flashCount = 0;
			GPIO_OUTPUT_SET(LED_GPIO, LED_ON);
			break;
				
		case LC_FLASH:
			if(flashCount)
				flashCount--;
			else{
				GPIO_OUTPUT_SET(LED_GPIO, LED_OFF);
				flashCount = LED_FLASH_COUNT;
				lcState = LC_FLASH_OFF;
			}
			break;
			
		case LC_FLASH_OFF:
			if(flashCount)
				flashCount--;
			else{
				GPIO_OUTPUT_SET(LED_GPIO, LED_ON);
				flashCount = LED_FLASH_COUNT;
				lcState = LC_FLASH;
			}
			break;		
			
		default:
			util_assert(FALSE, "Bad LED control state reached: %d", lcState);
			break;
		}
	
	#endif	// End WITH_LED
	
	// For relay cycle
	if(commandElements[CMD_CYCLE].p.i){
		if(!relayCycleTimer){
			relayCycleTimer = commandElements[CMD_CYCLE].p.i;
			relayToggle();
		}
		else relayCycleTimer--;
	}
	else if(relayCycleTimer)
		relayCycleTimer = 0;
}

/**
 * System initialization
 * Called once from user_init
 */

LOCAL void ICACHE_FLASH_ATTR sysInit(void)
{

	char *buf = util_zalloc(256); // Working buffer
	
	// I/O initialization
	gpio_init();
	
	#if STANDARD==MODE	 // Standard init
	// Initialize relay GPIO as an output
	easygpio_pinMode(RELAY_GPIO, EASYGPIO_NOPULL, EASYGPIO_OUTPUT);
	// Initialize output state
	GPIO_OUTPUT_SET(RELAY_GPIO, RELAY_OFF);
	
	// Initialize button GPIO input
	easygpio_pinMode(BUTTON_GPIO, EASYGPIO_PULLUP, EASYGPIO_INPUT);
	
	#endif // End standard init
	
	#if LATCHING==MODE // Latching Init
	// Initialize relay GPIO as an output
	easygpio_pinMode(RELAY_SET_GPIO, EASYGPIO_NOPULL, EASYGPIO_OUTPUT);
	easygpio_pinMode(RELAY_CLEAR_GPIO, EASYGPIO_NOPULL, EASYGPIO_OUTPUT);
	
	// Initialize output state
	GPIO_OUTPUT_SET(RELAY_SET_GPIO, RELAY_OFF);
	GPIO_OUTPUT_SET(RELAY_CLEAR_GPIO, RELAY_OFF);
	
	// Initialize button GPIO input
	easygpio_pinMode(BUTTON_GPIO, EASYGPIO_PULLUP, EASYGPIO_INPUT);
	#endif  // End Latching Init
	
	// LED output setup
	#ifdef WITH_LED
	easygpio_pinMode(LED_GPIO, EASYGPIO_NOPULL, EASYGPIO_OUTPUT);
	GPIO_OUTPUT_SET(LED_GPIO, LED_OFF);
	#endif // End LED setup
	
	
	// Uart init
	uart0_init(BIT_RATE_115200);

	os_delay_us(2000000); // To allow gtkterm to come up
	

	// Read in the config sector from flash
	configHandle = kvstore_open(KVS_DEFAULT_LOC);
	

	const char *ssidKey = commandElements[CMD_SSID].command;
	const char *WIFIPassKey = commandElements[CMD_WIFIPASS].command;
	const char *btLocalKey = commandElements[CMD_BTLOCAL].command;

	// Check for default configuration overrides
	if(!kvstore_exists(configHandle, ssidKey)){ // if no ssid, assume the rest of the defaults need to be set as well
		kvstore_put(configHandle, btLocalKey, configInfoBlock.e[MQTTBTLOCAL].value);
		kvstore_put(configHandle, ssidKey, configInfoBlock.e[WIFISSID].value);
		kvstore_put(configHandle, WIFIPassKey, configInfoBlock.e[WIFIPASS].value);

		// Write the KVS back out to flash	
	
		kvstore_flush(configHandle);
	}
	
	// Get the configurations we need from the KVS and store them in the commandElement data area
	
	kvstore_get_integer(configHandle, btLocalKey, &commandElements[CMD_BTLOCAL].p.i); // Retrieve button local
	
	commandElements[CMD_SSID].p.sp = kvstore_get_string(configHandle, ssidKey); // Retrieve SSID
	
	commandElements[CMD_WIFIPASS].p.sp = kvstore_get_string(configHandle, WIFIPassKey); // Retrieve WIFI Pass

	
	// Initialize MQTT connection 
	
	uint8_t *host = configInfoBlock.e[MQTTHOST].value;
	uint32_t port = (uint32_t) atoi(configInfoBlock.e[MQTTPORT].value);
	
	MQTT_InitConnection(&mqttClient, host, port,
	(uint8_t) atoi(configInfoBlock.e[MQTTSECUR].value));

	MQTT_InitClient(&mqttClient, configInfoBlock.e[MQTTDEVID].value, 
	configInfoBlock.e[MQTTCLNT].value, configInfoBlock.e[MQTTPASS].value,
	atoi(configInfoBlock.e[MQTTKPALIV].value), 1);

	MQTT_OnConnected(&mqttClient, mqttConnectedCb);
	MQTT_OnDisconnected(&mqttClient, mqttDisconnectedCb);
	MQTT_OnPublished(&mqttClient, mqttPublishedCb);
	MQTT_OnData(&mqttClient, mqttDataCb);
	
	// Last will and testament

	os_sprintf(buf, "muster{connstate:offline,device:%s}", configInfoBlock.e[MQTTDEVPATH].value);
	MQTT_InitLWT(&mqttClient, infoTopic, buf, 0, 0);

	// Subtopics
	commandTopic = util_make_sub_topic(configInfoBlock.e[MQTTDEVPATH].value, "command");
	statusTopic = util_make_sub_topic(configInfoBlock.e[MQTTDEVPATH].value, "status");
	INFO("Command subtopic: %s\r\n", commandTopic);
	INFO("Status subtopic: %s\r\n", statusTopic);
	

	// Timers
	os_timer_disarm(&pulseTimer);
	os_timer_setfn(&pulseTimer, (os_timer_func_t *)pulseTmerExpireCb, (void *)0);
	os_timer_disarm(&buttonTimer);
	os_timer_setfn(&buttonTimer, (os_timer_func_t *)nodeTimerCb, (void *)0);

	// Attempt WIFI connection
	
	char *wifipass = commandElements[CMD_WIFIPASS].p.sp;
	char *ssid = commandElements[CMD_SSID].p.sp;
	
	INFO("Attempting connection with: %s\r\n", ssid);
	
	// Attempt to connect to AP
	WIFI_Connect(ssid, wifipass, wifiConnectCb);
	
	// Timer to execute code every 100 mSec
	
	os_timer_arm(&buttonTimer, 100, 1);
	
	#if LATCHING==MODE
	lrState = LR_CLEAR; // Latching relay to off state 
	#endif
	
	// Free working buffer
	util_free(buf);
	
	INFO("\r\nSystem started ...\r\n");

}

/**
 * Called from startup
 */
 
void user_init(void)
{
	sysInit();
}

