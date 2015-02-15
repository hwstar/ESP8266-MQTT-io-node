**esp_8266_MQTT_io_node**
==========
This is an implementation of an MQTT Relay Node 

**Features:**

Provides one relay channel on GPIO12 and one button channel on GPIO0. Button channel can be linked to the relay for local control or isolated for separate use.

**Commands**

MQTT commands supported:

|Command| Description |
|-------| ----------- |
|ON 	| Turns relay on|
|OFF	| Turns relay off|
|PULSE:n| Pulses relay for N milliseconds|
|TOGGLE	| Toggles relay state|
|QUERY	| Returns relay state|
|SURVEY	| Returns WIFI survey information as seen by the node|

**Power on Message**

After booting, the node posts a message to /node/info with the following data:

|Field		| Description|
|-----      | -----------|
|ROOT TOPIC	| A root topic name (e.g. /home/lab/relay)|
|IP ADDRESS	| The IP address assigned to the node|
|SCHEMA		| A schema name of hwstar.relaynode (vendor.product ala xPL)|


The schema may be used to design a database of supported commands for each device:

root:/home/lab/relay;ip4:127.0.0.1;schema:hwstar.relaynode

The root topic encompasses subtopics command and status. Commands are sent to $ROOT_TOPIC/command (which the nodes subscribes to.) Status messages are
published by the node on $ROOT_TOPIC/status. The ROOT_TOPIC is set using the Configuration procedure described below.

**Status Messages**

Status messages which can be published:

* BUTTONSTATE:DEPRESSED
* BUTTONSTATE:RELEASED
* RELAYSTATE:ON
* RELAYSTATE:OFF

WIFI Survey Data in the following format:
AP: $AP, CHAN: $CHAN, RSSI: $RSSI

Can be multiple lines. One entry per line. 

**Configuration Patcher**

NB: WIFI and MQTT Configration is not stored in the source files. It is patched in using a custom Python utility which is available on my github account as
a separate project:


https://github.com/hwstar/ESP8266-MQTT-config-patcher

Post patching allows the configuration to be changed without having sensitive information in the source files.

**Electrical Details**

The code is configured to be used with an ESP module which has GPIO12. It can be reconfigured, but use of GPIO2 is problematic as that pin needs to be
high during boot, and that makes the electrical interface more complex.

The output of GPIO12 is low true to be compatible with the bootloader's initial pin states. 
(This prevents the relay from pulsing at power on).

**Notes**

Supports Linux build host only at this time.
Requires the ESP8266 toolchain be installed on the Linux system per the instructions available here:
https://github.com/esp8266/esp8266-wiki/wiki/Toolchain

**LICENSE - "MIT License"**

Copyright (c) 2015 Stephen Rodgers 
Copyright (c) 2014-2015 Tuan PM, https://twitter.com/TuanPMT

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
