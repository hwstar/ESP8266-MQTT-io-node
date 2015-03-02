**esp_8266_MQTT_io_node**
==========
This is an implementation of an MQTT Relay Node which runs natively on an ESP8266 ESP-03 module or other variant with enough free GPIO's.
Code is compiled using the toolchain referenced below.

**Features:**

Provides one relay channel on one GPIO or one bistable latching relay channel using 2 GPIO's and one button channel. The button channel can be linked to the relay for local control or isolated for separate use.
A GPIO can optionally be reserved for a connection state LED. GPIO ports are configurable in the source code. 

**Device Path**

The device path encompasses subtopics command and status. Commands are sent to $devicepath/command (which the nodes subscribes to.) All status messages are
published by the node on $devicepath/status except for the node configuration which is published on /node/info. The device path is set using the patching procedure described later.

**Control Messages**

Control messages are received by all nodes on /node/control. These are meant to be used to interrogate the nodes connected to the network, 
and perform other system-wide control functions.

One control message is currently supported: *muster*. This directs the node to re-send the node configuration information to /node/info. See the power on message below for further details


**Command Messages**

MQTT commands supported:

|Command| Description |
|-------| ----------- |
|ON 	| Turns relay on|
|OFF	| Turns relay off|
|PULSE:n| Pulses relay for N milliseconds|
|TOGGLE	| Toggles relay state|
|QUERY	| Returns relay state|
|SURVEY	| Returns WIFI survey information as seen by the node|
|BTLOCAL:n| 1 = link button to relay toggle, 0 = keep button separate|
|SSID:n| Query or set SSID|
|RESTART| Restart system|
|WIFIPASS:n| Query or set WIFI Password|
|CYCLE:n| Start or stop relay cycling (flashing)

Notes:

* Sending an SSID, or WIFIPASS command without a parameter will return the current value
* SSID:n, WIFIPASS:n change not effective until next system restart
* CYCLE:n Where n is the time of a half cycle in milliseconds

**Status Messages**

Status messages which can be published:

* buttonstate:depressed
* buttonstate:released
* relaystate:on
* relaystate:off
* WIFI survey data in the following format: ap:$AP;chan:$CHAN;rssi:$RSSI. Can be multiple lines. One entry per line. 

**Power on Message**

After booting, the node posts a message to /node/info with the following data:

|Field		| Description|
|-----      | -----------|
|CONNSTATE  | Connection state (online)
|DEVICE		| A device path (e.g. /home/lab/relay)|
|IP ADDRESS	| The IP address assigned to the node|
|SCHEMA		| A schema name of hwstar.relaynode (vendor.product ala xPL)|
|SSID       | SSID utilized|


The schema may be used to design a database of supported commands for each device.

Here is an example:

muster{connstate:online,device:/home/lab/relay,ip4:127.0.0.1,schema:hwstar.relaynode,ssid:yourssid}

**Last Will and Testament**

The following will be published to /node/info if the node is not heard from by the MQTT broker:

connstate:offline;device:$device

Where $device is the configured device path.

**Configuration Patcher**

NB: WIFI and MQTT Configration is not stored in the source files. It is patched in using a custom Python utility which is available on my github account as
a separate project:

https://github.com/hwstar/ESP8266-MQTT-config-patcher

Post patching allows the configuration to be changed without having sensitive information in the source files.

**Electrical Details**

The code is configured to be used with an ESP module with 1 uncommitted I/O for standard mode and 2 GPIO's for bistable latching mode. GPIO12 is the default. It can be reconfigured, but use of GPIO2 is problematic as that pin needs to be
high during boot, and that makes the electrical interface more complex.

The relay GPIO outputs are low true to be compatible with the bootloader's initial pin states. 
(This prevents the relay from pulsing at power on).

**Toolchain**

Requires the ESP8266 toolchain be installed on the Linux system per the instructions available here:
https://github.com/esp8266/esp8266-wiki/wiki/Toolchain

NB:Current Makefile supports Linux build hosts only at this time.

**LICENSE - "MIT License"**

Copyright (c) 2015 Stephen Rodgers 
Copyright (c) 2014-2015 Tuan PM, https://twitter.com/TuanPMT

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
