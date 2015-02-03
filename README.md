**esp_8266_io_node**
==========
This is am implementation of a MQTT Relay Node using a port from: [MQTT client library for Contiki](https://github.com/esar/contiki-mqtt) (thanks)

**Features:**

Provides one relay channel on GPIO2 and one button channel on GPIO0. Button channel can be linked to the relay for local control or isolated for separate use.

Relay can be turned on, turned off, pulsed or toggled.

There is an optional separate topic for relay state changes and button changes.

WIFI and MQTT Configration is not stored in the source files. It is patched in using a custom Python utility which is available on my github account as
a separate project:

https://github.com/hwstar/ESP8266-MQTT-config-patcher

Post patching allows the configuration to be changed without having sensitive information in the source files.


** Notes **

Supports Linux build host only at this time.
Requires the ESP8266 toolchain be installed on the Linux system per the instructions available here:
https://github.com/esp8266/esp8266-wiki/wiki/Toolchain

**LICENSE - "MIT License"**

Copyright (c) 2015 Stephen Rodgers 
Copyright (c) 2014-2015 Tuan PM, https://twitter.com/TuanPMT

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
