# CyLights

[![Build Status](https://travis-ci.com/cyrusbuilt/CyLights.svg?branch=master)](https://travis-ci.com/cyrusbuilt/CyLights)

IoT Integration Control Board and Firmware for remote controlled 5-outlet system.

## Synopsis

This system was designed to integrate with the [Etekcity Zap 5-Channel Wireless Outlet Switch System](https://www.etekcity.com/product/100068) and make it controllable via software.  I originally bought the Etekcity kit so that I could turn lamps on/off using the remote in my old apartment. I quickly realized it wouldn't be difficult to modify the remote so that I could integrate it into a home automation system. So this repo contains the design files for the board, sources for the firmware, and [OpenHAB](https://www.openhab.org/) integration files. CyLights uses an [ESP8266](https://www.adafruit.com/product/2471) at it's core, and basically just uses the remote that comes with the Etekcity kit as a transmitter, making the whole thing wireless (with the exception of the power supply) so you can place it anywhere that is in range of all your wireless outlets AND your WiFi router.

## Disclaimer

CyLights requires you to remove the battery and modify the remote that comes with the Etekcity kit. I am not liable for any damage you may cause to your controller (or any other hardware for that matter) should you attempt to do this.  Also, while I am currently using this system daily, it is still a work in progress and not yet production-ready and documentation is incomplete. If you are interested in this project, check back soon!

## I'm not a n00b, get on with it!

Ok, fair enough. Consider this a quick start guide then:

1) Review the schematics and BOM in the ```schematics``` folder.

2) Send the gerbers off to your preferred PCB fab house to get the boards made, then gather up the necessary components in the BOM. I also recommend 12 female dupont cables and a 12VDC @ 1.0A power supply and 2 TO-220 heat sinks (along with thermal paste).

3) Assemble the CyLights board, then remove the battery from the Etekcity remote and disassemble it (it only has 2 small phillips head screws).

4) Cut one end off each dupont cable, strip a small amount of insulation off each one and solder each lead to the inside contact of each button on the remote (guide coming soon).

5) Connect each dupont cable to the appropriate pins on pin header J2 on the CyLights board.

6) Connect 12VDC power supply to screw terminal block J1 on the CyLights board.

7) Connect the CyLights board to your computer using an FTDI cable and put the ESP8266 in program mode.

8) Build and flash the firmware to the MCU. Then place the MCU back into program mode again. Then edit the config.json file in the ```data``` folder with the appropriate values, then upload the filesystem image. Then reset the MCU and go into monitor mode so you can see the serial output.

9) If successful, you should be able to see the CyLights board boot correctly and connect to your WiFi. If you already have an MQTT broker set up on your network and you set the appropriate parameters in the config file, it should also successfully connect to the broker and publish its first status message.

10) If you already have OpenHAB setup somewhere on your network, you can copy the files in the ```openhab``` folder to their appropriate locations (and modify as needed) and you should be able to monitor and control the system from OpenHab via the "CyLights" sitemap.

Tada! At this point, if all is well then you can unplug the FTDI cable (unless you want to continue to monitor the serial output or access the built-in configuration and control menu) and mount or install the boards wherever/however you want. I personally mounted the whole thing in an ABS plastic box for the time being, but I intend to mount it in a custom panel along with a bunch of other gear once our new house is done being built.