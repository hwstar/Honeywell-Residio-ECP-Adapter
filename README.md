This firmware allows the SP8 Alarm Panel to talk to Honeywell/Residio 6160 and 6139 keypads. 
It translates the CBUS protocol from the SP8 alarm panel into the Honeywell/Residio ECP protocol.
This firmware is intended to be loaded into the KPA1 adapter board. The code is compiled using
platformio and Visual Code Studio. It will need to be downloaded into the board using a suitable
JTAG adapter. 

The firmware is meant to be run on a circuit board that I produce and sell on [Tindie](https://www.tindie.com/products/home-control-labs/8-zone-sensor-panel-for-esphome/) along with the 
SP8 alarm panel. 

![Circuit Board](https://github.com/hwstar/Honeywell-Residio-ECP-Adapter/assets/board.jpg)


For information on the SP8 alarm panel please refer to this [repository](https://github.com/hwstar/8-zone-sensor-panel)
