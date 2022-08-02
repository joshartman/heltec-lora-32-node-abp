# Heltec LoRa 32 LoRaWAN Node

Example code for a LoRaWAN Node using a Heltec LoRa 32 with OLED display.

Implemented the following:
* LMIC for the LoRaWAN stack
* U2g2 OLED driver to display temperature when sending
* OneWire to read a Dallas DS18B20 on a 1-wire bus on GPIO13

The example was coded in PlatformIO (Visual Studio Code)

The program makes use of the LMIC scheduling functions to read the DS18B20 every 30 seconds. The radio code will transmit the last read value every 5 minutes. This makes the 1-wire reading separate from the radio transmission code.

NOTE: This repo is archived as it depends on LMIC code that is also unmaintained.
