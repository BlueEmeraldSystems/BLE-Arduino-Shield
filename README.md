# BLE-Arduino-Shield
Bluetooth low energy Arduino Shield and support software

This software is designed to work with the Blue Emerald System's Bluetooth low energy Arduino Shield version 1.1.  

The Bluetooth Low Energy (BLE) Arduino Shield and this software has been tested on the following Arduino boards:

Arduino UNO R3, SparkFun RedBoard - Programmed with Arduino

Using the following version of the Arduino IDE from Arduino: v1.0.6, v1.6.2

Note, the Blue Emerald System's Bluetooth Low Energy Arduino Shield uses a BlueGiga BLE112 BLE module.

The Arduino "BLE_Generic_Read_Write.ino" sketch shows an example of how to interact with the Shield and associated BLE module and is meant for a starting point for your work.  It shows how to start and stop advertising.  It also shows how to read and write data via Bluetooth low energy.

What is needed for testing:

1. Arduino board such as Arduino UNO R3 or SparkFun RedBoard. (user provides)

2. Blue Emerald System's Bluetooth low energy Arduino Shield. (user provides)

3. BLE_Generic_Read_Write.ino Arduino Sketch.  (from this repository)

4. BGLib.h, BGLib.cpp, and BGLibConfig.h.  (from this repository or https://github.com/jrowberg/bglib/tree/master/Arduino)

5. BLE scanning apps or tools such as LightBlue (free) and other iOS apps, and nRF Master (free) and other Android apps. (user provides)

The BLE112 BLE module on board Blue Emerald System's Bluetooth Low Energy Arduino Shield comes with a "BLE113 BGAPI" firmware image flashed on them and two BLE profiles.  The profile characteristics that work with BLE_Generic_Read_Write.ino are called "Read Data" and "Write Data".
