// Arduino Bluetooth Low Energy demonstration sketch
// GAP Slave (aka Peripheral) - GATT Server
// 03-23-2015 by Jim Wootten
// Updates should be available at https://github.com/BlueEmeraldSystems/BLE-Arduino-Shield
//
// Changelog:
//      03-28-2015 - Updated comments
//      03-23-2015 - Updated comments
//      03-22-2015 - Initial release

/* ============================================
 Arduino Bluetooth Low Energy code is placed under the MIT license
 Copyright (c) 2015 Jim Wootten
 
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 ===============================================
 */

// uncomment this line for Serial.println() debug output
//#define DEBUG

#include <SoftwareSerial.h>
#include "BGLib.h"

// BLE112 module connections:
// - BLE P0_4  -> Arduino Digital Pin 2 (BLE TX -> Arduino soft RX)
// - BLE P0_5  -> Arduino Digital Pin 3 (BLE RX -> Arduino soft TX)
// - BLE Reset -> Arduino Digital Pin 4  (BLE Reset -> Arduino I/O 4)
// or via solder pad jumpers on Arduino Bluetooth Low Energy shield
// - BLE P0_4  -> Arduino Digital Pin 10 (BLE TX -> Arduino soft RX)
// - BLE P0_5  -> Arduino Digital Pin 11 (BLE RX -> Arduino soft TX)
// - BLE Reset -> Arduino Digital Pin 12 (BLE Reset -> Arduino I/O 12)
//
// - Arduino Bluetooth Low Energy Shield LED1 -> BLE P0_0 (turn on/off via BGLib)
//

#define ARDUINO_RX_PIN  2  // Arduino software serial port - RX
#define ARDUINO_TX_PIN  3  // Arduino software serial port - TX
#define BLE_RESET_PIN   4  // BLE Shield Reset
#define ARDUINO_LED_PIN 13  // Arduino Uno LED
#define ARDUINO_SPEED 38400 // Arduino hardware serial port speed (baud rate) for the serial communication to PC 
#define BLE_SPEED     19200 // Arduino software serial port speed (baud rate) for the serial communication to BLE Shield
#define GATT_HANDLE_C_RX_DATA   32  // supports "write" operation
#define GATT_HANDLE_C_TX_DATA   36  // supports "read" and "indicate" operations

SoftwareSerial bleSerialPort(ARDUINO_RX_PIN, ARDUINO_TX_PIN); // receive, transmit
BGLib ble112((HardwareSerial *)&bleSerialPort, 0, 1); // Initialize Bluegiga's BGLib

void setup() {
  // open Arduino USB serial (and wait, if we're using Leonardo)
  Serial.begin(ARDUINO_SPEED);
  while (!Serial);

  // welcome!
  Serial.println(F("\n\nArduino Bluetooth Low Energy Demo\n"));

  // initialize Arduino status LED and turn it off
  pinMode(ARDUINO_LED_PIN, OUTPUT);
  digitalWrite(ARDUINO_LED_PIN, LOW);

  // set the data rate for the SoftwareSerial port
  bleSerialPort.begin(BLE_SPEED);

  // set up internal status handlers
  ble112.onBusy = onBusy;
  ble112.onIdle = onIdle;
  ble112.onTimeout = onTimeout;

  // set up BGLib response handlers (called almost immediately after sending commands)
  ble112.ble_rsp_system_hello = my_rsp_system_hello;

  // set up BGLib event handlers (called at unknown times)
  ble112.ble_evt_system_boot = my_evt_system_boot;
  ble112.ble_evt_attributes_value = my_ble_evt_attributes_value;

  // reset the BLE112
  pinMode(BLE_RESET_PIN, OUTPUT);
  digitalWrite(BLE_RESET_PIN, LOW);
  delay(100); // wait 100ms
  digitalWrite(BLE_RESET_PIN, HIGH);
  delay(1000); // wait 1 sec

  // configure BLE112 io ports and turn shield LED off
  setupBLE112();        
}

void setupBLE112() {
  // turn shield LED off
  ble112.ble_cmd_hardware_io_port_config_direction(0, 0x09); // configure port 0 bits 0 and 4 as output
  while (ble112.checkActivity(1000)); // response should come back within milliseconds
  ble112.ble_cmd_hardware_io_port_write(0, 0x01, 0x00); // set port 0 pin 0 to low
  while (ble112.checkActivity(1000)); // response should come back within milliseconds
}

void loop() {
  displayMenu();
  while (1) {
    // check for software serial overflow
    if (bleSerialPort.overflow()) {
      Serial.println(F("\n!!! SoftwareSerial overflow!"));
    }
    
    // keep polling for new data from BLE112
    ble112.checkActivity();

    // check for input from the user
    if (Serial.available()) {
      uint8_t ch = Serial.read();
      if (ch == '0') {
        // hardware reset BLE112
        Serial.println(F("--> hardware reset"));
        digitalWrite(BLE_RESET_PIN, LOW);
        delay(100); // wait 100ms
        digitalWrite(BLE_RESET_PIN, HIGH);
        delay(1000); // wait 500ms 
        // configure BLE112 io ports and turn shield LED off
        setupBLE112();        
      }
      else if (ch == '1') {
        // say hello to the BLE112 and wait for response
        Serial.println(F("--> system hello"));
        ble112.ble_cmd_system_hello();
        while (ble112.checkActivity(1000)); // response should come back within milliseconds
      }
      else if (ch == '2') {
        // start advertising general discoverable / undirected connectable
        Serial.println(F("--> start advertising"));
        ble112.ble_cmd_gap_set_mode(BGLIB_GAP_GENERAL_DISCOVERABLE, BGLIB_GAP_UNDIRECTED_CONNECTABLE);
        while (ble112.checkActivity(1000)); // response should come back within milliseconds
      }
      else if (ch == '3') {
        // write data
        Serial.println(F("--> write data: "));
        uint8 write_data[] = {'T', 'e', 's', 't'};
        ble112.ble_cmd_attributes_write( GATT_HANDLE_C_RX_DATA, 0, sizeof(write_data), write_data);
        while (ble112.checkActivity(1000)); // response should come back within milliseconds
       }
      else if (ch == '4') {
        // read data
        Serial.println(F("--> read data is done automatically by callback function my_ble_evt_attributes_value()"));
      }
      else if (ch == '5') {
        // stop advertising
        Serial.println(F("--> stop advertising"));
        ble112.ble_cmd_gap_set_mode(BGLIB_GAP_NON_DISCOVERABLE, BGLIB_GAP_NON_CONNECTABLE);
        while (ble112.checkActivity(1000)); // response should come back within milliseconds
      }
      else if (ch == '6') {
        // turn shield LED on
        Serial.println(F("--> LED on"));
        ble112.ble_cmd_hardware_io_port_write(0, 0x01, 0x1); // set port 0 pin 0 to high
        while (ble112.checkActivity(1000)); // response should come back within milliseconds
      }
      else if (ch == '7') {
        // turn shield LED off 
        Serial.println(F("--> LED off"));
        ble112.ble_cmd_hardware_io_port_write(0, 0x01, 0x00); // set port 0 pin 0 to low
        while (ble112.checkActivity(1000)); // response should come back within milliseconds
      }
      else if (ch == '8') {
        // get amount of free RAM on Arduino
        Serial.print(F("--> Free Ram: "));
        Serial.print(freeRam());
        Serial.println(F(" bytes"));
      }
      displayMenu();
    }
  }
}

void displayMenu() {
  Serial.println(F("\nOperations Menu:"));
  Serial.println(F("0) Reset BLE112 module"));
  Serial.println(F("1) Say hello to the BLE112 and wait for response"));
  Serial.println(F("2) Start advertising"));
  Serial.println(F("3) Write data"));
  Serial.println(F("4) Read data"));
  Serial.println(F("5) Stop advertising"));
  Serial.println(F("6) Turn on  BLE112 Arduino Shield LED1"));
  Serial.println(F("7) Turn off BLE112 Arduino Shield LED1"));
  Serial.println(F("8) Get amount of free RAM on Arduino"));
  Serial.println(F("Command?\n"));
}

int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

// ================================================================
// INTERNAL BGLIB CLASS CALLBACK FUNCTIONS
// ================================================================

void onBusy() {
  // turn LED on when we're busy
  digitalWrite(ARDUINO_LED_PIN, HIGH);
}

void onIdle() {
  // turn LED off when we're no longer busy
  digitalWrite(ARDUINO_LED_PIN, LOW);
}

void onTimeout() {
  Serial.println(F("\n!!! Timeout occurred!"));
}

// ================================================================
// USER-DEFINED BGLIB RESPONSE CALLBACKS
// ================================================================

void my_rsp_system_hello(const ble_msg_system_hello_rsp_t *msg) {
  Serial.println(F("<-- system hello"));
}

// ================================================================
// USER-DEFINED BGLIB EVENT CALLBACKS
// ================================================================

void my_evt_system_boot(const ble_msg_system_boot_evt_t *msg) {
  Serial.print(F("### BLE112 system boot: { "));
  Serial.print(F("Software version: ")); 
  Serial.print(msg -> major, HEX);
  Serial.print(F(".")); 
  Serial.print(msg -> minor, HEX);
  Serial.print(F(".")); 
  Serial.print(msg -> patch, HEX);
  Serial.print(F(".")); 
  Serial.print(msg -> build, HEX);
  Serial.print(F(", Link layer version: "));     Serial.print(msg -> ll_version, HEX);
  Serial.print(F(", BGAPI protocol version: ")); Serial.print(msg -> protocol_version, HEX);
  Serial.print(F(", Hardware version: "));       Serial.print(msg -> hw, HEX);
  Serial.println(F(" }"));
}

// Callback function is triggered by a write to attributes database
//
void my_ble_evt_attributes_value(const struct ble_msg_attributes_value_evt_t *msg) {
        Serial.println(F("### Data read:"));
        Serial.print(F("    { "));
        Serial.print(F("connection: "));  Serial.print(msg -> connection, HEX);
        Serial.print(F(", reason: "));    Serial.print(msg -> reason, HEX);
        Serial.print(F(", handle: "));    Serial.print(msg -> handle, HEX);
        Serial.print(F(", offset: "));    Serial.print(msg -> offset, HEX);
        Serial.print(F(", value_len: ")); Serial.print(msg -> value.len, HEX);
        Serial.print(F(", value_data: "));
        // this is a "uint8array" data type, which is a length byte and a uint8_t* pointer
        for (uint8_t i = 0; i < msg -> value.len; i++) {
            if (msg -> value.data[i] < 16) Serial.write('0');
            Serial.print(msg -> value.data[i], HEX);
        }
        Serial.println(F(" }"));
}



