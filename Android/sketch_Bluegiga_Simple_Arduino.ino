
/* ============================================

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

// This software is for the BlueEmerald Arduino Shield Rev A
// There is a companion Android application which gives the Android control over I/O ports
// Digital pins 5, 6 7 and 8 are controlled as outputs
// Analog Pins 0, 1, 2 and 3 are sensed as digital inputs
//
// 04/28/2015

#include <SoftwareSerial.h>
#include <BGLib.h>
#include <BGLibConfig.h>

// Note: Debug output may reduce communication reliability due to time taken for output operations.
// Thus comment DEBUG define when debugging is completed.
// #define DEBUG

// ================================================================
// BLE STATE TRACKING (UNIVERSAL TO JUST ABOUT ANY BLE PROJECT)
// ================================================================

// BLE state machine definitions
#define BLE_STATE_STANDBY           0
#define BLE_STATE_SCANNING          1
#define BLE_STATE_ADVERTISING       2
#define BLE_STATE_CONNECTING        3
#define BLE_STATE_CONNECTED_MASTER  4
#define BLE_STATE_CONNECTED_SLAVE   5

// BLE state/link status tracker
uint8_t ble_state = BLE_STATE_STANDBY;
uint8_t ble_encrypted = 0;  // 0 = not encrypted, otherwise = encrypted
uint8_t ble_bonding = 0xFF; // 0xFF = no bonding, otherwise = bonding handle

// ================================================================
// HARDWARE CONNECTIONS AND GATT STRUCTURE SETUP
// ================================================================
#define LED_PIN         13  // Arduino Uno LED pin
#define BLE_RESET_PIN   4  // BLE reset pin (active-low)

// GATT Handles for Blue Emerald Arduino Shield hardware
#define GATT_HANDLE_C_PORT1_CFG 17
#define GATT_HANDLE_C_PORT1_DATA 20
#define GATT_HANDLE_C_PORT2_CFG 24
#define GATT_HANDLE_C_PORT2_DATA 27
#define GATT_HANDLE_C_READ_DATA 32
#define GATT_HANDLE_C_WRITE_DATA 36

// ---------------------------------------------------------
//  Port bit assignments
// ---------------------------------------------------------
// This is where the Bluetooth IO port bits are assigned
// to physical Arduino pins.
// For the demo, Port 0 is assigned to Arduino port 1 digital pins
// Port 1 is assigned to Arduino Analog input pins

// Defines mapping of bits in Bluetooth Port1 byte to Arduino pins
#define BLE_PORT0_BIT_0 5
#define BLE_PORT0_BIT_1 6
#define BLE_PORT0_BIT_2 7
#define BLE_PORT0_BIT_3 8

#define BLE_PORT0_BIT_4 9
#define BLE_PORT0_BIT_5 12
#define BLE_PORT0_BIT_6 13

#define BLE_PORT1_BIT_0 A0
#define BLE_PORT1_BIT_1 A1
#define BLE_PORT1_BIT_2 A2
#define BLE_PORT1_BIT_3 A3
#define BLE_PORT1_BIT_4 A4
#define BLE_PORT1_BIT_5 A5

#define PORT0_BIT_CNT 7
#define PORT1_BIT_CNT 6

uint8_t P0_Bits[8] = {BLE_PORT0_BIT_0, BLE_PORT0_BIT_1, BLE_PORT0_BIT_2, BLE_PORT0_BIT_3, BLE_PORT0_BIT_4, BLE_PORT0_BIT_5, BLE_PORT0_BIT_6};
uint8_t P1_Bits[6] = {BLE_PORT1_BIT_0, BLE_PORT1_BIT_1, BLE_PORT1_BIT_2, BLE_PORT1_BIT_3, BLE_PORT1_BIT_4, BLE_PORT1_BIT_5};

// Constants defined here are used in the BLE112 evt_system_boot event to initialize ble112 i/o ports. 
// P0_0 drives the blue led. 
// P0_4 and P0_5 are used by the BLE112 UART communications with the Arduino. 
// No configuration via Arduino sketch code is required for the UART, it is configured by BLE112 firmware.
const uint8_t BLE112_IO_PORT_1_CONFIG = 0x00;
const uint8_t BLE112_IO_PORT_0_CONFIG = 0x01;
    
uint8_t connectedFlag = 0;

// Port settings state store - reduces trips back to the GATT
uint8_t port1_old_bits = 0;
uint8_t port2_old_bits = 0;
uint8_t port1_config_bits = 0;
uint8_t port2_config_bits = 0;

uint8_t startUpTimer = 0;

// SoftwareSerial config - use pins D2/D3 for RX/TX to Bluetooth shield
SoftwareSerial bleSerialPort(2, 3);

//  Param_1: SoftwareSerial port for module comms
//  Param_2: 0 = null pointer
//  Param_3: '1' - Use Packet mode as serial comm is no flow control
BGLib ble112((HardwareSerial *)&bleSerialPort, 0, 1);

#define BGAPI_GET_RESPONSE(v, dType) dType *v = (dType *)ble112.getLastRXPayload()

// ********************************************************
//   Function: void setup()
// ********************************************************
void setup() {
    connectedFlag = false;
    
    // Assert Reset line active to Bluetooth module
    pinMode(BLE_RESET_PIN, OUTPUT);
    digitalWrite(BLE_RESET_PIN, LOW);
    
    // Initialize status LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    
    Serial.begin(19200);
    while (!Serial);

    // Init Aruduino I/O pins used in this demo
    pinMode(BLE_PORT1_BIT_0, INPUT_PULLUP);
    pinMode(BLE_PORT1_BIT_1, INPUT_PULLUP);
    pinMode(BLE_PORT1_BIT_2, INPUT_PULLUP);
    pinMode(BLE_PORT1_BIT_3, INPUT_PULLUP);
    
    pinMode(BLE_PORT0_BIT_0, OUTPUT);
    pinMode(BLE_PORT0_BIT_1, OUTPUT);
    pinMode(BLE_PORT0_BIT_2, OUTPUT);
    pinMode(BLE_PORT0_BIT_3, OUTPUT);
    
    // open BLE software serial port
    bleSerialPort.begin(19200);
    
    // Allow time for software serial init.
    delay(10);
    
    // Release Bluetooth reset
    pinMode(BLE_RESET_PIN, OUTPUT);
    digitalWrite(BLE_RESET_PIN, HIGH);

    // BGLib event handlers
    ble112.ble_evt_system_boot = my_ble_evt_system_boot;
    ble112.ble_evt_connection_status = my_ble_evt_connection_status;
    ble112.ble_evt_connection_disconnected = my_ble_evt_connection_disconnect;
    ble112.ble_evt_attributes_value = my_ble_evt_attributes_value;
}

// ********************************************************
//   Function: void loop()
// ********************************************************
//
// main application loop
void loop() {

  while (1) 
  {
        // keep polling for new data from BLE
        ble112.checkActivity();

        // blink Arduino LED based on state:
        //  1 slow flash = ADVERTISING
        //  2 flashes  = CONNECTED_SLAVE
        //  3 flashes = CONNECTED_SLAVE + encryption    
        uint16_t slice = millis() % 1000;
     
        if (ble_state == BLE_STATE_STANDBY) {
            digitalWrite(LED_PIN, HIGH);
        } else if (ble_state == BLE_STATE_ADVERTISING) {
            digitalWrite(LED_PIN, slice < 100);
        } else if (ble_state == BLE_STATE_CONNECTED_SLAVE) {
            if (!ble_encrypted) {
                digitalWrite(LED_PIN, slice < 100 || (slice > 200 && slice < 300));
            } else {
                digitalWrite(LED_PIN, slice < 100 || (slice > 200 && slice < 300) || (slice > 400 && slice < 500));
            }
        }

        // Poll 10hz for input pin state change
        if (slice % 100 == 0)
        {
	  // When rollover occurs update port notification state
          transmit_input_port_pin_states();
          
          if (startUpTimer > 0)
          {
             startUpTimer --; 
          }
        }
    }
}

// **************************************************************************
//   Function:  my_ble_evt_system_boot(const ble_msg_system_boot_evt_t *msg)
// **************************************************************************
// Callback activated during Bluegiga module boot
//
void my_ble_evt_system_boot(const ble_msg_system_boot_evt_t *msg) {
    #ifdef DEBUG
        Serial.print(F("###\tsystem_boot: { "));
        Serial.print(F("major: ")); Serial.print(msg -> major, HEX);
        Serial.print(F(", minor: ")); Serial.print(msg -> minor, HEX);
        Serial.print(F(", patch: ")); Serial.print(msg -> patch, HEX);
        Serial.print(F(", build: ")); Serial.print(msg -> build, HEX);
        Serial.print(F(", ll_version: ")); Serial.print(msg -> ll_version, HEX);
        Serial.print(F(", protocol_version: ")); Serial.print(msg -> protocol_version, HEX);
        Serial.print(F(", hw: ")); Serial.print(msg -> hw, HEX);
        Serial.println(F(" }"));
    #endif

     // Init Bluetooth module Port 0
     ble112.ble_cmd_hardware_io_port_config_direction( 0, BLE112_IO_PORT_0_CONFIG);
     while (ble112.checkActivity(1000));
     
     // Init Bluetooth module Port 1
     ble112.ble_cmd_hardware_io_port_config_direction( 1, BLE112_IO_PORT_1_CONFIG);
     while (ble112.checkActivity(1000));
    
     // Init Bluetooth GATT Characteristic port 1 and 2 configuration bits
     // This enables the Remote system to self configure it's user interface.
     //
     uint8_t portCfg = 0x0f; // Lower 4 bits of port1 GATT are used as outputs this demo
     port1_config_bits = portCfg;
          
     ble112.ble_cmd_attributes_write( GATT_HANDLE_C_PORT1_CFG, 0, 1,  (uint8*) &portCfg);
     while (ble112.checkActivity(1000));
     
     portCfg = 0x00; // All bits of port 2 GATT are used as inputs
     ble112.ble_cmd_attributes_write( GATT_HANDLE_C_PORT2_CFG, 0, 1,  (uint8*) &portCfg);
     while (ble112.checkActivity(1000));

    //  Flash LED to show Bluetooth boot event detected, marking start of advertising
    flashLed(3);
    
    //  All ready to start advertsing
    startAdvertising();
}

// ***************************************************************************************
//   Function:  my_ble_evt_connection_status(const ble_msg_connection_status_evt_t *msg)
// ***************************************************************************************
// Called when connection status changes
void my_ble_evt_connection_status(const ble_msg_connection_status_evt_t *msg) {
    #ifdef DEBUG
        Serial.print(F("###\tconnection_status: "));
        Serial.print(F("connection: ")); Serial.print(msg -> connection, HEX);
        Serial.print(F(", flags: ")); Serial.print(msg -> flags, HEX);
        Serial.print(F(", address: "));
        // this is a "bd_addr" data type, which is a 6-byte uint8_t array
        for (uint8_t i = 0; i < 6; i++) {
            if (msg -> address.addr[i] < 16) Serial.write('0');
            Serial.print(msg -> address.addr[i], HEX);
        }
        Serial.print(F(", address_type: ")); Serial.print(msg -> address_type, HEX);
        Serial.print(F(", conn_interval: ")); Serial.print(msg -> conn_interval, HEX);
        Serial.print(F(", timeout: ")); Serial.print(msg -> timeout, HEX);
        Serial.print(F(", latency: ")); Serial.print(msg -> latency, HEX);
        Serial.print(F(", bonding: ")); Serial.print(msg -> bonding, HEX);
        Serial.println(F(" }"));
    #endif

    // "flags" bit description:
    //  - bit 0: connection_connected
    //           Indicates the connection exists to a remote device.
    //  - bit 1: connection_encrypted
    //           Indicates the connection is encrypted.
    //  - bit 2: connection_completed
    //           Indicates that a new connection has been created.
    //  - bit 3; connection_parameters_change
    //           Indicates that connection parameters have changed, and is set
    //           when parameters change due to a link layer operation.

    // check for new connection established
    if ((msg -> flags & 0x05) == 0x05) {
        // track state change based on last known state, since we can connect two ways
        if (ble_state == BLE_STATE_ADVERTISING) {
            ble_state = BLE_STATE_CONNECTED_SLAVE;
            
            // Trigger Transmission of input port bits when connection is established
            startUpTimer = 10;
            
        } else {
          // no other state supported
        }
    }

    // update "encrypted" status
    ble_encrypted = msg -> flags & 0x02;
    
    // update "bonded" status
    ble_bonding = msg -> bonding;
    
    connectedFlag = true;
}


// ******************************************************************************************************
//   Function: my_ble_evt_connection_disconnect(const struct ble_msg_connection_disconnected_evt_t *msg)
// ******************************************************************************************************
// Callback function for disconnect
//
void my_ble_evt_connection_disconnect(const struct ble_msg_connection_disconnected_evt_t *msg) {
    #ifdef DEBUG
        Serial.print(F("###Connection_disconnect: { "));
        Serial.print(F("connection: ")); Serial.print(msg -> connection, HEX);
        Serial.print(F(", reason: ")); Serial.print(msg -> reason, HEX);
        Serial.println(F(" }"));
    #endif

    connectedFlag = false;
    startUpTimer = 0;
    
    while (ble112.checkActivity(1000));

    // Reconfigure module to discoverable/connectable mode
    ble112.ble_cmd_gap_set_mode(BGLIB_GAP_GENERAL_DISCOVERABLE, BGLIB_GAP_UNDIRECTED_CONNECTABLE);
    while (ble112.checkActivity(1000)); // response should come back within milliseconds

    // set state to ADVERTISING
    ble_state = BLE_STATE_ADVERTISING;

    // clear "encrypted" and "bonding" info
    ble_encrypted = 0;
    ble_bonding = 0xFF;
}

// ******************************************************************************************
//   Function: my_ble_evt_attributes_value(const struct ble_msg_attributes_value_evt_t *msg)
// ******************************************************************************************
// Callback function is triggered by a write to attributes database
//
void my_ble_evt_attributes_value(const struct ble_msg_attributes_value_evt_t *msg) {
    #ifdef DEBUG
        Serial.print(F("###\tattributes_value: { "));
        Serial.print(F("connection: ")); Serial.print(msg -> connection, HEX);
        Serial.print(F(", reason: ")); Serial.print(msg -> reason, HEX);
        Serial.print(F(", handle: ")); Serial.print(msg -> handle, HEX);
        Serial.print(F(", offset: ")); Serial.print(msg -> offset, HEX);
        Serial.print(F(", value_len: ")); Serial.print(msg -> value.len, HEX);
        Serial.print(F(", value_data: "));
        // this is a "uint8array" data type, which is a length byte and a uint8_t* pointer
        for (uint8_t i = 0; i < msg -> value.len; i++) {
            if (msg -> value.data[i] < 16) Serial.write('0');
            Serial.print(msg -> value.data[i], HEX);
        }
        Serial.println(F(" }"));
    #endif
      
      // 
      // *************************************
      //   Port 1 breakout - Digital I/O pins
      // *************************************
      //   Bit 0 --> DPin_5
      //   Bit 1 --> DPin_6
      //   Bit 2 --> DPin_7
      //   Bit 3 --> DPin_8
      //   Bit 4 --> DPin_9
      //   Bit 5 --> DPin_12
      //   Bit 6 --> DPin_13 (Shared with Arduino LED)
      //   Bit 7 --> Unmapped
      // *************************************
      
      // ************************************
      //   Port 2 breakout - Analog I/O pins
      // ************************************
      //   Bit 0 --> Pin_A0
      //   Bit 1 --> Pin_A1
      //   Bit 2 --> Pin_A2
      //   Bit 3 --> Pin_A3
      //   Bit 4 --> Pin_A4
      //   Bit 5 --> Pin_A5
      // *************************************
      
    // Decode GATT Characteristic is being written
    if (msg -> value.len > 0)
    {
      if (msg -> handle == GATT_HANDLE_C_PORT1_DATA ) { 
        writeBluetooth_IO_Port_Data(msg -> value.data[0], 0);
        Serial.print(F("Port 0 write: 0x"));
        Serial.println(msg -> value.data[0], HEX );
        
        while (ble112.checkActivity(1000));
      }
      
      if (msg -> handle == GATT_HANDLE_C_PORT2_DATA) {
        writeBluetooth_IO_Port_Data(msg -> value.data[0], 1);
        Serial.print(F("Port 0 write: 0x"));
        Serial.println(msg -> value.data[0], HEX );
        
        while (ble112.checkActivity(1000));
      }
    }
}

// **************************************************************************
//   Function: writeBluetooth_IO_Port_Data(uint8_t outputByte, uint8_t port)
// **************************************************************************
// Maps byte bits to drive Arduino port pins
//
// Port 0 maps to Arduino Digital Pins
// Port 1 maps to Arduino Analog Pins
//
//
void writeBluetooth_IO_Port_Data(uint8_t outputByte, uint8_t port)
{
  if (port == 0)
  {
    uint8_t bitConfig = port1_config_bits;
    
    for(int i = 0; i < PORT0_BIT_CNT; i++)
    { 
        // Write bits which are configured as output bits
        if (bitConfig & 0x01 == 1)
        {
          writeOutputBit(P0_Bits[i], outputByte & 1);
        }
        
        outputByte = outputByte >> 1;
        bitConfig = bitConfig >> 1;
    }
  }
  else if (port == 1)
  {
    uint8_t bitConfig = port2_config_bits;
            
    for(int i = 0; i < PORT1_BIT_CNT; i++)
    {
        // Write bits which are configured as output bits
        if (bitConfig & 0x01 == 1)
        {
          writeOutputBit(P1_Bits[i], outputByte & 1);
        }
        
        outputByte = outputByte >> 1;
        bitConfig = bitConfig >> 1;
    }
  }
}

// ****************************************************************
//   Function: writeOutputBit(uint8_t bitNumber, uint8_t bitValue)
// ****************************************************************
//
void writeOutputBit(uint8_t bitNumber, uint8_t bitValue)
{
  digitalWrite(bitNumber, bitValue); 
}

// *****************************************
//   Function: hwResetBluetoothModule(void)
// *****************************************
// Active low reset pulse. 
//
void hwResetBluetoothModule(void)
{
      digitalWrite(BLE_RESET_PIN, LOW);
      delay(100);
      digitalWrite(BLE_RESET_PIN, HIGH);
}

// **********************************************
//   Function: flashLed(uint8_t numberOfFlashes)
// **********************************************
// Assumes LED is tied the Bluegiga Port 0, bit 0
// Active high drive, LED tied to port pin and gnd
// through a resistor.
//
void flashLed(uint8_t numberOfFlashes)
{
     for (int i = 0; i < numberOfFlashes; i++)
     {
       ble112.ble_cmd_hardware_io_port_write(0, 01, 01);
       while (ble112.checkActivity(100));
       
       delay (50);
       
       ble112.ble_cmd_hardware_io_port_write(0, 01, 00);
       while (ble112.checkActivity(100));
       
       delay (200);
     }
}

// ***********************************
//   Function: startAdvertising(void)
// ***********************************
// Initialization to allow the device to be discovered and connectable
//
void startAdvertising(void)
{
    ble112.ble_cmd_gap_set_mode(BGLIB_GAP_GENERAL_DISCOVERABLE, BGLIB_GAP_UNDIRECTED_CONNECTABLE);
    while (ble112.checkActivity(1000)); // response should come back within milliseconds
    
    // set state to ADVERTISING
    ble_state = BLE_STATE_ADVERTISING;  
}

// *************************************************
//   Function: transmit_input_port_pin_states(void)
// *************************************************
void transmit_input_port_pin_states(void)
{
    // Transmits inputs bits from Arduino Analog pins 0 - 3
    uint8 inputBits = 0x00;
    
    inputBits = (digitalRead(A0) | (digitalRead(A1) << 1) | (digitalRead(A2) << 2) | (digitalRead(A3) << 3));
    
    // Write bit to bluetooth port 2 value   
    if (connectedFlag == true)
     {
       // Output only required if (data has changed, OR first time read) to elimiate redunant communication
       if ((inputBits != port1_old_bits) || (startUpTimer > 0))
       {
         Serial.print(F("Sending input pin data..."));
         Serial.print(inputBits, HEX);
         Serial.println(F(" "));
         ble112.ble_cmd_attributes_write( GATT_HANDLE_C_PORT2_DATA, 0, 1,  (uint8*) &inputBits);
         while (ble112.checkActivity(100));
       }
     }

     port1_old_bits = inputBits;
}

