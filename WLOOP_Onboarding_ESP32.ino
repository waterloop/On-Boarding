// Bluetooth Reference: https://randomnerdtutorials.com/esp32-bluetooth-classic-arduino-ide
// UART Reference: https://www.youtube.com/watch?v=1veVFMcxe3M

#include "BluetoothSerial.h"
#include <HardwareSerial.h>
// #include <ESP32Servo.h> // install ESP32Servo Library

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;
HardwareSerial SerialPort(2); // port 2 for UART communication 
// Servo servo;

// int servoPin = 15; // GPIO to connect servo (digital out)

// Interrupt Handler 
void IRAM_ATTR isr() {
  SerialPort.println("BRAKE");
}

void setup() {
  // Servo Setup 
  // servo.setPeriodHertz(50); 
  // servo.attach(servoPin, 500, 2500); // attach servo on servoPin to servo object

  SerialPort.begin(115200, SERIAL_8N1, 34, 35); // initialize UART (34 = RX, 35 = TX)
  Serial.begin(115200); // initialize serial port for debugging 


  SerialBT.begin("ESP32Test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");

  attachInterrupt()
  
}

void loop() {

  // check if bytes received in serial port
  // if (Serial.available()) {
  //   SerialBT.write(Serial.read()); // send via Bluetooth to connected device 
  // }

  // check if bytes available to read in Bluetooth serial port  
  if (SerialBT.available()) {
    //Serial.write(SerialBT.read()); // write bytes to serial monitor 

    String bluetoothData = SerialBT.readStringUntil('\n'); // read data from Bluetooth
    Serial.println("Received from Bluetooth: " + bluetoothData);
    
    // Send desired commands to STM32
    if (bluetoothData.equals("BEGIN")) {
      SerialPort.println("BEGIN");
    } else if (bluetoothData.equals("END")) {
      SerialPort.println("END");
    }
  }

  // check if data is present in UART buffer 
  if(SerialPort.available()){
    String bufferUART = SerialPort.readStringUntil('\n'); // read data from UART
    Serial.println("Received from STM32: " + bufferUART);
  }

  delay(20);
}
