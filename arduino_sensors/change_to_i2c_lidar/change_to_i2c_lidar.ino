#include <Wire.h>
#include <SoftwareSerial.h>

// Define the SoftwareSerial pins for communication with TFmini Plus
#define SOFT_RX 2 // Connect to TFmini Plus TX
#define SOFT_TX 3 // Connect to TFmini Plus RX

SoftwareSerial tfminiSerial(SOFT_RX, SOFT_TX); // RX, TX

void setup() {
  Wire.begin();
  Serial.begin(9600);
  tfminiSerial.begin(115200); // TFmini Plus default baud rate
  
  // Give some time for the sensor to power up
  delay(1000);

  Serial.println("Switching TFmini Plus to I2C mode...");

  // Send the command to switch to I2C mode
  switchToI2CMode();
}

void loop() {
  // Nothing to do here
}

void switchToI2CMode() {
  // UART to I2C command
  uint8_t command[] = {0x5A, 0x05, 0x0A, 0x01, 0x6A};

  // Send the command to the TFmini Plus via SoftwareSerial
  for (uint8_t i = 0; i < sizeof(command); i++) {
    tfminiSerial.write(command[i]);
    delay(10); // Small delay between bytes
  }

  Serial.println("Command sent. Reboot the TFmini Plus to apply the changes.");
}

void switchToUARTMode() {
  // I2C to UART command
  uint8_t command[] = {0x5A, 0x05, 0x0A, 0x00, 0x69};

  // Send the command to the TFmini Plus via SoftwareSerial
  for (uint8_t i = 0; i < sizeof(command); i++) {
    tfminiSerial.write(command[i]);
    delay(10); // Small delay between bytes
  }

  Serial.println("Command sent. Reboot the TFmini Plus to apply the changes.");
}
