#include <Arduino.h>

void setup() {
  Serial1.begin(9600); // Begin the serial communication
}

void loop() {
  if (Serial1.available()) { // Check if data is available to read
    String received = Serial1.readStringUntil('\n'); // Read the incoming data
    Serial1.println(received); // Echo the received data back to the Raspberry Pi
  }

  Serial1.println("Hello Raspberry Pi"); // Send a message to the Raspberry Pi
  delay(1000); // Wait for a second
}