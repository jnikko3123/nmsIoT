#include <Wire.h>                // enable I2C.
#define address 104              // default I2C ID number for EZO Flow

char flow_data[20];              // we make a 20-byte character array to hold incoming data from the Flow circuit.
byte i = 0;                      // counter used for flow_data array.
unsigned long previousMillis = 0; // variable to store the last time data was read
int time_ = 1000;                // read data every 1000 milliseconds (1 second)

void setup()                    // hardware initialization.
{
  Serial.begin(9600);           // enable serial port.
  Wire.begin();                 // enable I2C port.
}

void loop() {                   // the main loop.
  unsigned long currentMillis = millis();  // get the current time

  if (currentMillis - previousMillis >= time_) {
    previousMillis = currentMillis;  // save the last time data was read

    Wire.beginTransmission(address);    // call the circuit by its ID number.
    Wire.write("R");                    // send the command to request data (assuming "R" is the correct command).
    Wire.endTransmission();              // end the I2C data transmission.

    delay(50);  // wait for the sensor to respond

    Wire.requestFrom(address, 20, 1);   // call the circuit and request 20 bytes (this may be more than we need)
    byte code = Wire.read();            // the first byte is the response code

    if (code == 1) {                    // check if the response code indicates success
      i = 0;                            // reset the counter for the array
      while (Wire.available() && i < 19) {
        char in_char = Wire.read();      // receive a byte
        flow_data[i] = in_char;         // load this byte into our array
        i += 1;                         // increment the counter
      }
      flow_data[i] = '\0';              // null-terminate the data
      Serial.println(flow_data);        // print the data to the serial monitor
    } else {
      Serial.println("Failed to read data.");
    }
  }
}
