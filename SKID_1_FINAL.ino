#include "rak1906.h"
rak1906 rak1906;
#include "rak1910.h"
rak1910 rak1910;
#include <Wire.h>
#define MAX_SIZE 100
#define OTAA_PERIOD   (20000)
#define OTAA_BAND     (RAK_REGION_EU868)
#define OTAA_DEVEUI   {0xAC, 0x1F, 0x09, 0xFF, 0xFE, 0x08, 0x6C, 0x3D}
#define OTAA_APPEUI   {0xAC, 0x1F, 0x09, 0xFF, 0xFE, 0x08, 0x6C, 0x3D}
#define OTAA_APPKEY   {0x8A, 0xE9, 0xA1, 0xA9, 0x30, 0xB0, 0x77, 0x57, 0x82, 0x93, 0x83, 0x3F, 0xDF, 0xCE, 0x97, 0xA6}
#define LED1_PIN 35
#define LED2_PIN 36
#define RAK5801_PIN WB_A1
#define NO_OF_SAMPLES 1000
#define PIN_VBAT A0
uint32_t vbat_pin = PIN_VBAT;
#define VBAT_MV_PER_LSB (0.73242188F)
#define VBAT_DIVIDER (0.4F)
#define VBAT_DIVIDER_COMP (1.73)
#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)
uint8_t pulseCounterAddress = 104;
char flow_data[20];
byte i = 0;
unsigned long previousMillis = 0;
int time_ = 1000;
uint8_t collected_data[64] = { 0 };
String flowRateString = "";

void uplink_routine();
void scanI2CDevices();
void initializeEZOFlowSensor();
//float readFlowRateFromEZO();
float readVBAT(void)
{
  float raw;
  raw = analogRead(vbat_pin);
  return raw * REAL_VBAT_MV_PER_LSB;
}

uint8_t mvToPercent(float mvolts)
{
  if (mvolts < 3300)
    return 0;

  if (mvolts < 3600)
  {
    mvolts -= 3300;
    return mvolts / 30;
  }

  mvolts -= 3600;
  return 10 + (mvolts * 0.15F);
}

uint8_t mvToLoRaWanBattVal(float mvolts)
{
  if (mvolts < 3300)
    return 0;

  if (mvolts < 3600)
  {
    mvolts -= 3300;
    return mvolts / 30 * 2.55;
  }

  mvolts -= 3600;
  return (10 + (mvolts * 0.15F)) * 2.55;
}

void recvCallback(SERVICE_LORA_RECEIVE_T * data)
{
  if (data->BufferSize > 0) {
    Serial.println("Something received!");
    for (int i = 0; i < data->BufferSize; i++) {
      Serial.printf("%x", data->Buffer[i]);
    }
    Serial.print("\r\n");
  }
}

void joinCallback(int32_t status)
{
  Serial.printf("Join status: %d\r\n", status);
}

void sendCallback(int32_t status)
{
  if (status == RAK_LORAMAC_STATUS_OK) {
    Serial.println("Successfully sent");
  } else {
    Serial.println("Sending failed");
  }
}

void setup()
{
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  digitalWrite(LED1_PIN, HIGH);
  Wire.begin();
  Wire.setClock(400000);
  initializeEZOFlowSensor();
  scanI2CDevices();
  Serial.begin(115200, RAK_AT_MODE);
  while (!Serial);
  Serial.printf("RAK1906 init %s\r\n", rak1906.init() ? "Success" : "Fail");
  rak1910.init();
  Serial.printf("RAK1910 init %s\r\n", rak1910.init() ? "Success" : "Fail");
  Serial.println("Pepnoun SKid #1");
  Serial.println("------------------------------------------------------");
  if (api.lorawan.nwm.get() != 1)
  {
    Serial.printf("Set Node device work mode %s\r\n",
                  api.lorawan.nwm.set(1) ? "Success" : "Fail");
    api.system.reboot();
  }

  // OTAA Device EUI MSB first
  uint8_t node_device_eui[8] = OTAA_DEVEUI;

  uint8_t node_app_eui[8] = OTAA_APPEUI;

  uint8_t node_app_key[16] = OTAA_APPKEY;

  if (!api.lorawan.appeui.set(node_app_eui, 8)) {
    Serial.printf("LoRaWan OTAA - set application EUI is incorrect! \r\n");
    return;
  }
  if (!api.lorawan.appkey.set(node_app_key, 16)) {
    Serial.printf("LoRaWan OTAA - set application key is incorrect! \r\n");
    return;
  }
  if (!api.lorawan.deui.set(node_device_eui, 8)) {
    Serial.printf("LoRaWan OTAA - set device EUI is incorrect! \r\n");
    return;
  }

  if (!api.lorawan.band.set(OTAA_BAND)) {
    Serial.printf("LoRaWan OTAA - set band is incorrect! \r\n");
    return;
  }
  if (!api.lorawan.deviceClass.set(RAK_LORA_CLASS_A)) {
    Serial.printf("LoRaWan OTAA - set device class is incorrect! \r\n");
    return;
  }
  if (!api.lorawan.njm.set(RAK_LORA_OTAA))
  {
    Serial.printf("LoRaWan OTAA - set network join mode is incorrect! \r\n");
    return;
  }
  if (!api.lorawan.join())  // Join to Gateway
  {
    Serial.printf("LoRaWan OTAA - join fail! \r\n");
    return;
  }

  /** Wait for Join success */
  while (api.lorawan.njs.get() == 0) {
    Serial.print("Wait for LoRaWAN join...");
    api.lorawan.join();
    delay(10000);
  }

  if (!api.lorawan.adr.set(true)) {
    Serial.printf("LoRaWan OTAA - set adaptive data rate is incorrect! \r\n");
    return;
  }
  if (!api.lorawan.rety.set(1)) {
    Serial.printf("LoRaWan OTAA - set retry times is incorrect! \r\n");
    return;
  }
  if (!api.lorawan.cfm.set(1)) {
    Serial.printf("LoRaWan OTAA - set confirm mode is incorrect! \r\n");
    return;
  }

  /** Check LoRaWan Status*/
  Serial.printf("Duty cycle is %s\r\n", api.lorawan.dcs.get() ? "ON" : "OFF");  // Check Duty Cycle status
  Serial.printf("Packet is %s\r\n", api.lorawan.cfm.get() ? "CONFIRMED" : "UNCONFIRMED");  // Check Confirm status
  uint8_t assigned_dev_addr[4] = { 0 };
  api.lorawan.daddr.get(assigned_dev_addr, 4);
  Serial.printf("Device Address is %02X%02X%02X%02X\r\n", assigned_dev_addr[0], assigned_dev_addr[1], assigned_dev_addr[2], assigned_dev_addr[3]);  // Check Device Address
  Serial.printf("Uplink period is %ums\r\n", OTAA_PERIOD);
  Serial.println("");
  api.lorawan.registerRecvCallback(recvCallback);
  api.lorawan.registerJoinCallback(joinCallback);
  api.lorawan.registerSendCallback(sendCallback);
  analogReference(AR_INTERNAL_3_0); // Set the reference voltage to 3V
  analogReadResolution(12);
  delay(1);
  readVBAT();
  digitalWrite(LED1_PIN, LOW);
}

void addInt32ToData(uint8_t* data, uint8_t* length, int32_t value) {
  data[(*length)++] = (value >> 24) & 0xFF;
  data[(*length)++] = (value >> 16) & 0xFF;
  data[(*length)++] = (value >> 8) & 0xFF;
  data[(*length)++] = value & 0xFF;
}

void uplink_routine()
{
  digitalWrite(LED1_PIN, HIGH);
  delay(1000);

  Serial.println("Starting uplink routine...");

  float pressure;
  float pressure_min = 0;
  float pressure_max = 10;
  initializeEZOFlowSensor();

  String debugString = "";

  float initialFlowRate = atof(flow_data);

  // Battery Voltage Reading
  float vbat_mv = readVBAT();
  uint8_t vbat_per = mvToPercent(vbat_mv);
  debugString += "Battery: " + String(vbat_mv) + "mV, " + String(vbat_per) + "%; ";

  // RAK5801 Sensor Data Reading
  int sensorValue = analogRead(RAK5801_PIN);
  float voltage = sensorValue * (3.0 / 4096.0);
  float current_mA = voltage / 150.0 * 1000.0;
  if (current_mA >= 4) {
    pressure = map(current_mA, 4, 20, pressure_min * 100, pressure_max * 100) / 100.0;
  }
  debugString += "Sensor: " + String(current_mA) + "mA, " + String(pressure) + "Bar; ";

  // RAK1910 GPS Module Status Check and Data Reading
  uint8_t status = rak1910.status();
  if (status == 0) {
    if (rak1910.update()) {
      debugString += "GPS: Lat " + String(rak1910.latitude(), 6) + ", Lon " + String(rak1910.longitude(), 6) + "; ";
    } else {
      debugString += "GPS Update Fail; ";
    }
  } else if (status == 1) {
    debugString += "GPS: Wait for stronger signal; ";
  } else if (status == 2) {
    debugString += "GPS Init Fail; ";
  }

  uint8_t data_len = 0;

  // Battery Percentage (vbat_per)
  collected_data[data_len++] = 0x04; // Data Channel: 4
  collected_data[data_len++] = 0x06; // Type: Generic Percentage
  collected_data[data_len++] = vbat_per; // Battery Percentage

  // Latitude (lat) and Longitude (long) in degrees
  int32_t lat_fixed = (int32_t)(rak1910.latitude() * 10000); // Adjusted for 3 bytes
  int32_t long_fixed = (int32_t)(rak1910.longitude() * 10000); // Adjusted for 3 bytes
  collected_data[data_len++] = 0x06; // Data Channel: 6
  collected_data[data_len++] = 0x88; // Type: GPS Location (3+3 bytes)
  collected_data[data_len++] = (uint8_t)(lat_fixed >> 16); // Lat high byte
  collected_data[data_len++] = (uint8_t)(lat_fixed >> 8);  // Lat mid byte
  collected_data[data_len++] = (uint8_t)lat_fixed;         // Lat low byte
  collected_data[data_len++] = (uint8_t)(long_fixed >> 16); // Long high byte
  collected_data[data_len++] = (uint8_t)(long_fixed >> 8);  // Long mid byte
  collected_data[data_len++] = (uint8_t)long_fixed;         // Long low byte

  // Temperature (temperature) in 0.1°C resolution
  if (rak1906.update()) {
    int16_t temperature_fixed = static_cast<int16_t>(rak1906.temperature() * 10);
    collected_data[data_len++] = 0x01; // Data Channel: 1
    collected_data[data_len++] = 0x67; // Type: Temperature Sensor
    collected_data[data_len++] = (uint8_t)(temperature_fixed >> 8); // Temp high byte
    collected_data[data_len++] = (uint8_t)temperature_fixed;        // Temp low byte
    debugString += "Temperature: " + String(temperature_fixed) + "°C; ";
  } else {
    Serial.println("Please plug in the sensor RAK1906 and Reboot");
  }

  // Humidity (humidity) in 0.5% resolution
  if (rak1906.update()) {
    float humidity = rak1906.humidity();
    collected_data[data_len++] = 0x02;               // Data Channel: 2
    collected_data[data_len++] = 0x68;               // Type: Humidity Sensor
    collected_data[data_len++] = static_cast<uint8_t>(humidity * 2); // Humidity in 0.5% resolution
    debugString += "Humidity: " + String(humidity) + "%; ";
  } else {
    Serial.println("Please plug in the sensor RAK1906 and Reboot");
  }

  // Pressure in Bar
  int32_t pressure_fixed = static_cast<int32_t>(pressure * 1000 * 10); // Multiply by 10 for Cayenne LPP format
  collected_data[data_len++] = 0x03; // Data Channel: 3
  collected_data[data_len++] = 0x73; // Type: Barometric Pressure
  collected_data[data_len++] = (uint8_t)(pressure_fixed >> 8); // Pressure high byte
  collected_data[data_len++] = (uint8_t)pressure_fixed;        // Pressure low byte

  // Flow Rate in m3
 
  int initialflowRateInt = static_cast<int>(initialFlowRate * 100);  //
  collected_data[data_len++] = 7;       // Data Channel: 7 for Flow Rate
  collected_data[data_len++] = 0x02;    // Type: Analog Input
  collected_data[data_len++] = initialflowRateInt >> 8;  // High byte
  collected_data[data_len++] = initialflowRateInt & 0xFF; // Low byte
  debugString += "Initial Flow Rate: " + String(initialFlowRate) + "m3/min; ";
  
  Serial.println(debugString);

  // LoRaWAN Data Transmission
  if (api.lorawan.send(data_len, (uint8_t *)&collected_data, 2, true, 1)) {
    Serial.println("Sending is requested");
  } else {
    Serial.println("Sending failed");
  }
  Serial.println("Uplink routine completed.");
  delay(1000);
  digitalWrite(LED1_PIN, LOW);
}

void loop()
{
  static uint64_t last = 0;
  static uint64_t elapsed;
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= time_) {
    previousMillis = currentMillis;
    // float flowRate = readFlowRateFromEZO();
    // Serial.print("Flow Rate: ");
    // Serial.print(flowRate);
    // Serial.println(" mL/min");
  }
  if ((elapsed = millis() - last) > OTAA_PERIOD) {
    uplink_routine();

    last = millis();
  }
  Serial.printf("Try sleep %ums..", OTAA_PERIOD);
  api.system.sleep.all(OTAA_PERIOD);
  Serial.println("Wakeup..");
  delay(1000);
}

void scanI2CDevices() {
  byte error, address;
  int deviceCount = 0;

  Serial.println("I2C Scanner");

  Serial.println("Scanning...");

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(" !");
      deviceCount++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  if (deviceCount == 0) {
    Serial.println("No I2C devices found.");
  } else {
    Serial.println("Scan complete.");
  }
}

void initializeEZOFlowSensor() {
  unsigned long currentMillis = millis();  // get the current time

  if (currentMillis - previousMillis >= time_) {
    previousMillis = currentMillis;  // save the last time data was read

    Wire.beginTransmission(pulseCounterAddress);    // call the circuit by its ID number.
    Wire.write("R");                    // send the command to request data (assuming "R" is the correct command).
    Wire.endTransmission();              // end the I2C data transmission.

    delay(50);  // wait for the sensor to respond

    Wire.requestFrom(pulseCounterAddress, 20, 1);   // call the circuit and request 20 bytes (this may be more than we need)
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

// float readFlowRateFromEZO() {
//   Wire.beginTransmission(pulseCounterAddress);
//   Wire.write("R");
//   Wire.endTransmission();
//   delay(50);

//   Wire.requestFrom(pulseCounterAddress, 20, 1);
//   byte code = Wire.read();

//   if (code == 1) {
//     i = 0;
//     while (Wire.available() && i < 19) {
//       char in_char = Wire.read();
//       flow_data[i] = in_char;
//       i += 1;
//     }
//     flow_data[i] = '\0';
//     Serial.println(flow_data);
//   } else {
//     Serial.println("Failed to read data.");
//   }
// }