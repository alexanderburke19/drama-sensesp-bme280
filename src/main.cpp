/*
#include <Wire.h>
#include <Arduino.h>

void setup()
{
  Wire.begin();         // Initialize the I2C bus
  Serial.begin(115200); // Start the serial communication at 115200 baud rate
  while (!Serial)
    ; // Wait for the serial port to open (for Leonardo boards)
  Serial.println("\nI2C Scanner");
}

void loop()
{
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++)
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");
      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000); // wait 5 seconds for the next scan
}
*/

#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp_app_builder.h"
#include "sensesp/transforms/linear.h"
#include <Adafruit_BME280.h>
#include <Wire.h>
#include <Arduino.h>

using namespace sensesp;

reactesp::ReactESP app;

// The setup function performs one-time application initialization.

// BME280
Adafruit_BME280 bme280;

// Define the function that will be called every time we want
// an updated temperature value from the sensor. The sensor reads degrees
// Celsius, but all temps in Signal K are in Kelvin, so add 273.15.
float read_temp_callback() { return (bme280.readTemperature() + 273.15); }
float read_pressure_callback() { return (bme280.readPressure()); }
float read_humidity_callback() { return (bme280.readHumidity()); }

// The setup function performs one-time application initialization.
void setup()
{
#ifndef SERIAL_DEBUG_ENABLED
  SetupSerialDebug(115200);
#endif

  // bme280.begin(21); // join i2c bus (address optional for master)

  auto outside_temperature_metadata =
      new SKMetadata("K",                                               // units
                     "Outside Temperature",                             // display name
                     "Outside Temperature gathered from BME280 sensor", // description
                     "Outside Temp",                                    // short name
                     10.                                                // timeout, in seconds
      );
  auto outside_barometric_pressure_metadata =
      new SKMetadata("Pa",                                                      // units
                     "Outside Pressure",                                        // display name
                     "Outside Barometric Pressure gathered from BME280 sensor", // description
                     "Outside Pressure",                                        // short name
                     10.                                                        // timeout, in seconds
      );
  auto outside_humidity_metadata =
      new SKMetadata("",                                                      // units
                     "Outside Humidity",                                      // display name
                     "Outside Relative Humidity gathered from BME280 sensor", // description
                     "Outside Humidity",                                      // short name
                     10.                                                      // timeout, in seconds
      );

  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("drama-sensesp_outside")
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    ->set_wifi("drama_network", "sv_drama")
                    ->set_sk_server("10.10.10.1", 3000)
                    ->enable_uptime_sensor()
                    ->enable_ota("drama")
                    ->get_app();

  /// BME280 SENSOR CODE - Temp/Humidity/Altitude/Pressure Sensor ////

  // 0x77 is the default address. Some chips use 0x76, which is shown here.
  // If you need to use the TwoWire library instead of the Wire library, there
  // is a different constructor: see bmp280.h

  if (!bme280.begin(0x76))
  {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1)
      ;
  }

  // Create a RepeatSensor with float output that reads the temperature
  // using the function defined above.
  auto *bme280_temp =
      new RepeatSensor<float>(5000, read_temp_callback);

  auto *bme280_pressure =
      new RepeatSensor<float>(60000, read_pressure_callback);

  auto *bme280_humidity =
      new RepeatSensor<float>(60000, read_humidity_callback);

  // Send the temperature to the Signal K server as a Float
  bme280_temp->connect_to(new SKOutputFloat("environment.outside.temperature", outside_temperature_metadata));

  bme280_pressure->connect_to(new SKOutputFloat("environment.outside.pressure", outside_barometric_pressure_metadata));

  bme280_humidity->connect_to(new SKOutputFloat("environment.outside.humidity", outside_humidity_metadata));

  // Start networking, SK server connections and other SensESP internals
  sensesp_app->start();
}

void loop()
{
  app.tick();
}
