#include <Adafruit_TCS34725.h>
#include <helper_3dmath.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <I2Cdev.h>


#include <DataFrame.h>
#include <QuickStats.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include <SPI.h>
#include <Adafruit_AMG88xx.h>
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

QuickStats stats;

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

bool blinkState = false;

int16_t ax, ay, az;
int16_t gx, gy, gz;

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 4
#define BME_CS 8

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO,  BME_SCK);



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

int pirPin = 7; // Input for HC-S501

int val = 0;   // Place to store read PIR Value
int pirState = LOW;
int mic = A0;

Adafruit_AMG88xx amg;
const int tempReadings = 10;
const int pressureReadings = 10;
const int humidityReadings = 10;
const int gasReadings = 10;
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
float temperature[tempReadings];
float pressure[pressureReadings];
float humidity[humidityReadings];
float gas[gasReadings];
int gasIndex = 0;
int humidityIndex = 0;
int pressureIndex = 0;
int tempIndex = 0;

void setup() {

  // ===                   MICROPHONE .                      ===

  pinMode(mic, INPUT);

  // ===                TEMPERATURE + HUMIDITY               ===
  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
  // ===                   MOTION SENSOR                     ===

  pinMode(pirPin, INPUT);
  // ===                      Color Illumination                     ===
  if (tcs.begin()) {
    Serial.println("Found sensor");
  } else {
    Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }


  // ===                   ACCELEROMETER                     ===

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(9600);
  while (!Serial); 
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

  /*frame.createFrame(ASCII);
    frame.addSensor("Microphone Average", stats.average(readings, readIndex) );
    frame.addSensor("Microphone Median", stats.median(readings, readIndex) );
    frame.showFrame();
  */
  // ===                      Color Illumination                     ===
  uint16_t r, g, b, c, colorTemp, lux;

  tcs.getRawData(&r, &g, &b, &c);
  colorTemp = tcs.calculateColorTemperature(r, g, b);
  lux = tcs.calculateLux(r, g, b);

  Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
  Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
  Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
  Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
  Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
  Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
  Serial.println(" ");
  //    ===                   Thermal Camera .                  ===
  amg.readPixels(pixels);


  Serial.println("THERMAL CAMERA");
  Serial.print("Average: ");
  Serial.println(stats.average(pixels, AMG88xx_PIXEL_ARRAY_SIZE));
  Serial.print("Maximum: ");
  Serial.println(stats.maximum(pixels, AMG88xx_PIXEL_ARRAY_SIZE));
  Serial.print("Minimum: ");
  Serial.println(stats.minimum(pixels, AMG88xx_PIXEL_ARRAY_SIZE, .0000001));
  Serial.print("Standard Deviation: ");
  Serial.println(stats.stdev(pixels, AMG88xx_PIXEL_ARRAY_SIZE));

  Serial.print("Sum: ");
  Serial.println(stats.sum(pixels, AMG88xx_PIXEL_ARRAY_SIZE));

  // ===                   MICROPHONE                        ===

  // read from the sensor:
  mic = analogRead(mic);
  Serial.println("Microphone");
  Serial.print(mic);


  /*Serial.println("MICROPHONE");
    Serial.print("Average: ");
    Serial.println(stats.average(readings, readIndex));
    Serial.print("Median: ");
    Serial.println(stats.median(readings, readIndex));
    Serial.print("Mode: ");
    Serial.println(stats.mode(readings, readIndex, .0000001));
    Serial.print("Standard Deviation: ");
    Serial.println(stats.stdev(readings, readIndex));
    if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
    }
  */

  // ===                TEMPERATURE + HUMIDITY               ===

  temperature[tempIndex] = bme.temperature;
  tempIndex = tempIndex + 1;
  Serial.println("TEMPERATURE");
  Serial.print("Average: ");
  Serial.println(stats.average(temperature, tempIndex));
  Serial.print("Maximum: ");
  Serial.println(stats.maximum(temperature, tempIndex));
  Serial.print("Minimum: ");
  Serial.println(stats.minimum(temperature, tempIndex, .0000001));
  Serial.print("Standard Deviation: ");
  Serial.println(stats.stdev(temperature, tempIndex));
  Serial.println("Sum: ");
  Serial.println(stats.sum(temperature, tempIndex));
  pressure[pressureIndex] = bme.pressure / 100.0;
  pressureIndex = pressureIndex + 1;
  Serial.println("PRESSURE");
  Serial.print("Average: ");
  Serial.println(stats.average(pressure, pressureIndex));
  Serial.print("Maximum: ");
  Serial.println(stats.maximum(pressure, pressureIndex));
  Serial.print("Minimum: ");
  Serial.println(stats.minimum(pressure, pressureIndex, .0000001));
  Serial.print("Standard Deviation: ");
  Serial.println(stats.stdev(pressure, pressureIndex));
  Serial.println("Sum: ");
  Serial.println(stats.sum(pressure, pressureIndex));
  humidity[humidityIndex] = bme.humidity;
  humidityIndex = humidityIndex + 1;
  Serial.println("HUMIDITY");
  Serial.print("Average: ");
  Serial.println(stats.average(humidity, humidityIndex));
  Serial.print("Maximum: ");
  Serial.println(stats.maximum(humidity, humidityIndex));
  Serial.print("Mode: ");
  Serial.println(stats.mode(humidity, humidityIndex, .0001));
  Serial.print("Standard Deviation: ");
  Serial.println(stats.stdev(humidity, humidityIndex));
  Serial.println("Sum: ");
  Serial.println(stats.sum(humidity, humidityIndex));
  gas[gasIndex] = bme.gas_resistance / 1000.0;
  gasIndex = gasIndex + 1;
  Serial.println("GAS");
  Serial.print("Average: ");
  Serial.println(stats.average(gas, gasIndex));
  Serial.print("Maximum: ");
  Serial.println(stats.maximum(gas, gasIndex));
  Serial.print("Minimum: ");
  Serial.println(stats.minimum(gas, gasIndex, .0001));
  Serial.print("Standard Deviation: ");
  Serial.println(stats.stdev(gas, gasIndex));
  Serial.println("Sum: ");
  Serial.println(stats.sum(gas, gasIndex));
  Serial.println("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");
  delay(1000);
  if (tempIndex >= tempReadings) {
    // ...wrap around to the beginning:
    tempIndex = 0;
  }
  if (pressureIndex >= pressureReadings) {
    // ...wrap around to the beginning:
    pressureIndex = 0;
  }
  if (humidityIndex >= humidityReadings) {
    // ...wrap around to the beginning:
    humidityIndex = 0;
  }
  delay(1000);
  // ===                   MOTION SENSOR                     ===


  val = digitalRead(pirPin);  // read input value
  Serial.println("Motion Sensor val :");
  Serial.println(val);
  if (val == HIGH) {            // check if the input is HIGH
    if (pirState == LOW) {
      // we have just turned on
      Serial.println("Motion detected!");
      // We only want to print on the output change, not state
      pirState = HIGH;
    }
  } else {
    if (pirState == HIGH) {
      // we have just turned of
      Serial.println("Motion ended!");
      // We only want to print on the output change, not state
      pirState = LOW;
    }
  }


  // ===                   ACCELEROMETER                     ===

   

mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);

    #ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values
        Serial.print("a/g:\t");
        Serial.print(ax); Serial.print("\t");
        Serial.print(ay); Serial.print("\t");
        Serial.print(az); Serial.print("\t");
        Serial.print(gx); Serial.print("\t");
        Serial.print(gy); Serial.print("\t");
        Serial.println(gz);
    #endif

    #ifdef OUTPUT_BINARY_ACCELGYRO
        Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
        Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
        Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
        Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
        Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
        Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
    #endif
}


