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
const int tempReadings = 100;
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
  } else {
    while (1);
  }
    // ===                      Thermal Sensor                     ===
bool status;
status = amg.begin();
    if (!status) {
        Serial.println("Could not find a valid AMG88xx sensor, check wiring!");
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

  mpu.initialize();




  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

}




// ================================================================
// ===                    Functions .                           ===
// ================================================================
// ===                   Temperature                        ===

void temperaturefunc() {
  for (int tempIndex = 0; tempIndex < 10; tempIndex++)
  {
    temperature[tempIndex] = bme.temperature;
  }
  if (tempIndex >= tempReadings)
  {
    tempIndex = 0;
  }
  if (tempIndex = 10)
  {
    Serial.println("ta");
    Serial.println(stats.average(temperature, tempIndex));
    Serial.println("tmax");
    Serial.println(stats.maximum(temperature, tempIndex));
    Serial.println("tmin");
    Serial.println(stats.minimum(temperature, tempIndex));
    Serial.println("tst");
    Serial.println(stats.stdev(temperature, tempIndex));
    Serial.println("ts");
    Serial.println(stats.sum(temperature, tempIndex));
    Serial.println("tr");
    Serial.println(stats.range(temperature, tempIndex));
  }
}
// ===                   Pressure                        ===

void pressurefunc() {
  for (int pressureIndex = 0; pressureIndex < 10; pressureIndex++)
  {
    pressure[pressureIndex] = bme.pressure / 100.0;
  }
  if (pressureIndex >= tempReadings)
  {
    pressureIndex = 0;
  }
  if (pressureIndex = 10)
  {
    Serial.println("pa");
    Serial.println(stats.average(pressure, pressureIndex));
    Serial.println("pmax");
    Serial.println(stats.maximum(pressure, pressureIndex));
    Serial.println("pmin");
    Serial.println(stats.minimum(pressure, pressureIndex));
    Serial.println("pst");
    Serial.println(stats.stdev(pressure, pressureIndex));
    Serial.println("ps");
    Serial.println(stats.sum(pressure, pressureIndex));
    Serial.println("pr");
    Serial.println(stats.range(pressure, pressureIndex));

  }
}
// ===                   Gas                        ===

void gasfunc() {

  for (int gasIndex = 0; gasIndex < 10; gasIndex++)
  {
    gas[gasIndex] = bme.gas_resistance / 1000.0;
  }
  if (gasIndex >= gasReadings)
  {
    gasIndex = 0;
  }
  if (gasIndex = 10)
  {
    Serial.println("ga");
    Serial.println(stats.average(gas, gasIndex));
    Serial.println("gmax");
    Serial.println(stats.maximum(gas, gasIndex));
    Serial.println("gmin: ");
    Serial.println(stats.minimum(gas, gasIndex));
    Serial.println("gst");
    Serial.println(stats.stdev(gas, gasIndex));
    Serial.println("gs");
    Serial.println(stats.sum(gas, gasIndex));
    Serial.println("gr");
    Serial.println(stats.range(gas, gasIndex));
  }
}
// ===                   Humidity                        ===

void humidityfunc() {
  for (int humidityIndex = 0; humidityIndex < 10; humidityIndex++)
  {
    humidity[humidityIndex] = bme.humidity;
  }
  if (humidityIndex >= humidityReadings)
  {
    humidityIndex = 0;
  }
  if (humidityIndex = 10)
  {
    Serial.println("hp");
    Serial.println(stats.average(humidity, humidityIndex));
    Serial.println("hmax");
    Serial.println(stats.maximum(humidity, humidityIndex));
    Serial.println("hmin");
    Serial.println(stats.minimum(humidity, humidityIndex));
    Serial.println("hst");
    Serial.println(stats.stdev(humidity, humidityIndex));
    Serial.println("hs");
    Serial.println(stats.sum(humidity, humidityIndex));
    Serial.println("hr");
    Serial.println(stats.range(humidity, humidityIndex));
  }
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
  Serial.println("l");
  Serial.println(lux);
  Serial.println("r");
  Serial.println(r);
  Serial.println("g");
  Serial.println(g);
  Serial.println("b");
  Serial.println(b);
  Serial.println("ct");
  Serial.println(colorTemp);






  /* Serial.print("Color Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
    Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
    Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
    Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
    Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
    Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
    Serial.println(" ");*/
   //    ===                   Thermal Camera .                  ===
 amg.readPixels(pixels);

    Serial.println("[");
    for(int i=1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++){
      Serial.print(pixels[i-1]);
      Serial.print(", ");
      if( i%8 == 0 ) Serial.println();
    }
    Serial.println("]");
    



  /*  // ===                   MICROPHONE                        ===

    // read from the sensor:
    mic = analogRead(mic);

    //Serial.println("Microphone");
    Serial.println(mic);


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
  if (! bme.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  temperaturefunc();
  Serial.println("Pressure");
  pressurefunc();
  Serial.println("Gas");
  gasfunc();
  Serial.println("Humidity");
  humidityfunc();

  Serial.println("alt");
  Serial.println(bme.readAltitude(SEALEVELPRESSURE_HPA));

  // ===                   MOTION SENSOR                     ===


  val = digitalRead(pirPin);  // read input value

  Serial.println("ms");
  Serial.println(val);

  /*if (val == HIGH) {            // check if the input is HIGH
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
  */

  // ===                   ACCELEROMETER                     ===



  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // these methods (and a few others) are also available
  //accelgyro.getAcceleration(&ax, &ay, &az);
  //accelgyro.getRotation(&gx, &gy, &gz);
  /*
    Serial.println(ax);
    Serial.println(ay);
    Serial.println(az);
    Serial.println(gx);
    Serial.println(gy);
    Serial.println(gz);
    delay(1000);
  */
}


