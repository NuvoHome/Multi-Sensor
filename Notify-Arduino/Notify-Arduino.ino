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
#define MAG_ADDR  0x0E //7-bit address for the MAG3110, doesn't change
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
float row1 = 0;
float row2 = 0;
float row3 = 0;
float row4 = 0;
float row5 = 0;
float row6 = 0;
float row7 = 0;
float row8 = 0;
float column1 = 0;
float column2 = 0;
float column3 = 0;
float column4 = 0;
float column5 = 0;
float column6 = 0;
float column7 = 0;
float column8 = 0;


void setup() {

  // ===                   MICROPHONE .                      ===

  pinMode(mic, INPUT);
  // ===                   Magnetometer .                      ===
  Wire.begin();

    config();            // turn the MAG3110 on


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
// ===                   Magnetometer                         ===

void config(void)
{
  Wire.beginTransmission(MAG_ADDR); // transmit to device 0x0E
  Wire.write(0x11);             // cntrl register2
  Wire.write(0x80);             // write 0x80, enable auto resets
  Wire.endTransmission();       // stop transmitting
  
  delay(15);
  
  Wire.beginTransmission(MAG_ADDR); // transmit to device 0x0E
  Wire.write(0x10);             // cntrl register1
  Wire.write(1);                // write 0x01, active mode
  Wire.endTransmission();       // stop transmitting
}

void print_values(void)
{
  Serial.println("x");
  Serial.println(read_x());  
  Serial.println("y");    
  Serial.println(read_y());       
  Serial.println("z");    
  Serial.println(read_z());
}

int mag_read_register(int reg)
{
  int reg_val;
  
  Wire.beginTransmission(MAG_ADDR); // transmit to device 0x0E
  Wire.write(reg);              // x MSB reg
  Wire.endTransmission();       // stop transmitting
 
  delayMicroseconds(2); //needs at least 1.3us free time between start and stop
  
  Wire.requestFrom(MAG_ADDR, 1); // request 1 byte
  while(Wire.available())    // slave may write less than requested
  { 
    reg_val = Wire.read(); // read the byte
  }
  
  return reg_val;
}

int mag_read_value(int msb_reg, int lsb_reg)
{
  int val_low, val_high;  //define the MSB and LSB
  
  val_high = mag_read_register(msb_reg);
  
  delayMicroseconds(2); //needs at least 1.3us free time between start and stop
  
  val_low = mag_read_register(lsb_reg);
  
  int out = (val_low|(val_high << 8)); //concatenate the MSB and LSB
  return out;
}

int read_x(void)
{
  return mag_read_value(0x01, 0x02);
}

int read_y(void)
{
  return mag_read_value(0x03, 0x04);
}

int read_z(void)
{
  return mag_read_value(0x05, 0x06);
}
// ===                   Temperature                        ===

void temperaturefunc() {

  for (int tempIndex = 0; tempIndex < 10;)
  {
    temperature[tempIndex] = bme.temperature;
    tempIndex++;


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
  if (tempIndex > tempReadings)
  {
    tempIndex = 0;
  }
}
// ===                   Pressure                        ===

void pressurefunc() {
  for (int pressureIndex = 0; pressureIndex < 10; pressureIndex++)
  {
    pressure[pressureIndex] = bme.pressure / 100.0;
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
  if (pressureIndex > tempReadings)
  {
    pressureIndex = 0;
  }
}
// ===                   Gas                        ===

void gasfunc() {

  for (int gasIndex = 0; gasIndex < 10; gasIndex++)
  {
    gas[gasIndex] = bme.gas_resistance / 1000.0;
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
  if (gasIndex > gasReadings)
  {
    gasIndex = 0;
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
    Serial.println("ha");
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
// ===                   Thermal Camera Row + Column AVG                        ===

void thermalAvg() {
  // ===                              Row                         ===

  row1 = pixels[0]  + pixels[1]  + pixels[2]  + pixels[3]  + pixels[4]  + pixels[5]  + pixels[6]  + pixels[7];
  row2 = pixels[8]  + pixels[9]  + pixels[10] + pixels[11] + pixels[12] + pixels[13] + pixels[14] + pixels[15];
  row3 = pixels[16] + pixels[17] + pixels[18] + pixels[19] + pixels[20] + pixels[21] + pixels[22] + pixels[23];
  row4 = pixels[24] + pixels[25] + pixels[26] + pixels[27] + pixels[28] + pixels[29] + pixels[30] + pixels[31];
  row5 = pixels[32] + pixels[33] + pixels[34] + pixels[35] + pixels[36] + pixels[37] + pixels[38] + pixels[39];
  row6 = pixels[40] + pixels[41] + pixels[42] + pixels[43] + pixels[44] + pixels[45] + pixels[46] + pixels[47];
  row7 = pixels[48] + pixels[49] + pixels[50] + pixels[51] + pixels[52] + pixels[53] + pixels[54] + pixels[55];
  row8 = pixels[56] + pixels[57] + pixels[58] + pixels[59] + pixels[60] + pixels[61] + pixels[62] + pixels[63];
  Serial.println("r1");
  Serial.println(row1 / 8);
  Serial.println("r2");
  Serial.println(row2 / 8);
  Serial.println("r3");
  Serial.println(row3 / 8);
  Serial.println("r4");
  Serial.println(row4 / 8);
  Serial.println("r5");
  Serial.println(row5 / 8);
  Serial.println("r6");
  Serial.println(row6 / 8);
  Serial.println("r7");
  Serial.println(row7 / 8);
  Serial.println("r8");
  Serial.println(row8 / 8);
  // ===                              Column                         ===

  column1 = pixels[0] + pixels[8] + pixels[16] + pixels[24] + pixels[32] + pixels[40] + pixels[48] + pixels[56];
  column2 = pixels[1] +  pixels[9] + pixels[17] + pixels[25] + pixels[33] + pixels[41] + pixels[49] + pixels[57];
  column3 = pixels[2] + pixels[10] + pixels[18] + pixels[26] + pixels[34] + pixels[42] + pixels[50] + pixels[58];
  column4 = pixels[3] + pixels[11] + pixels[19] + pixels[27] + pixels[35] + pixels[43] + pixels[51] + pixels[59];
  column5 = pixels[4] + pixels[12] + pixels[20] + pixels[28] + pixels[36] + pixels[44] + pixels[52] + pixels[60];
  column6 = pixels[5] + pixels[13] + pixels[21] + pixels[29] + pixels[37] + pixels[45] + pixels[53] + pixels[61];
  column7 = pixels[6] + pixels[14] + pixels[22] + pixels[30] + pixels[38] + pixels[46] + pixels[54] + pixels[62];
  column8 = pixels[7] + pixels[15] + pixels[23] + pixels[31] + pixels[39] + pixels[47] + pixels[55] + pixels[63];
  Serial.println("c1");
  Serial.println(column1 / 8);
  Serial.println("c2");
  Serial.println(column2 / 8);
  Serial.println("c3");
  Serial.println(column3 / 8);
  Serial.println("c4");
  Serial.println(column4 / 8);
  Serial.println("c5");
  Serial.println(column5 / 8);
  Serial.println("c6");
  Serial.println(column6 / 8);
  Serial.println("c7");
  Serial.println(column7 / 8);
  Serial.println("c8");
  Serial.println(column8 / 8);


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
  // ===                   Magnetometer                         ===

  print_values();

  //    ===                   Thermal Camera .                  ===
  amg.readPixels(pixels);

  Serial.println("[");
  for (int i = 1; i <= AMG88xx_PIXEL_ARRAY_SIZE; i++) {
    Serial.print(pixels[i - 1]);
    Serial.print(", ");
    if ( i % 8 == 0 ) Serial.println();
  }
  Serial.println("]");

  thermalAvg();


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

  for (int i = 0; i < 10; i++)
  {
    Serial.println(temperature[i]);
  }
  Serial.println(bme.temperature);

  temperaturefunc();
  pressurefunc();


  gasfunc();


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
   Serial.println("accelx");
    Serial.println(ax);
      Serial.println("accely");
    Serial.println(ay);
      Serial.println("accelz");
    Serial.println(az);
      Serial.println("gyrox");
    Serial.println(gx);
      Serial.println("gyroy");
    Serial.println(gy);
      Serial.println("gyroz");
    Serial.println(gz);
 
}



