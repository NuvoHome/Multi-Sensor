#include <ArduinoJson.h>

#include <Adafruit_BME680.h>

#include <Adafruit_TCS34725.h>
#include <I2Cdev.h>
#include <QuickStats.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <Adafruit_AMG88xx.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <WiFi101.h>

#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

QuickStats stats;

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

bool blinkState = false;



/* IMU Data */

// TODO: Make calibration routine

#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 4
#define BME_CS 8
#define MAG_ADDR  0x0E //7-bit address for the MAG3110, doesn't change
#define SEALEVELPRESSURE_HPA (1013.25)
#define SECRET_SSID "Home"
#define SECRET_PASS "Casabl12"
Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO,  BME_SCK);
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

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
int row1 = 0;
int row2 = 0;
int row3 = 0;
int row4 = 0;
int row5 = 0;
int row6 = 0;
int row7 = 0;
int row8 = 0;
int column1 = 0;
int column2 = 0;
int column3 = 0;
int column4 = 0;
int column5 = 0;
int column6 = 0;
int column7 = 0;
int column8 = 0;
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t ax, ay, az;
int16_t gx, gy, gz;
char ssid[] = SECRET_SSID;  
char pass[] = SECRET_PASS;       

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data
const uint8_t IMUAddress = 0x68; // AD0 is logic low on the PCB
const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication
int status = WL_IDLE_STATUS;     // the WiFi radio's status

// TODO: Make calibration routine


void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }

  // attempt to connect to WiFi network:
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }

  // you're connected now, so print out the data:
  Serial.print("You're connected to the network");

  Serial.begin(115200);

  // ===                   MICROPHONE .                      ===

  pinMode(mic, INPUT);
  // ===                   Magnetometer .                      ===
  Wire.begin();
  Wire.beginTransmission(MAG_ADDR); // transmit to device 0x0E
  Wire.write(0x11);             // cntrl register2
  Wire.write(0x80);             // write 0x80, enable auto resets
  Wire.endTransmission();       // stop transmitting

  delay(15);

  Wire.beginTransmission(MAG_ADDR); // transmit to device 0x0E
  Wire.write(0x10);             // cntrl register1
  Wire.write(1);                // write 0x01, active mode
  Wire.endTransmission();
  // ===                   ACCELEROMETER                     ===
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif


  // turn the MAG3110 on


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

    // Connect to WPA/WPA2 network:

//accelconfig();
//
//  mpu.initialize();
}




// ================================================================
// ===                    Functions .                           ===
// ================================================================
// ===                   Magnetometer                         ===



void print_values(void)
{

//  root["Magnetometerx"] = read_x();
  Serial.println("x");
  Serial.println(read_x());
//root["Magnetometery"] = read_y();

  Serial.println("y");
  Serial.println(read_y());
//root["Magnetometerz"] = read_z();
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
  while (Wire.available())   // slave may write less than requested
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

  int out = (val_low | (val_high << 8)); //concatenate the MSB and LSB
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
  for (int tempIndex = 0; tempIndex < 10; tempIndex++)
  {
    temperature[tempIndex] = bme.temperature;
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
    /*    for (int i = 0; i < 10; i++)
        {
          Serial.println(temperature[i]);
        }
        Serial.println(bme.temperature);
    */
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
void accelconfig() {
  Wire.begin();
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();
}
uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop) {
  return i2cWrite(registerAddress, &data, 1, sendStop); // Returns 0 on success
}

uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
  if (rcode) {
    Serial.print(F("i2cWrite failed: "));
    Serial.println(rcode);
  }
  return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
}

uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
  uint32_t timeOutTimer;
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
  if (rcode) {
    Serial.print(F("i2cRead failed: "));
    Serial.println(rcode);
    return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
  }
  Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true); // Send a repeated start and then release the bus after reading
  for (uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available())
      data[i] = Wire.read();
    else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if (Wire.available())
        data[i] = Wire.read();
      else {
        Serial.println(F("i2cRead timeout"));
        return 5; // This error value is not already taken by endTransmission
      }
    }
  }
  return 0; // Success
}
void Accelgyro() {
  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  /* Print Data */
  /*#if 0 // Set to 1 to activate
    Serial.print(accX); Serial.print("\t");
    Serial.print(accY); Serial.print("\t");
    Serial.print(accZ); Serial.print("\t");

    Serial.print(gyroX); Serial.print("\t");
    Serial.print(gyroY); Serial.print("\t");
    Serial.print(gyroZ); Serial.print("\t");

    Serial.print("\t");
    #endif*/

  // Serial.print(roll); Serial.print("\t");
  // Serial.print(gyroXangle); Serial.print("\t");
 // Serial.print(compAngleX); Serial.print("\t");
  Serial.println("fgyrox");
  Serial.println(kalAngleX);

  //Serial.print("\t");

  //Serial.print(pitch); Serial.print("\t");
  // Serial.print(gyroYangle); Serial.print("\t");
  //Serial.print(compAngleY); Serial.print("\t");
  Serial.println("fgyroy");
Serial.println(kalAngleY);
}
// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
StaticJsonBuffer<200> jsonBuffer;
JsonObject& root = jsonBuffer.createObject();
JsonObject& micjson = jsonBuffer.createObject();

root["Magnetometerx"] = read_x;
root["Magnetometery"] = read_y;
root["Magnetometerz"] = read_z;
root["Temperature"] = bme.temperature;
root["Pressure"] = bme.pressure / 100.0;
root["Gas"] = bme.gas_resistance / 1000.0;
root["Humidity"] = bme.humidity;

  /*frame.createFrame(ASCII);
    frame.addSensor("Microphone Average", stats.average(readings, readIndex) );
    frame.addSensor("Microphone Median", stats.median(readings, readIndex) );
    frame.showFrame();
  */
  // ===                   ACCELEROMETER                     ===
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

//  Accelgyro();
//  Serial.println("accelx");
//  Serial.println(ax);
//  Serial.println("accely");
//  Serial.println(ay);
//  Serial.println("accelz");
//  Serial.println(az);
//  Serial.println("gyrox");
//  Serial.println(gx);
//  Serial.println("gyroy");
//  Serial.println(gy);
//  Serial.println("gyroz");
//  Serial.println(gz);


  // ===                   MICROPHONE                        ===

  mic = analogRead(mic);

  Serial.println("mic");
  Serial.println(mic);
  micjson["Microphone"] = mic;

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

  // ===                      Color Illumination                     ===
//  if (micros() - lastMicros1 > INTERVAL1) {
//    lastMicros1 = micros(); // do this first or your interval is too long!
    uint16_t r, g, b, c, colorTemp, lux;

    tcs.getRawData(&r, &g, &b, &c);
    colorTemp = tcs.calculateColorTemperature(r, g, b);
    lux = tcs.calculateLux(r, g, b);
    root["l"] = lux;


    root["r"] = r;
     root["g"] = g;
    root["b"] = b;
    root["ct"] = colorTemp;
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







    // ===                   Magnetometer                         ===

//    print_values();
  root.printTo(Serial);
  jsonBuffer.clear();

    //    ===                   Thermal Camera .                  ===
    amg.readPixels(pixels);

    thermalAvg();




    // ===                TEMPERATURE + HUMIDITY               ===
    if (! bme.performReading()) {
      Serial.println("Failed to perform reading :(");
      return;
    }
//    temperaturefunc();
//    pressurefunc();
//    gasfunc();
//    humidityfunc();
    Serial.println("alt");
    Serial.println(bme.readAltitude(SEALEVELPRESSURE_HPA));
  //}

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




}




