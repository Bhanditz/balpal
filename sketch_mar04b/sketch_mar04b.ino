// GY-81 Code

#include <Wire.h>
#define BMP085_ADDRESS 0x77  //address of the Barometer

#define BMA180_ADDRESS 0x40  //address of the accelerometer
#define BMA180_RESET 0x10
#define BMA180_PWR 0x0D
#define BMA180_BW 0X20
#define BMA180_RANGE 0X35
#define BMA180_DATA 0x02
#define BMA180_A_TO_READ 6  // 2 bytes for each axis x, y, z

#define ITG3205_ADDRESS 0x68 //address of the Gyroscope
#define ITG3205_SMPLRT_DIV 0x15
#define ITG3205_DLPF_FS 0x16
#define ITG3205_INT_CFG 0x17
#define ITG3205_PWR_MGM 0x3E
#define ITG3205_DATA 0x1B
#define ITG3205_G_TO_READ 8 // 2 bytes for each axis x, y, z, temp

#define HMC5883L_ADDRESS 0x1E
#define HMC5883L_CONFIG_REG_A 0x00
#define HMC5883L_CONFIG_REG_B 0x01
#define HMC5883L_MODE_REG 0x02
#define HMC5883L_MEASURE_CONT 0x00
#define HMC5883L_MEASURE_SINGLE 0x01
#define HMC5883L_MEASURE_IDLE 0x03
#define HMC5883L_DATA 0x03
#define HMC5883L_M_TO_READ 6 // 2 bytes for each axis x, y, z

//-----------BMP085 Barometer Variables---------
const unsigned char OSS = 0;  // Oversampling Setting
// Calibration values
int ac1;
int ac2;
int ac3;
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1;
int b2;
int mb;
int mc;
int md;
// b5 is calculated in bmp085GetTemperature(...), this variable is also used in bmp085GetPressure(...)
// so ...Temperature(...) must be called before ...Pressure(...).
long b5;

//-----------BMA180 Accelerometer Variables----
int a_offx = 31;
int a_offy = 47;
int a_offz = -23;
byte a_resolution = 0x08;
float a_scale = 2048.0;

//-----------ITG3205 Gyroscope Variables----
int g_offx = 120;
int g_offy = 20;
int g_offz = 93;

//-----------HMC5883L Magnetometer Variables----
float m_scale;
int m_error = 0;

//-----------BMP085 Barometer Functions---------
void bmp085Init() {
  ac1 = bmp085ReadInt(0xAA);
  ac2 = bmp085ReadInt(0xAC);
  ac3 = bmp085ReadInt(0xAE);
  ac4 = bmp085ReadInt(0xB0);
  ac5 = bmp085ReadInt(0xB2);
  ac6 = bmp085ReadInt(0xB4);
  b1 = bmp085ReadInt(0xB6);
  b2 = bmp085ReadInt(0xB8);
  mb = bmp085ReadInt(0xBA);
  mc = bmp085ReadInt(0xBC);
  md = bmp085ReadInt(0xBE);
}

short bmp085GetTemperature(unsigned int ut) {
  // Calculate temperature given ut.
  // Value returned will be in units of 0.1 deg C
  // All math from datasheet
  long x1, x2;
  x1 = (((long)ut - (long)ac6) * (long)ac5) >> 15;
  x2 = ((long)mc << 11) / (x1 + md);
  b5 = x1 + x2;
  return ((b5 + 8) >> 4);
}

long bmp085GetPressure(unsigned long up) {
  // Calculate pressure given up
  // calibration values must be known
  // b5 is also required so bmp085GetTemperature(...) must be called first.
  // Value returned will be pressure in units of Pa.
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;
  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6) >> 12) >> 11;
  x2 = (ac2 * b6) >> 11;
  x3 = x1 + x2;
  b3 = (((((long)ac1) * 4 + x3) << OSS) + 2) >> 2;
  // Calculate B4
  x1 = (ac3 * b6) >> 13;
  x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (ac4 * (unsigned long)(x3 + 32768)) >> 15;
  b7 = ((unsigned long)(up - b3) * (50000 >> OSS));
  if (b7 < 0x80000000)
    p = (b7 << 1) / b4;
  else
    p = (b7 / b4) << 1;
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  p += (x1 + x2 + 3791) >> 4;
  return p;
}

char bmp085Read(unsigned char address) {
  // Read 1 byte from the BMP085 at 'address'
  unsigned char data;
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(BMP085_ADDRESS, 1);
  while (!Wire.available());
  return Wire.read();
}

int bmp085ReadInt(unsigned char address) {
  // Read 2 bytes from the BMP085
  // First byte will be from 'address'
  // Second byte will be from 'address'+1
  unsigned char msb, lsb;
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  Wire.requestFrom(BMP085_ADDRESS, 2);
  while (Wire.available() < 2);
  msb = Wire.read();
  lsb = Wire.read();
  return (int) msb << 8 | lsb;
}

unsigned int bmp085ReadUT() {
  // Read the uncompensated temperature value
  unsigned int ut;
  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  // Fills eeprom address 0xF6 with current temp
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();
  // Wait at least 4.5ms
  delay(5);
  // Read two bytes from registers 0xF6 and 0xF7
  ut = bmp085ReadInt(0xF6);
  return ut;
}

unsigned long bmp085ReadUP() {
  // Read the uncompensated pressure value
  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;
  // Write 0x34+(OSS<<6) into register 0xF4
  // Request a pressure reading w/ oversampling setting
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x34 + (OSS << 6));
  Wire.endTransmission();
  // Wait for conversion, delay time dependent on OSS
  delay(2 + (3 << OSS));
  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF6);
  Wire.endTransmission();
  Wire.requestFrom(BMP085_ADDRESS, 3);
  // Wait for data to become available
  while (Wire.available() < 3);
  msb = Wire.read();
  lsb = Wire.read();
  xlsb = Wire.read();
  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8 - OSS);
  return up;
}


//-----------BMA180 Accelerometer Functions----
void bma180Init(int resolution) {
  byte temp[1];
  byte temp1;

  bma180SetResolution(resolution);

  writeTo(BMA180_ADDRESS, BMA180_RESET, 0xB6);
  //wake up mode
  writeTo(BMA180_ADDRESS, BMA180_PWR, 0x10);
  // low pass filter,
  readFrom(BMA180_ADDRESS, BMA180_BW, 1, temp);
  temp1 = temp[0] & 0x0F;
  writeTo(BMA180_ADDRESS, BMA180_BW, temp1);
  // 0x08 is range +/- 4g
  readFrom(BMA180_ADDRESS, BMA180_RANGE, 1 , temp);
  temp1 = (temp[0] & 0xF1) | a_resolution;
  writeTo(BMA180_ADDRESS, BMA180_RANGE, temp1);
}

void bma180SetResolution(int gValue) {
  switch (gValue) {
    case 1:
      a_resolution = 0x00;
      a_scale = 4096.0 * 2;
      break;
    case 2:
      a_resolution = 0x04;
      a_scale = 4096.0;
      break;
    case 3:
      a_resolution = 0x06;
      a_scale = 4096.0 * 2.0 / 3.0;
      break;
    case 4:
      a_resolution = 0x08;
      a_scale = 4096.0 / 2.0;
      break;
    case 8:
      a_resolution = 0x0A;
      a_scale = 4096.0 / 4;
      break;
    case 16:
      a_resolution = 0x0C;
      a_scale = 4096.0 / 8;
      break;
    default:
      a_resolution = 0x04;
      a_scale = 4096.0;
      break;
  }
}

void bma180GetAccelerometerData(int * result) {
  byte buff[BMA180_A_TO_READ];
  readFrom(BMA180_ADDRESS, BMA180_DATA, BMA180_A_TO_READ , buff);

  result[0] = (( buff[0] | buff[1] << 8) >> 2) + a_offx;
  result[1] = (( buff[2] | buff[3] << 8) >> 2) + a_offy;
  result[2] = (( buff[4] | buff[5] << 8) >> 2) + a_offz;
}


//-----------ITG3205 Gyroscope Variables----
void itg3205Init() {
  /*****************************************
    ITG 3200
    power management set to:
    clock select = internal oscillator
    no reset, no sleep mode
    no standby mode
    sample rate to = 125Hz
    parameter to +/- 2000 degrees/sec
    low pass filter = 5Hz
    no interrupt
  ******************************************/
  writeTo(ITG3205_ADDRESS, ITG3205_PWR_MGM, 0x00);
  writeTo(ITG3205_ADDRESS, ITG3205_SMPLRT_DIV, 0x07); // EB, 50, 80, 7F, DE, 23, 20, FF
  writeTo(ITG3205_ADDRESS, ITG3205_DLPF_FS, 0x1E); // +/- 2000 dgrs/sec, 1KHz, 1E, 19
  writeTo(ITG3205_ADDRESS, ITG3205_INT_CFG, 0x00);
}

void itg3205GetGyroscopeData(int * result) {
  /**************************************
    Gyro ITG-3200 I2C
    registers:
    temp MSB = 1B, temp LSB = 1C
    x axis MSB = 1D, x axis LSB = 1E
    y axis MSB = 1F, y axis LSB = 20
    z axis MSB = 21, z axis LSB = 22
  *************************************/
  int temp, x, y, z;
  byte buff[ITG3205_G_TO_READ];
  readFrom(ITG3205_ADDRESS, ITG3205_DATA, ITG3205_G_TO_READ, buff); //read the gyro data from the ITG3200

  result[0] = ((buff[2] << 8) | buff[3]) + g_offx;
  result[1] = ((buff[4] << 8) | buff[5]) + g_offy;
  result[2] = ((buff[6] << 8) | buff[7]) + g_offz;
  result[3] = (buff[0] << 8) | buff[1]; // temperature
}

//-----------HMC5883L Magnetometer Functions----
void hmc5883lInit(float scale, byte mode) {
  m_scale = 1;
  hmc5883lSetScale(scale);
  hmc5883lSetMeasurementMode(mode);
}

void hmc5883lSetScale(float gauss) {
  uint8_t regValue = 0x00;
  if (gauss == 0.88) {
    regValue = 0x00;
    m_scale = 0.73;
  } else if (gauss == 1.3) {
    regValue = 0x01;
    m_scale = 0.92;
  } else if (gauss == 1.9) {
    regValue = 0x02;
    m_scale = 1.22;
  } else if (gauss == 2.5) {
    regValue = 0x03;
    m_scale = 1.52;
  } else if (gauss == 4.0) {
    regValue = 0x04;
    m_scale = 2.27;
  } else if (gauss == 4.7) {
    regValue = 0x05;
    m_scale = 2.56;
  } else if (gauss == 5.6) {
    regValue = 0x06;
    m_scale = 3.03;
  } else if (gauss == 8.1) {
    regValue = 0x07;
    m_scale = 4.35;
  }
  // Setting is in the top 3 bits of the register.
  regValue = regValue << 5;
  writeTo(HMC5883L_ADDRESS, HMC5883L_CONFIG_REG_B, regValue);
}

int hmc5883lSetMeasurementMode(byte mode) {
  writeTo(HMC5883L_ADDRESS, HMC5883L_MODE_REG, mode);
}

void hmc5883GetMagnetometerData(int * result) {
  byte buff[HMC5883L_M_TO_READ];
  readFrom(HMC5883L_ADDRESS, HMC5883L_DATA, HMC5883L_M_TO_READ, buff);

  result[0] = (buff[0] << 8) | buff[1];
  result[1] = (buff[2] << 8) | buff[3];
  result[2] = (buff[4] << 8) | buff[5];
}

//-----------Extra Wire Functions
void writeTo(int DEVICE, byte address, byte val) {
  //Writes val to address register on ACC
  Wire.beginTransmission(DEVICE);   //start transmission to ACC
  Wire.write(address);               //send register address
  Wire.write(val);                   //send value to write
  Wire.endTransmission();           //end trnsmisson
}

void readFrom(int DEVICE, byte address , int num , byte buff[]) {
  //reads num bytes starting from address register in to buff array
  Wire.beginTransmission(DEVICE); //start transmission to ACC
  Wire.write(address);            //send reguster address
  Wire.endTransmission();        //end transmission

  Wire.beginTransmission(DEVICE); //start transmission to ACC
  Wire.requestFrom(DEVICE, num); //request 6 bits from ACC

  int i = 0;
  while (Wire.available()) {       //ACC may abnormal
    buff[i] = Wire.read();       //receive a byte
    i++;
  }
  Wire.endTransmission();
}

/*********************************/

#define CMD_ASD   0
#define CMD_START 1
#define CMD_STOP  2
#define CMD_PLUS  3
#define CMD_MINUS 4

/*********************************/

#define MOTOR_V_MAX 5
#define MOTOR_PIN 9

int motorPrs = 40;
int motorVal;
int MotorIn;

/*********************************/

#define INCOMING_BUFFER_LENGTH 5

unsigned int incomingBytesIdx = 0;
int *incomingBytesBuffer = (int*)malloc(INCOMING_BUFFER_LENGTH);

/*********************************/

bool running = true;

void start() {
  //  Serial.println("STARTING");
  running = true;
  digitalWrite(LED_BUILTIN, HIGH);
}
void stop() {
  //  Serial.println("STOPPING");
  running = false;
  digitalWrite(LED_BUILTIN, LOW);
  analogWrite(MOTOR_PIN, 0);
}

/*********************************/

void setup() {
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(9600);
  Wire.begin();

  bmp085Init();
  Serial.println("Barometer has been initialized");

  bma180Init(8);
  Serial.println("Accelerometer has been initialized");

  itg3205Init();
  Serial.println("Gyro has been initialized");

  hmc5883lInit(1.3, HMC5883L_MEASURE_CONT);
  Serial.println("Gyro has been initialized");

  start();
}

/*********************************/

int setMotorPrs(int prs) {
  if ( prs <   0 ) prs = 0;
  if ( prs > 100 ) prs = 100;
  motorPrs = prs;
  motorVal = ((MOTOR_V_MAX / 5.0) * 255.0) * (motorPrs / 100.0);
  return motorVal;
}

int gyrox[4];
int adelta;
int iterations = 0;

void loop() {
  iterations++;

  itg3205GetGyroscopeData(gyrox);
  adelta = (gyrox[1] - 64) / 5.0;

  setMotorPrs(adelta);

  if ( adelta < 0 ) {
  } else {
    analogWrite(MOTOR_PIN, motorVal);
  }

  if ( iterations % 100 == 0 ) {
    Serial.println(motorVal);
  }
}
