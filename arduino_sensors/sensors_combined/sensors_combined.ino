#include <Wire.h>

// I2C Address
#define LIDAR_ADDRESS 0x10
#define IMU_ADDRESS 0x68
#define BAROMETER_ADDRESS 0x77
#define IR_ADDRESS 0x31  



// Lidar code
#include <Wire.h>     // Arduino standard I2C/Two-Wire Library
#include "printf.h"
#include <TFMPI2C.h>  // TFMini-Plus I2C Library v1.7.3
TFMPI2C tfmP;         // Create a TFMini-Plus I2C object

void Lidar_setup()
{
    Serial.begin(115200);   // Initialize terminal serial port
    printf_begin();          // Initialize printf library.
    delay(20);
    
    printf("\n");            // say 'hello'
    printf( "TFMPlus I2C Library Example - 14JAN2022");
    printf("\n\n");

    tfmP.recoverI2CBus();
    printf( "System reset: ");
    if( tfmP.sendCommand( SOFT_RESET, 0))
    {
        printf( "passed.\r\n");
    }
    else tfmP.printReply();  // This response and 'printStatus()' are for
                             // troubleshooting and not strictly necessary.
    //
    // - - Display the firmware version - - - - - - - - -
    printf( "Firmware version: ");
    if( tfmP.sendCommand( GET_FIRMWARE_VERSION, 0))
    {
        printf( "%1u.",  tfmP.version[ 0]); // print three single numbers
        printf( "%1u.",  tfmP.version[ 1]); // each separated by a dot
        printf( "%1u\n", tfmP.version[ 2]);
    }
    else tfmP.printReply();
    //
    // - - Set the data frame-rate to 20 - - - - - - - - -
    printf( "Data-Frame rate: ");
    if( tfmP.sendCommand( SET_FRAME_RATE, FRAME_20))
    {
        printf( "%2uHz.\n", FRAME_20);
    }
    else tfmP.printReply();
    // - - - - -   End of example commands- - - - - - - - - -

    delay(500);            // And wait for half a second.
}

// Initialize data variables
int16_t tfDist = 0;       // Distance to object in centimeters
int16_t tfFlux = 0;       // Signal strength or quality of return signal
int16_t tfTemp = 0;       // Internal temperature of Lidar sensor chip

double lidar_distance = 0.0;
void Lidar_getvalue()
{
    tfmP.getData( tfDist, tfFlux, tfTemp); // Get a frame of data
    if( tfmP.status == TFMP_READY)         // If no error...
    {
        //printf( "Dist:%04icm ", tfDist);
        lidar_distance = (double)tfDist / 100; // display distance,
        //printf( "Flux:%05i ", tfFlux);     // display signal strength/quality,
        //printf( "Temp:%2i%s", tfTemp, "°C" );   // display temperature,
        //printf( "\n");                     // end-of-line.
    }
    else
    {
        tfmP.printFrame();                 // Display error and data frame
        if( tfmP.status == TFMP_I2CWRITE)  // If I2C error...
        {
            tfmP.recoverI2CBus();          // recover hung bus.
        }
    }
}




// Barometer Code
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp;
void barometer_setup()
{
    Serial.begin(115200);
    while (!Serial);
    Serial.println("Adafruit BMP388 / BMP390 test");

    if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
    //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
    //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

    // Set up oversampling and filter initialization
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

double init_altitude = 0;
double base_distance = 0;
void initialize_altitude() {
  for(int i=0; i < 20; ++i ){
    init_altitude = init_altitude + bmp.readAltitude(SEALEVELPRESSURE_HPA);
    tfmP.getData( tfDist, tfFlux, tfTemp);
    base_distance = base_distance + tfDist;
    delay(100);
  }
  init_altitude = init_altitude / 20;
  //Serial.println(init_altitude);
  base_distance = base_distance / 20;
  //Serial.println(base_distance);
}

bool whether_init_altitude = false;
double baro_altitude = 0;
void barometer_getvalue()
{
  if (!whether_init_altitude) {
    delay(200);
    initialize_altitude();
    whether_init_altitude = true;
    init_altitude = 200;
  }
  //Serial.println(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  baro_altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA) - init_altitude + base_distance;
}


// IMU Code
#include <ICM20948_WE.h>
#define ICM20948_ADDR 0x68
ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR);

void IMU_setup() {
  Serial.begin(115200);  // activating serial
  Wire.begin();
  while (!Serial) {}
  if (!myIMU.init()) {
    //Serial.println("ICM20948 does not respond");
  } else {
    //Serial.println("ICM20948 is connected");
  }
//  myIMU.setAccOffsets(-16330.0, 16450.0, -16600.0, 16180.0, -16640.0, 16560.0);

  //Serial.println("Position your ICM20948 flat and don't move it - calibrating...");
  delay(1000);
  myIMU.autoOffsets();
  //Serial.println("Done!");

  /* enables or disables the gyroscope sensor, default: enabled */
   myIMU.enableGyr(true);
   myIMU.enableAcc(true);

  myIMU.setGyrRange(ICM20948_GYRO_RANGE_250);

  /*  Choose a level for the Digital Low Pass Filter or switch it off. 
   *  ICM20948_DLPF_0, ICM20948_DLPF_2, ...... ICM20948_DLPF_7, ICM20948_DLPF_OFF 
   *  
   *  DLPF       3dB Bandwidth [Hz]      Output Rate [Hz]
   *    0              196.6               1125/(1+GSRD) 
   *    1              151.8               1125/(1+GSRD)
   *    2              119.5               1125/(1+GSRD)
   *    3               51.2               1125/(1+GSRD)
   *    4               23.9               1125/(1+GSRD)
   *    5               11.6               1125/(1+GSRD)
   *    6                5.7               1125/(1+GSRD) 
   *    7              361.4               1125/(1+GSRD)
   *    OFF          12106.0               9000
   *    
   *    GSRD = Gyroscope Sample Rate Divider (0...255)
   *    You achieve lowest noise using level 6  
   */
  myIMU.setGyrSampleRateDivider(28);
  myIMU.setGyrDLPF(ICM20948_DLPF_6);
  myIMU.setAccDLPF(ICM20948_DLPF_6);
}


#include <algebra.h>
#include <altitude.h>
#include <filters.h>
#include "Quaternion.hpp"
Quaternion orientation;
#define to_rad 3.14159265359 / 180.0
uint32_t last_read = 0;
double roll = 0.0;
double pitch = 0.0;
double yaw = 0.0;
void IMU_getvalue(){
  myIMU.readSensor();
  uint32_t read = millis();
  uint32_t dt = read - last_read;
  last_read = read;
  xyzFloat gyr = myIMU.getGyrValues();
  xyzFloat corrAccRaw = myIMU.getCorrectedAccRawValues();
  //Serial.print("Gyroscope: ");
  //Serial.print(gyr.x);
  //Serial.print(" ");
  //Serial.print(gyr.y);
  //Serial.print(" ");
  //Serial.println(gyr.z);
  float omega[3] = {gyr.x * to_rad, gyr.y * to_rad, gyr.z * to_rad};
  float accel[3] = {corrAccRaw.x, corrAccRaw.y, corrAccRaw.z};
      float dt_s = dt / 1000.0;
      float dtheta[3] = {omega[0] * dt_s, omega[1] * dt_s, omega[2] * dt_s};
      orientation = orientation * Quaternion::from_rotvec(dtheta);
      Quaternion::to_euler(orientation, pitch, roll, yaw);

      //Serial.print("orientation: ");
      //Serial.print(orientation.w);
      //Serial.print(" ");
      //Serial.print(orientation.x);
      //Serial.print(" ");
      //Serial.print(orientation.y);
      //Serial.print(" ");
      //Serial.println(orientation.z);

      //Serial.print("Yaw: ");
      //Serial.println(yaw);
      //Serial.print(" Pitch: ");
      //Serial.println(pitch);
      //Serial.print(" Roll: ");
      //Serial.println(roll);
  }



//IR Code

// For TeraRanger One
// #define SENSOR_ADDR 0x30

// Create a Cyclic Redundancy Checks table used in the "crc8" function
static const uint8_t crc_table[] = {
    0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31,
    0x24, 0x23, 0x2a, 0x2d, 0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65,
    0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d, 0xe0, 0xe7, 0xee, 0xe9,
    0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
    0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1,
    0xb4, 0xb3, 0xba, 0xbd, 0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2,
    0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea, 0xb7, 0xb0, 0xb9, 0xbe,
    0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
    0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16,
    0x03, 0x04, 0x0d, 0x0a, 0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42,
    0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a, 0x89, 0x8e, 0x87, 0x80,
    0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
    0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8,
    0xdd, 0xda, 0xd3, 0xd4, 0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c,
    0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44, 0x19, 0x1e, 0x17, 0x10,
    0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
    0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f,
    0x6a, 0x6d, 0x64, 0x63, 0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b,
    0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13, 0xae, 0xa9, 0xa0, 0xa7,
    0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
    0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef,
    0xfa, 0xfd, 0xf4, 0xf3
};

/*
 * Brief : Calculate a Cyclic Redundancy Checks of 8 bits
 * Param1 : (*p) pointer to receive buffer
 * Param2 : (len) number of bytes returned by the TeraRanger
 * Return : (crc & 0xFF) checksum calculated locally
 */
uint8_t crc8(uint8_t *p, uint8_t len) {
  uint8_t i;
  uint8_t crc = 0x0;
  while (len--) {
    i = (crc ^ *p++) & 0xFF;
    crc = (crc_table[i] ^ (crc << 8)) & 0xFF;
  }
  return crc & 0xFF;
}

uint8_t buf[3];           // The variable "buf[3]" will contain the frame sent by the TeraRanger
uint16_t IRdistance = 0;    // The variable "distance" will contain the distance value in millimeter
uint8_t CRC = 0;          // The variable "CRC" will contain the checksum to compare at TeraRanger's one
double IR_distance = 0;

void IR_getvalue(){
  Wire.write(0x00);  
  Wire.requestFrom(IR_ADDRESS, 3);          // Read back three bytes from TR1 (THIS IS THE I2C BASE ADDRESS, CHANGE HERE IN CASE IT IS DIFFERENT)
  buf[0] = Wire.read();               // First byte of distance
  buf[1] = Wire.read();               // Second byte of distance
  buf[2] = Wire.read();               // Byte of checksum
  
  CRC = crc8(buf, 2);                 // Save the "return" checksum in variable "CRC" to compare with the one sent by the TeraRanger
  
  if (CRC == buf[2]) {                 // If the function crc8 return the same checksum than the TeraRanger, then:
    IRdistance = (buf[0]<<8) + buf[1];    // Calculate distance in mm                   // Start of the ERASABLE part                             //                                                   //
  }
  else {                                                    //
    Serial.println("CRC error!"); }
  IR_distance = (double)IRdistance / 1000;
  
  }



// 初始化卡尔曼滤波器参数
double x_est_last = 0; // 初始状态
double P_last = 1; // 初始估计协方差，设为非零值
// 噪声参数
double Q = 0.022; // 过程噪声协方差
double R = 0.617; // 测量噪声协方差

double kalmanfilter(double h1, double h2) {
  double x_pred = x_est_last;
  double P_pred = P_last + Q;

  // 更新步骤
  double K = P_pred / (P_pred + R);
  double x_est = x_pred + K * (h1 - x_pred); // 使用h1更新
  double P_est = (1 - K) * P_pred;

  // 使用h2更新，调整权重
  double weight_h1 = 0.3; // h1的权重
  double weight_h2 = 0.7; // h2的权重
  K = P_est / (P_est + R);
  x_est = x_est + K * (weight_h1 * h1 + weight_h2 * h2 - x_est);
  P_est = (1 - K) * P_est;

  // 保存状态
  x_est_last = x_est;
  P_last = P_est;

  Serial.print("Height: ");
  Serial.print(x_est);
  Serial.println("cm");

  return x_est;
}



void setup() {
  Wire.begin(); // 初始化I2C
  Serial.begin(115200); // 初始化串行通信
  Lidar_setup();
  IMU_setup();
  barometer_setup();
}

void loop() {
  Serial.println("------------------------------");
  
  // Barometer
  Wire.beginTransmission(BAROMETER_ADDRESS);
  barometer_getvalue();
  Wire.endTransmission();
  
  // LIDAR
  Wire.beginTransmission(LIDAR_ADDRESS);
  Lidar_getvalue();
  Wire.endTransmission();
  
  // IMU
  Wire.beginTransmission(IMU_ADDRESS);
  IMU_getvalue();
  Wire.endTransmission();

  // IR
  Wire.beginTransmission(IR_ADDRESS);
  IR_getvalue();
  Wire.endTransmission();

  double lidar_Altitude = lidar_distance * cos(pitch)* cos(roll);
  double IR_Altitude = IR_distance * cos(pitch)* cos(roll);
  Serial.print("lidar_Altitude:");
  Serial.print(lidar_Altitude);
  Serial.print("\t");
  Serial.print("IR_Altitude:");
  Serial.println(IR_Altitude);
  //Serial.println(baro_altitude);
  //kalmanfilter(Altitude,  baro_altitude);
  delay(200);
}
