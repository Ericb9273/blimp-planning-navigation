#include <Wire.h>

// I2C Address
#define LIDAR_ADDRESS 0x10
#define IMU_ADDRESS 0x68
#define BAROMETER_ADDRESS 0x63


// Barometer Code
#include <DFRobot_ICP10111.h>
DFRobot_ICP10111 icp;
void barometer_setup()
{
    Serial.begin(115200);
    while(icp.begin() != 0){
      //Serial.println("Failed to initialize the sensor");
      }
     //Serial.println("Success to initialize the sensor");
     /**
      * @brief 设置工作模式
      * |------------------|-----------|-------------------|----------------------|
      * |       api        |   mode    |Conversion Time(ms)|Pressure RMS Noise(Pa)|
      * |icp.eLowPower     |  低功耗   |      1.8          |        3.2           |
      * |icp.eNormal       |  正常模式 |      6.3          |        1.6           |
      * |icp.eLowNoise     |  低噪声   |      23.8         |        0.8           |
      * |icp.eUltraLowNoise|  超低噪声 |      94.5         |        0.4           |
      */
     icp.setWorkPattern(icp.eUltraLowNoise);
}
double elevation = 0.0;
void barometer_getvalue()
{
  //Serial.print("Read air pressure:");
  //Serial.print(icp.getAirPressure());
  //Serial.println("Pa");
  //Serial.print("Read temperature:");
//  Serial.print(icp.getTemperature());
//  Serial.println("℃");
//  Serial.print("Read altitude:");
//  Serial.print(icp.getElevation());
//  Serial.println("m");
    elevation = icp.getElevation();
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

double Dist = 0.0;
void Lidar_getvalue()
{
    tfmP.getData( tfDist, tfFlux, tfTemp); // Get a frame of data
    if( tfmP.status == TFMP_READY)         // If no error...
    {
        //printf( "Dist:%04icm ", tfDist);
        Dist = tfDist; // display distance,
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


// 初始化卡尔曼滤波器参数
float x_est_last = 0; // 初始状态
float P_last = 0; // 初始估计协方差
// 噪声参数
float Q = 0.022; // 过程噪声协方差
float R = 0.617; // 测量噪声协方差

double kalmanfilter (double h1, double h2) {
  float x_pred = x_est_last;
  float P_pred = P_last + Q;

  // 更新步骤
  float K = P_pred / (P_pred + R);
  float x_est = x_pred + K * (h1 - x_pred); // 使用h1更新
  float P_est = (1 - K) * P_pred;

  // 使用h2更新
  K = P_est / (P_est + R);
  x_est = x_est + K * (h2 - x_est);
  P_est = (1 - K) * P_est;

  // 保存状态
  x_est_last = x_est;
  P_last = P_est;

  Serial.print("Height: ");
  Serial.print(x_est);
  Serial.println("cm");

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
  // LIDAR
  Wire.beginTransmission(LIDAR_ADDRESS);
  Lidar_getvalue();
  Wire.endTransmission();
  
  // IMU
  Wire.beginTransmission(IMU_ADDRESS);
  IMU_getvalue();
  Wire.endTransmission();

  // Barometer
  Wire.beginTransmission(BAROMETER_ADDRESS);
  barometer_getvalue();
  Wire.endTransmission();

  double Altitude = Dist * cos(pitch)* cos(roll);
  Serial.print("Dist: ");
  Serial.println(Dist);
  Serial.print("Altitude:");
  Serial.println(Altitude);
  //kalmanfilter(Altitude, (elevation - 238)*100);
  delay(25);
}
