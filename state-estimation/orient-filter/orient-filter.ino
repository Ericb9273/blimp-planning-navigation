#include <algebra.h>
#include <altitude.h>
#include <filters.h>

#include <algebra.h>
#include <altitude.h>
#include <filters.h>

#include <algebra.h>
#include <altitude.h>
#include <filters.h>

//Orient-Filter

#include <Servo.h>
#include <Wire.h>
#include <ICM20948_WE.h> // IMU
#include <DFRobot_ICP10111.h> // Barometer
#include "Quaternion.hpp"
#include "altitude.h" // Altitude Estimator


#define ICM20948_ADDR 0x68
#define channumber 8      //8 channels of pwm in total.
#define to_rad 3.14159265359 / 180.0
int channel[channumber];  //read Channel values
int PPMin = 13;           // PPM signal sending from rc receiver to the D13 pin

ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR); // IMU
DFRobot_ICP10111 icp; // Barometer


// Initializing Servo objects:
Servo right_motor;
Servo left_motor;
Servo back_motor;

Servo right_servo;
Servo left_servo;
Servo capture;


uint32_t mytime = 0;

// The corresponding digital pins for sending signal to the servos.
int right_motor_output_pin = 5;
int right_servo_pin = 4;

int left_motor_output_pin = 2;
int left_servo_pin = 3;

int back_motor_output_pin = 7;

// barometer state variables

// imu state variables
bool imu_state = true;
uint32_t last_read = 0; //time of last read

// gyr state variables
bool gyr_state = true;

// accel state variables
bool accel_state = true;

// orientation estimator varianles
bool do_orient = true;
Quaternion orientation;

// altitude estimation object
bool do_alt = true ;
AltitudeEstimator altitude = AltitudeEstimator(0.0005, 	// sigma Accel
                                               0.0005, 	// sigma Gyro
                                               0.018,   // sigma Baro
                                               0.5, 	// ca
                                               0.1);	// accelThreshold

// signed objects for storing decoded pwm signals
signed servo_right_pulse, servo_left_pulse, right_motor_pulse, left_motor_pulse, back_motor_pulse,
  capture_angle;


void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);  // activating serial

  Wire.begin();
  while (!Serial) {}

  if (!myIMU.init()) {
    Serial.println("ICM20948 does not respond");
  } else {
    Serial.println("ICM20948 is connected");
  }

  // myIMU.setAccOffsets(-16330.0, 16450.0, -16600.0, 16180.0, -16640.0, 16560.0);
  Serial.println("Position your ICM20948 flat and don't move it - calibrating...");
  delay(1000);
  myIMU.autoOffsets();
  myIMU.setAccRange(ICM20948_ACC_RANGE_2G); // Default
  Serial.println("Done!");

  /*  The gyroscope data is not zero, even if you don't move the ICM20948. 
   *  To start at zero, you can apply offset values. These are the gyroscope raw values you obtain
   *  using the +/- 250 degrees/s range. 
   *  Use either autoOffsets or setGyrOffsets, not both.
   */
  //myIMU.setGyrOffsets(-115.0, 130.0, 105.0);

  /* enables or disables the gyroscope sensor, default: enabled */
   myIMU.enableGyr(gyr_state);
   myIMU.enableAcc(accel_state);

  /*  ICM20948_GYRO_RANGE_250       250 degrees per second (default)
   *  ICM20948_GYRO_RANGE_500       500 degrees per second
   *  ICM20948_GYRO_RANGE_1000     1000 degrees per second
   *  ICM20948_GYRO_RANGE_2000     2000 degrees per second
   */
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

  icp.setWorkPattern(icp.eUltraLowNoise);

  pinMode(PPMin, INPUT);  // setting up PPMin pin as input pin.

  // attach motors to corresponding pin
  right_motor.attach(right_motor_output_pin);
  left_motor.attach(left_motor_output_pin);
  back_motor.attach(back_motor_output_pin);
  right_servo.attach(right_servo_pin);
  left_servo.attach(left_servo_pin);
  capture.attach(10);

  mytime = millis();
}

void loop() {
  char incomingByte = Serial.read();
  if (incomingByte == '0'){imu_state = false;}
  if (incomingByte == '1'){imu_state = true;}
  if(imu_state) {
    // Poll sensors
    myIMU.readSensor();
    float baroElevation = icp.getElevation();
    uint32_t read = millis();
    
    uint32_t dt = read - last_read;
    last_read = read;

    xyzFloat gyr = myIMU.getGyrValues();
    xyzFloat accRaw = myIMU.getAccRawValues();
    xyzFloat corrAccRaw = myIMU.getCorrectedAccRawValues();
    xyzFloat gVal = myIMU.getGValues();
    float omega[3] = {gyr.x * to_rad, gyr.y * to_rad, gyr.z * to_rad};
    float accel[3] = {corrAccRaw.x, corrAccRaw.y, corrAccRaw.z};

    if(do_orient && gyr_state){
      float dt_s = dt / 1000.0;
      float dtheta[3] = {omega[0] * dt_s, omega[1] * dt_s, omega[2] * dt_s};
      orientation * Quaternion::from_rotvec(dtheta);
      float* euler = Quaternion::to_euler(orientation);

      Serial.print("Ori - Yaw: ");
      Serial.print(euler[0]);
      Serial.print(" Pitch: ");
      Serial.print(euler[1]);
      Serial.print(" Roll: ");
      Serial.println(euler[2]);
    }
    if(do_alt && gyr_state && accel_state) {
      altitude.estimate(accel, omega, baroElevation, read);
      Serial.print("Alt - Altitude: ");
      Serial.print(altitude.getAltitude());
      Serial.print(" Vertical Vel: ");
      Serial.print(altitude.getVerticalVelocity());
      Serial.print(" Vertical Accel: ");
      Serial.println(altitude.getVerticalAcceleration());
    }

  }
  delay(25);
}


// PPM signal decoder
void get_signal() {

  if (pulseIn(PPMin, HIGH) > 1500)  //2100 microseconds is just some time picked through tunning
  {
    for (int i = 1; i <= 8; i++)  //Read the pulses of the remainig channels
    {
      channel[i - 1] = pulseIn(PPMin, HIGH);
      // Serial.println(String("channel") + String(i-1) + ": " + channel[i-1]);
      
    }
    Serial.println("?");
  }
}
