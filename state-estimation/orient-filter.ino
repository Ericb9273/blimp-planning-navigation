//Orient-Filter

#include <Servo.h>
#include <Wire.h>
#include <ICM20948_WE.h>
#include "Quaternion.hpp

#define ICM20948_ADDR 0x68
#define channumber 8      //8 channels of pwm in total.
#define to_rad 3.14159265359 / 180.0
int channel[channumber];  //read Channel values
int PPMin = 13;           // PPM signal sending from rc receiver to the D13 pin

ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR);


// Initializing Servo objects:
Servo right_motor;
Servo left_motor;
Servo back_motor;

Servo right_servo;
Servo left_servo;
Servo capture;


unsigned long mytime = 0;

// The corresponding digital pins for sending signal to the servos.
int right_motor_output_pin = 5;
int right_servo_pin = 4;

int left_motor_output_pin = 2;
int left_servo_pin = 3;

int back_motor_output_pin = 7;

// imu state variables
bool imu_state = true;
Quaternion orientation;
unsigned long last_read = 0;

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
//  myIMU.setAccOffsets(-16330.0, 16450.0, -16600.0, 16180.0, -16640.0, 16560.0);

  Serial.println("Position your ICM20948 flat and don't move it - calibrating...");
  delay(1000);
  myIMU.autoOffsets();
  Serial.println("Done!");

  /*  The gyroscope data is not zero, even if you don't move the ICM20948. 
   *  To start at zero, you can apply offset values. These are the gyroscope raw values you obtain
   *  using the +/- 250 degrees/s range. 
   *  Use either autoOffsets or setGyrOffsets, not both.
   */
  //myIMU.setGyrOffsets(-115.0, 130.0, 105.0);

  /* enables or disables the gyroscope sensor, default: enabled */
   myIMU.enableGyr(true);
   myIMU.enableAcc(true);

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
  if(imu_state){
    myIMU.readSensor();
    unsigned long read = millis();
    unsigned long dt = read - last_read;
    float dt_s = dt / 1000.0;
    xyzFloat gyr = myIMU.getGyrValues();
    float omega[3] = {gyr.x * to_rad, gyr.y * to_rad, gyr.z * to_rad};
    float dtheta[3] = {omega[0] * dt_s, omega[1] * dt_s, omega[2] * dt_s};
    orientation * Quaternion::from_rotvec(dtheta);
    float euler[3] = Quaternion::to_euler(orientation);

    Serial.print("Orient- Yaw: ");
    Serial.print(euler[0]);
    Serial.print(" Pitch: ");
    Serial.print(euler[1]);
    Serial.print(" Roll: ");
    Serial.println(euler[2]);

    delay(25);
  }
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
