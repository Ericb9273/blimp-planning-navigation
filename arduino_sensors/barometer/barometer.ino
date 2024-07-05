#include <DFRobot_ICP10111.h>
DFRobot_ICP10111 icp;
void setup(void)
{
    Serial.begin(115200);
    while(icp.begin() != 0){
      Serial.println("Failed to initialize the sensor");
      }
     Serial.println("Success to initialize the sensor");
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
void loop(void)
{
  Serial.println("------------------------------");
  Serial.print("Read air pressure:");
  Serial.print(icp.getAirPressure());
  Serial.println("Pa");
  Serial.print("Read temperature:");
  Serial.print(icp.getTemperature());
  Serial.println("℃");
  Serial.print("Read altitude:");
  Serial.print(icp.getElevation());
  Serial.println("m");
  delay(1000);
}
