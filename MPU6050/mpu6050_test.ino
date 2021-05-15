#include <Wire.h>

/*MPU-6050 gives you 16 bits data so you have to create some 16int constants
 * to store the data for accelerations and gyro*/

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050
int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;

float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];

float elapsedTime, timeH, timePrev;
int i;
float rad_to_deg = 180/3.141592654;
float deviation;

//functions
float imu();


void setup() {
  Serial.begin(115200);
  Wire.begin();    //Wire.begin(21, 22, 100000); // sda, scl, clock speed  //begin the wire comunication
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // set to zero (wakes up the MPU−6050)
  Wire.endTransmission(true);

  timeH = millis(); //Start counting time in milliseconds

  //calibrate deviation -> funny enough no Mean-Calculation necessary
  for(i=0; i<500; i++){
    imu(); //let it reach optimal operation temperature?
  }
  deviation = imu() * (-1);
}

void loop() {

/////////////////////////////I M U/////////////////////////////////////

  Serial.print(imu() + deviation); Serial.print(" , ");    // y-angle
  Serial.println(deviation);
  /*In Arduino IDE open Serial Monitor or Serial Plotter*/

}



float imu(){
   timePrev = timeH;  // the previous time is stored before the actual time read
   timeH = millis();  // actual time read
   elapsedTime = (timeH - timePrev) / 1000;
  
   Wire.beginTransmission(MPU_ADDR);
   Wire.write(0x3B); //Ask for/start with the 0x3B register- correspond to AcX (ACCEL_XOUT_H)
   Wire.endTransmission(false);
   Wire.requestFrom(MPU_ADDR,6,true); // request a total of 6 registers

   //each value needs two registers
   Acc_rawX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
   Acc_rawY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
   Acc_rawZ=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
   /*---X---*/
   Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
   /*---Y---*/
   Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
 
   Wire.beginTransmission(MPU_ADDR);
   Wire.write(0x43); //Gyro data first adress
   Wire.endTransmission(false);
   Wire.requestFrom(MPU_ADDR,4,true); //request just 4 registers

   //Once again we shift and sum
   Gyr_rawX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
   Gyr_rawY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
   //GyZ = Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L) //not being used in this test
   /*---X---*/
   Gyro_angle[0] = Gyr_rawX/131.0; 
   /*---Y---*/
   Gyro_angle[1] = Gyr_rawY/131.0;
   
   /*---X axis angle---*/
   Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
   /*---Y axis angle---*/
   Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];

   /*Now we have our angles in degree and values from -100º to 100º aprox*/
   return Total_angle[1];
}
