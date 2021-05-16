#include <Wire.h>

/************* IMU kram *************/
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
// Define Functions
float imu();

/************* PID kram *************/
float PID, pwmLeft, pwmRight, error, previous_error;
float pid_p=0;
float pid_i=0;
float pid_d=0;
/////////////////PID CONSTANTS/////////////////
double kp=3.55;//3.55
double ki=0.005;//0.003
double kd=2.05;//2.05
///////////////////////////////////////////////
float desired_angle = 0; //This is the angle in which we whant the
                         //balance to stay steady

/************* Motor kram *************/
// Define pin connections & motor's steps per revolution
const int dirPin = 2;
const int stepPin = 4;
int SPR = 400; //stepsPerRevolution
// Define LED pin
const int LED_SETUP = 5;


void setup() {
  Serial.begin(115200);
  // Initialize LED pin as an output
  pinMode(LED_SETUP, OUTPUT);
  // LED_SETUP = ON
  digitalWrite(LED_SETUP, HIGH);
  // MPU6050 communication begin
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // set to zero (wakes up the MPU−6050)
  Wire.endTransmission(true);

  // Declare motor pins as Outputs
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  timeH = millis(); //Start counting time in milliseconds

  //calibrate deviation -> funny enough no Mean-Calculation necessary
  for(i=0; i<500; i++){
    imu(); //let it reach optimal operation temperature?
  }
  deviation = imu() * (-1);

  // LED_SETUP = OFF
  digitalWrite(LED_SETUP, LOW);
}

void loop() {

/////////////////////////////I M U/////////////////////////////////////   Muss wohl per interrupt geschehen? Oder nachher mit FreeRTOS zwei unabhaengige Prozesse

  Serial.print(imu() + deviation); Serial.print(" , ");    // y-angle
  Serial.println(deviation);
  /*In Arduino IDE open Serial Monitor or Serial Plotter*/

/////////////////////////////P I D/////////////////////////////////////

  error = Total_angle[1] - desired_angle;

  pid_p = kp*error;

  if(-3 <error <3){
    pid_i = pid_i+(ki*error);  
  }

  pid_d = kd*((error - previous_error)/elapsedTime);

  PID = pid_p + pid_i + pid_d;

  /*Finnaly we calculate the desired throttle
  Must be inverse to what we have done so far....not finished version!*/
  pwmLeft = SPR + PID;
  pwmRight = SPR + PID;

  //Remember to store the previous error.
  previous_error = error;

/////////////////////////////MOTOR/////////////////////////////////////
  // Set motor direction clockwise
  digitalWrite(dirPin, HIGH);

  // Spin motor
  digitalWrite(stepPin, HIGH);
  //delayMicroseconds(1000); == delay(1);
  //delay(1000); // Wait a second

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
