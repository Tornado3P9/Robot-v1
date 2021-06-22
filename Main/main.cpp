// Include Arduino framework
#include <Arduino.h>
// Include Wire Library for I2C Communications
#include <Wire.h>

// Define LED pin
const int statusLED = 5;

/************* IMU kram *************/
/*MPU-6050 gives you 16 bits data so you have to create some 16int constants
 * to store the data for accelerations and gyro*/
int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If ADO is HIGH -> 0x69

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
float desired_angle = 0; //This is the angle in which we want the
//balance to stay steady: 0 degrees for standing, maybe 20 degrees for driving

/************* Motor kram *************/
// stepsPerRevolution for Full Step = 200, 1.8 degrees for each Step
// Define pin connections
const int dirPinR = 2;
const int stepPinR = 4;
const int dirPinL = 15;
const int stepPinL = 18;
const int enablePin = 19;
int motorState = LOW;  //Steppermotoren bekommen wechselnd HIGH und LOW voltage
int R_motor_Dir = HIGH; //motor direction clockwise
int L_motor_Dir = LOW;  //motor direction counterclockwise
unsigned long previousMicros = 0;
long interval = 1000;  //interval in microseconds, fast=20, slow=1000
// Define max-Function
float minimal(float a, float b);

/************* Voltage Measurement kram *************/
float voltage;
const int sensor_vp = 36;

/************* Server kram *************/
void server();


void setup() {
  Serial.begin(115200);
  // Initialize LED pin as an output
  pinMode(statusLED, OUTPUT);
  // statusLED = ON
  digitalWrite(statusLED, HIGH);

  // MPU6050 communication begin
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // set to zero (wakes up the MPU−6050)
  Wire.endTransmission(true);

  // Declare motor pins as Outputs
  pinMode(stepPinR, OUTPUT);
  pinMode(dirPinR, OUTPUT);
  pinMode(stepPinL, OUTPUT);
  pinMode(dirPinL, OUTPUT);
  pinMode(enablePin, OUTPUT);

  // Declare Voltage Measurement pin as Input, may not be necessary if microprocessor
  // has it set to 0 on startup and therefore would make it an Input automatically or sth.
  // Also, some pins are input-only. So maybe only ouput needs to be defined?
  pinMode(sensor_vp, INPUT);
  
  timeH = 0;
/*
  for(i=0; i<500; i++){                                                   //Create 500 loops
    if(i % 15 == 0)digitalWrite(statusLED, !digitalRead(statusLED));      //Change the state of the LED every 15 loops to make the LED blink fast
    Wire.beginTransmission(MPU_ADDR);                                     //Start communication with the gyro
    Wire.write(0x43);                                                     //Start reading the Who_am_I register 75h
    Wire.endTransmission();                                               //End the transmission
    Wire.requestFrom(MPU_ADDR, 2);                                        //Request 2 bytes from the gyro
    deviation += Wire.read()<<8|Wire.read();                              //Combine the two bytes to make one integer
    delayMicroseconds(3700);                                              //Wait for 3700 microseconds to simulate the main program loop time
  }
  deviation /= 500;                                                       //Divide the total value by 500 to get the avarage gyro offset
  deviation *= -1;                                                        //Getting ready-to-use deviation error
*/
  //calibrate deviation -> funny enough no Mean-Calculation necessary
  for(i=0; i<500; i++){
    imu(); //let it reach optimal operation temperature?
  }
  deviation = imu() * (-1);

  // statusLED = OFF
  digitalWrite(statusLED, LOW);
}

void loop() {

/////////////////////////////I M U/////////////////////////////////////

  error = imu() + deviation - desired_angle;

  Serial.print(error); Serial.print(" , ");    // x-angle
  Serial.print(deviation); Serial.print(" , ");
  /*In Arduino IDE open Serial Monitor or Serial Plotter*/

/////////////////////////////P I D/////////////////////////////////////
/*
  error = imu() + deviation - desired_angle;

  pid_p = kp*error;

  if(-3 <error <3){
    pid_i = pid_i+(ki*error);  
  }

  pid_d = kd*((error - previous_error)/elapsedTime);

  PID = pid_p + pid_i + pid_d;

  //Remember to store the previous error.
  previous_error = error;
*/

  float temp = minimal(20.0, 1000 - (0.392*(error*error))); //der Wert soll nicht in den Minusbereich abdriften
  interval = (long)temp;
  Serial.print(interval); Serial.print(" , ");

/////////////////////////////MOTOR/////////////////////////////////////

  // if(-2 <error <2){ disable motor using enable_pin to reduce unnecessary jerking movements while standing }
  // if(error < -45 || error > 45){ disable motor using enable_pin because the robot is falling either way }
  if ((2 < abs(error)) || (abs(error) < 45)) {
    digitalWrite(enablePin, LOW);  // driver is active
  } else {
    digitalWrite(enablePin, HIGH); // driver is inactive
  }
  
  unsigned long currentMicros = micros();
  if (currentMicros - previousMicros >= interval) {
    previousMicros = currentMicros;

    // Set motor direction
    if(error < 0){
      R_motor_Dir = LOW;  //motor direction counterclockwise
      L_motor_Dir = HIGH; //motor direction clockwise
    } else {
      R_motor_Dir = HIGH; //motor direction clockwise
      L_motor_Dir = LOW;  //motor direction counterclockwise
    }
    digitalWrite(dirPinR, R_motor_Dir);
    digitalWrite(dirPinL, L_motor_Dir);

    // Spin motor
    if (motorState == LOW) {
        motorState = HIGH;
    } else {
        motorState = LOW;
    }
    digitalWrite(stepPinR, motorState);
    digitalWrite(stepPinL, motorState);
  }

/////////////////////////////VOLTAGE/////////////////////////////////////
  voltage = (float)analogRead(sensor_vp) / 4096 * 14.8 * 28695 / 28700;
  Serial.print(voltage,1); Serial.println("v");
  if (voltage < 12.8) {
    digitalWrite(statusLED, HIGH); //Using inactive status led as warning signal
  }

/////////////////////////////SERVER/////////////////////////////////////
  server();
}


float imu(){
   timePrev = timeH;  // the previous time is stored before the actual time read
   timeH = micros();  // actual time read
   elapsedTime = (timeH - timePrev) / 1000000;
  
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
   //Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
 
   Wire.beginTransmission(MPU_ADDR);
   Wire.write(0x43); //Gyro data first adress
   Wire.endTransmission(false);
   //Wire.requestFrom(MPU_ADDR,4,true); //request just 4 registers
   Wire.requestFrom(MPU_ADDR,2,true); //request just 2 registers when only using x-axis

   //Once again we shift and sum
   Gyr_rawX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
   //Gyr_rawY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
   /*---X---*/
   Gyro_angle[0] = Gyr_rawX/131.0; 
   /*---Y---*/
   //Gyro_angle[1] = Gyr_rawY/131.0;
   
   /*---X axis angle---*/
   Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
   /*---Y axis angle---*/
   //Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];

   /*Now we have our angles in degree and values from -100º to 100º aprox*/
   return Total_angle[0];
}

float minimal(float a, float b){
  if(a>=b){
    return a;
  } else {
    return b;
  }
}

void server(){;}
