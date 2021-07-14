// Include Arduino framework
#include <Arduino.h>
// Include Wire Library for I2C Communications with the gyro
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
float temp;

/************* Motor kram *************/
// stepsPerRevolution for Full Step = 200, 1.8 degrees for each Step
// Define pin connections
const int dirPinR = 2; //(SD_DATA0 External pulldown) If HIGH during boot, then no new content can be flashed
const int stepPinR = 4;
const int dirPinL = 15; //If LOW, the the esp will not show the log anymore at bootup (Serial.print)
const int stepPinL = 18;
const int enablePin = 19;
int R_motor_Dir = LOW;  //motor direction counterclockwise //Pin_2 must be LOW at start for boot to work!
int L_motor_Dir = HIGH; //motor direction clockwise
// Define max-Function
float minimal(float a, float b);

/************* Motor ISR kram *************/
/* create a hardware timer */
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile byte state = LOW;    //Steppermotoren bekommen wechselnd HIGH und LOW voltage
volatile int interval = 900;  //interval in microseconds, fast=20, slow=1000
volatile unsigned long previousMicros = 0;
volatile unsigned long currentMicros = 0;

/************* Voltage Measurement kram *************/
float voltage;
const int sensor_vp = 36;

/************* Server kram *************/
void server();



void IRAM_ATTR onTimer(){
  currentMicros = micros();
  if (currentMicros - previousMicros > interval) {
    previousMicros = currentMicros;

    state = !state;
    //digitalWrite(statusLED, state);
    
    digitalWrite(stepPinR, state);
    digitalWrite(stepPinL, state);
  }
}


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

  // Disable Stepper Driver at start
  digitalWrite(enablePin, HIGH);

  // Declare Voltage Measurement pin as Input, may not be necessary if microprocessor
  // has it set to 0 on startup and therefore would make it an Input automatically or sth.
  // Also, some pins are input-only. So only ouput needs to be defined!
  // pinMode(sensor_vp, INPUT);
  
  timeH = 0;
  temp = 1000; //beginne halt nur nicht bei 0 oder generell zu schnell fuer den Motor

  //calibrate deviation -> funny enough no Mean-Calculation necessary
  for(i=0; i<500; i++){
    //if(i % 15 == 0)digitalWrite(statusLED, !digitalRead(statusLED));
    imu(); //let it reach optimal operation temperature?
  }
  deviation = imu() * (-1);

  /* Use 1st timer of 4 */
  /* 1 tick take 1/(80MHZ/80) = 1us so we set divider 80 and count up */
  timer = timerBegin(0, 80, true);
  /* Attach onTimer function to our timer */
  timerAttachInterrupt(timer, &onTimer, true);
  /* Set alarm to call onTimer function every second 1 tick is 1us
  => 1 second is 1000000us */
  /* Repeat the alarm (third parameter) */
  timerAlarmWrite(timer, 20, true);
  /* Start an alarm */
  timerAlarmEnable(timer);

  // statusLED = OFF
  digitalWrite(statusLED, LOW);
}

void loop() {

/////////////////////////////I M U/////////////////////////////////////
  //unsigned long stopTime = micros();    //stopTime start
  error = imu() + deviation + desired_angle;
  //Serial.println(micros() - stopTime);  //stopTime end = 1500us

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

  if (error < 0) {
    temp = minimal(50.0, 500 + (10.6667*error));
  } else if(error > 0) {
    temp = minimal(50.0, 500 - (10.6667*error));
  } else {
    temp = 250000;
  }

  portENTER_CRITICAL(&timerMux);
  interval = (long)temp;
  portEXIT_CRITICAL(&timerMux);

  Serial.print(interval); Serial.print("\n");

/////////////////////////////MOTOR/////////////////////////////////////

  // if(-2 <error <2){ disable motor using enable_pin to reduce unnecessary jerking movements while standing }
  // if(error < -45 || error > 45){ disable motor using enable_pin because the robot is falling either way }
  //if ((1 > abs(error)) || (abs(error) > 40)) { //Problem, because enabling driver can also produce jerking movements, therefore setting temp = 250000 is better
  if (abs(error)) < 30)) {
    digitalWrite(enablePin, LOW);  // driver is active
    digitalWrite(statusLED, LOW);
  } else {
    digitalWrite(enablePin, HIGH); // driver is inactive
    digitalWrite(statusLED, HIGH);
  }
  
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

  //Serial.println(micros() - stopTime); // = 1590us without voltage and server

/////////////////////////////VOLTAGE/////////////////////////////////////
//  voltage = (float)analogRead(sensor_vp) / 4096 * 14.8 * 28695 / 28700;
//  Serial.print(voltage,1); Serial.println("v");
//  if (voltage < 12.8) {
//    digitalWrite(statusLED, HIGH); //Using inactive status led or other as warning signal
//  }

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

void server(){

}
