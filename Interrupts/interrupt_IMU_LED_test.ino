#include <Wire.h>

// Define LED pin
const int statusLED = 5;

/************* IMU kram *************/
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

// Define max-Function
float minimal(float a, float b);

/************* ISR kram *************/
/* create a hardware timer */
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile byte state = LOW;
volatile int interval = 900;
volatile unsigned long previousMicros = 0;
volatile unsigned long currentMicros = 0;


void IRAM_ATTR onTimer(){
  currentMicros = micros();
  if (currentMicros - previousMicros > interval) {
    previousMicros = currentMicros;

    state = !state;
    digitalWrite(statusLED, state);
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

  timeH = 0;
  
  //calibrate deviation -> funny enough no Mean-Calculation necessary
  for(i=0; i<500; i++){
    imu(); //let it reach optimal operation temperature?
  }
  deviation = imu() * (-1);

  // statusLED = OFF
  digitalWrite(statusLED, LOW);

  /************* ISR kram *************/
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
}

void loop() {

/////////////////////////////I M U/////////////////////////////////////
  error = imu() + deviation;

  Serial.print(error); Serial.print(" , ");    // x-angle
  Serial.print(deviation); Serial.print(" , ");
  /*In Arduino IDE open Serial Monitor or Serial Plotter*/

/////////////////////////////P I D/////////////////////////////////////

  float temp = minimal(800, 500000 - (246.519*(error*error)));

  portENTER_CRITICAL(&timerMux);
  interval = (long)temp;
  portEXIT_CRITICAL(&timerMux);

  Serial.println(interval);

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

