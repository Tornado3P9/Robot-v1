// Half-Step-Mode
const int ledPin = 5;
long interval = 1000;
long interval2 = 40;
unsigned long previousMillis = 0;
long askel = 1;

int a = 1;

const int dirPin = 2;
const int stepPin = 4;

// Define Functions
void blink_led(int LED, int delaytime);

void setup(){
  // Declare pins as Outputs
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(ledPin, OUTPUT);

  digitalWrite(dirPin, HIGH);

  for (int i = 0; i<6; i++){
    blink_led(ledPin, 500);
  }
}

void loop(){
  
  unsigned long currentMillis = millis();
  
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(interval);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(interval);

  if(currentMillis - previousMillis > interval2) {
    previousMillis = currentMillis;
    if (interval > 100) {
      if(a == 1){
        interval = interval - askel;
      }
      if(a == 2 && interval < 2000){
        interval = interval + askel;
      }
    } else {
      a = 2;
      interval = 110;
    }
  }
}

void blink_led(int LED, int delaytime){
  // Set output HIGH for specified time
  digitalWrite(LED, HIGH);
  delay(delaytime);

  // Set output LOW for specified time
  digitalWrite(LED, LOW);
  delay(delaytime);
}
