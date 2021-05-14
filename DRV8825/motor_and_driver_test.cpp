
#include <Arduino.h>

// Define pin connections & motor's steps per revolution
const int dirPin = 2;
const int stepPin = 4;
const int stepsPerRevolution = 400;
// Define LED pin
int LED_PIN = 1;
// Define Functions
void blink_led(int LED, int delaytime);

void setup()
{
	// Declare pins as Outputs
	pinMode(stepPin, OUTPUT);
	pinMode(dirPin, OUTPUT);
	// Initialize LED pin as an output
	pinMode(LED_PIN, OUTPUT);

	for (int i = 0; i<6; i++){
		blink_led(LED_PIN, 500);
	}
}
void loop()
{
	// Set motor direction clockwise
	digitalWrite(dirPin, HIGH);

	// Spin motor slowly
	for(int x = 0; x < stepsPerRevolution; x++)
	{
		digitalWrite(stepPin, HIGH);
		delayMicroseconds(1000);
		digitalWrite(stepPin, LOW);
		delayMicroseconds(1000);
	}
	delay(1000); // Wait a second
	
	// Set motor direction counterclockwise
	digitalWrite(dirPin, LOW);

	// Spin motor quickly
	for(int x = 0; x < stepsPerRevolution; x++)
	{
		digitalWrite(stepPin, HIGH);
		delayMicroseconds(500);
		digitalWrite(stepPin, LOW);
		delayMicroseconds(500);
	}
	delay(1000); // Wait a second
}

void blink_led(int LED, int delaytime){
  // Set output HIGH for specified time
  digitalWrite(LED, HIGH);
  delay(delaytime);

  // Set output LOW for specified time
  digitalWrite(LED, LOW);
  delay(delaytime);
}
