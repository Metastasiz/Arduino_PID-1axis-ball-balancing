#include <Servo.h>
////////////////
Servo myservo;
float time;        
float errorDistance, prevErrorDistance;
int dt = 50;  //Refresh rate period of the loop is 50ms
////////////////variables for PID
float kp=8;
float ki=0.2;
float kd=3100;
float PID_p, PID_i, PID_d, PID_total;
float iSmoothMax = 3; //Distance on the rail that excludes I value of PID
float iSmoothMin = -4;
////////////////variables for sensor
float rowReading;
float sensorValue;
float A = 34.25;
float B = -5.93;
////////////////variables for servo
float midAngle = 135;	//Distance to both min and max angle should be the same
float minAngle = 100;
float maxAngle = 170;
////////////////variables for railing
float midDistance = 27.5;	//Should be the distance from sensor to the middle of the bar in mm
////////////////variables for railing normalisation
float minDistance = 10;
float maxDistance = 35;
float normaliseRailLength = 15; //Length of the preferred rail after normalisation 
boolean normaliseRail = false;
////////////////

void setup() {
  Serial.begin(9600);  
  myservo.attach(9);
  myservo.write(midAngle); //the balance in the middle
  time = millis();
}

void loop() {
  if (millis() > time+dt)
  {
    time = millis();    
    getSensorValue(10);
	////////////////Normalisation
	if (normaliseRail){
		sensorValue = bound(sensorValue, minDistance, maxDistance);
	}
    errorDistance = sensorValue - midDistance;
	////////////////Normalisation
	if (normaliseRail){
		if (errorDistance<0){
			errorDistance /= abs(midDistance-minDistance)*normaliseRailLength
		}
		if (errorDistance>0){
			errorDistance /= abs(midDistance-maxDistance)*normaliseRailLength
		}
	}
	////////////////PID
	calculatePID();
	PID_total = map(PID_total, -150, 150, minAngle, maxAngle);
	//
    myservo.write(PID_total);  
    prevErrorDistance = errorDistance;
  }
}
void calculatePID(){
	//
	PID_p = kp*errorDistance;
	//
	if(iSmoothMin < errorDistance && errorDistance < iSmoothMax) //excludes I from PID value
    {
      PID_i += ki*errorDistance*dt;
    }
    else
    {
      PID_i = 0;
    }
	//
	PID_d = kd*((errorDistance - prevErrorDistance)/dt);
	//
	PID_total = PID_p + PID_i + PID_d;
}
float getSensorValue(int avgTimes){
  rowReading = 0;
  for (int i =0; i < avgTimes; i++){
    rowReading += analogRead(A0);
    }
    rowReading /= avgTimes;
  sensorValue = A/((float)rowReading*5/1023)+B;
  return sensorValue;
}
float bound(float input, float upper, float lower){
if (input < lower)input = lower;
if (input > upper)input = upper;
return input;
}
