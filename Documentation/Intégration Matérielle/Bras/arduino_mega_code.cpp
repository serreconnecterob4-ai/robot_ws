#include "RoboClaw.h"


RoboClaw roboclaw(&Serial1, 10000);


const int potPin = A0;         
const int RELAY_PIN = 4;       


const int RELAY_ON = HIGH; 
const int RELAY_OFF = LOW; 


const int POS_MIN = 40;         
const int POS_MAX = 1000;       


int targetPosition = POS_MIN;   
float currentSpeed = 0;         
bool isRoboclawOn = false;     


float maxSpeed = 60.0;         
float minSpeed = 20.0;          
float Kp = 1.2;                 
float accel = 2.0;             


int stopTolerance = 10;         
int wakeUpTolerance = 20;       
unsigned long timeAtTarget = 0; 
const unsigned long SLEEP_DELAY = 2000;


void allumerRoboclaw() {
 if (!isRoboclawOn) {
   digitalWrite(RELAY_PIN, RELAY_ON);
   delay(500);
   isRoboclawOn = true;
 }
}


void eteindreRoboclaw() {
 if (isRoboclawOn) {
   digitalWrite(RELAY_PIN, RELAY_OFF);
   isRoboclawOn = false;
 }
}


void setup() {
 Serial.begin(115200);           
 Serial.setTimeout(20);        
 pinMode(RELAY_PIN, OUTPUT);
 delay(1000); // Chauffe
 analogRead(potPin); delay(10);
  targetPosition = constrain(analogRead(potPin), POS_MIN, POS_MAX);
 allumerRoboclaw();
 roboclaw.begin(38400);       
}


void loop() {
 // 1. LECTURE DE LA COMMANDE BLINDÉE (Format exigé : "P500,V60")
 if (Serial.available() > 0) {
   String input = Serial.readStringUntil('\n');
   input.trim();
 
   // Si le message commence bien par 'P' et contient ',V'
   if (input.startsWith("P") && input.indexOf(",V") != -1) {
     int vIndex = input.indexOf(",V");
    
     // Extraction des valeurs après le P et après le V
     int newPos = input.substring(1, vIndex).toInt();
     int newSpeed = input.substring(vIndex + 2).toInt();
    
     targetPosition = constrain(newPos, POS_MIN, POS_MAX);
     maxSpeed = constrain(newSpeed, 15, 127);
   }
 }


 int currentPosition = analogRead(potPin);
 int error = targetPosition - currentPosition;


 // 2. REVEIL
 if (!isRoboclawOn && abs(error) > wakeUpTolerance) {
     allumerRoboclaw();
 }


 // 3. CALCUL DE LA VITESSE AVEC BOOST
 float desiredSpeed = error * Kp;
 if (desiredSpeed > maxSpeed) desiredSpeed = maxSpeed;
 if (desiredSpeed < -maxSpeed) desiredSpeed = -maxSpeed;


 if (abs(error) > stopTolerance) {
   if (desiredSpeed > 0 && desiredSpeed < minSpeed) desiredSpeed = minSpeed;
   if (desiredSpeed < 0 && desiredSpeed > -minSpeed) desiredSpeed = -minSpeed;
 } else {
   desiredSpeed = 0;
 }


 // 4. ACCÉLÉRATION
 if (currentSpeed < desiredSpeed) {
   currentSpeed += accel;
   if (currentSpeed > desiredSpeed) currentSpeed = desiredSpeed;
 } else if (currentSpeed > desiredSpeed) {
   currentSpeed -= accel;
   if (currentSpeed < desiredSpeed) currentSpeed = desiredSpeed;
 }


 // 5. ENVOI DES ORDRES
 if (isRoboclawOn) {
   if (abs(currentSpeed) > 1) {
     if (currentSpeed > 0) roboclaw.ForwardM1(0x80, (uint8_t)currentSpeed);
     else roboclaw.BackwardM1(0x80, (uint8_t)abs(currentSpeed));
   } else {
     roboclaw.ForwardM1(0x80, 0);
   }
 }


 // 6. EXTINCTION RAPIDE
 if (abs(currentSpeed) < 0.1 && abs(error) <= stopTolerance) {
   if (timeAtTarget == 0) timeAtTarget = millis();
   else if (millis() - timeAtTarget > SLEEP_DELAY) eteindreRoboclaw();
 } else {
   timeAtTarget = 0;
 }


 delay(20);
}

