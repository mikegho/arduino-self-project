
/*
 *  These are the pins defined which are based on the elegoo car
 *  kit. You will need their smart robot car kit 3.0.
 */

/*
 * Libraries and header files
 */
#include <Servo.h>                        // Servo library

/*
 * Local definitions
 */
#define ENA 5                             // L298N Channel A (Pin 5), controls left motors
#define ENB 6                             // L298N Channel B (Pin 6), controls right motors
#define IN1 7                             // Controls wheels on left motor
#define IN2 8                             // Controls wheels on left motor
#define IN3 9                             // Controls wheels on right motor
#define IN4 11                            // Controls wheels on right motor
#define spd 250                           // 250 - Max number for speed, must be at least 150
int echoPin = A4;                         // Pin for servo to echo, A4 is on the expansion board
int trigPin = A5;                         // Pin for servo to trigger, A5 is on the expansion board
int currDist;                             // Variable for current distance
int tmpDist1, tmpDist2;                   // Temporary variables for storing previous distance
int leftDeg = 180;                        // Variable for 180 degrees
int leftDiagdeg = 135;                    // Variable for 135 degrees
int frontDeg = 90;                        // Variable for 90 degrees
int rightDiagDeg = 45;                    // Variable for 45 degrees
int rightDist = 0;                        // Variable for 0 degrees
int timeDelay = 1000;                     // 1000 microseconds = 1 second
int minDist = 45;                         // Minimum distance before object collision (cm)
int turnDelay = 360;                      // Time delay (microseconds) for turning left or right
Servo servo;                              // Object class for servo

/*
 * This moves the car forward.
 */
void forward(){
  analogWrite(ENA, spd);
  analogWrite(ENB, spd);
  /* Left motors move forward */
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  /* Right motors move forward */
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}
/*
 * This moves the car backward
 */
void back(){
  analogWrite(ENA, spd);
  analogWrite(ENB, spd);
  /* Left motors move backward */
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  /* Right motors move backward */
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

/*
 * This turns the car left
 */
void left(){
  analogWrite(ENA, spd);
  analogWrite(ENB, spd);
  /* Left motors move backward */
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  /* Right motors move forward */
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

/*
 * This turns the car right
 */
void right(){
  analogWrite(ENA, spd);
  analogWrite(ENB, spd);
  /* Left motors move forward */
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  /* Right motors move backward */
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW); 
}

//Ultrasonic distance measurement Sub function
int distance() {
  digitalWrite(trigPin, LOW);   
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);  
  delayMicroseconds(20);
  digitalWrite(trigPin, LOW);   
  float Fdistance = pulseIn(echoPin, HIGH);  
  Fdistance= Fdistance*0.034/2;       
  return (int)Fdistance;
}  

/*
 * This stops both motors from moving
 */
void stop(){
   digitalWrite(ENA, LOW);
   digitalWrite(ENB, LOW);
} 

/*
 * Set up the pins for Arduino, expansion board, and motors
 */
void setup(){
  /*
   * The server has 3 parameters for this attach method: pin, pulse width (microseconds) corresponding to
   * minimum (0 degrees) angle, and pulse width (microseconds) corresponding to maximum (180 degrees) angle
   */
  servo.attach(3,600,2400);
  Serial.begin(9600);     
  pinMode(echoPin, INPUT);    
  pinMode(trigPin, OUTPUT);  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

}

/*
 * Infinite loop for Arduino to call
 */
void loop() {

  // Start the server at 90 degrees or straight forward
  servo.write(frontDeg);

  // Calculate distance in front (cm)
  currDist = distance();

  // If distance is less than 45 centimeters, look elsewhere to move
  if (currDistance < minDist){
    stop();

    // Check distance for far left
    servo.write(leftDeg);
    delay(timeDelay);
    tmpDist1 = distance();
    delay(timeDelay/2);

    // Check distance for far right
    wrote.write(rightDeg);
    delay(timeDelay);
    tmpDist = distance();
    delay(timeDelay/2);

    // Check if left distance is bigger than right
    if (tmpDist1 > tmpDist2 && tmpDist1 > minDist){
      back();
      delay(turnDelay/2);
      left();
      delay(turnDelay);
    }
    // Check if right distance is bigger than left
    else if (tmpDist1 < tmpDist2 && tmpDist2 > minDist){
      back();
      delay(turnDelay/2);
      right();
      delay(turnDelay);
    }
    // If neither are greater distance, move backwards and turn left
    else{
      back();
      delay(turnDelay/2);
      left();
      delay(turnDelay);
    }
  }
  // Else keep moving forward
  else{
    forward();
  }
  
}
