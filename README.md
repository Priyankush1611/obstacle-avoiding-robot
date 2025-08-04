# obstacle-avoiding-robot

An Obstacle Avoiding Car is a smart robotic vehicle that moves independently while detecting and avoiding obstacles in its path. It changes direction automatically to prevent collisions, making it a fun and educational project that demonstrates the basics of autonomous navigation and real-time decision making.

# Componments used

- Arduino Uno (or compatible) 
- 2 × DC motors + wheels (robot chassis) 
- Motor driver (e.g., L298N) 
- Ultrasonic distance sensor (HC-SR04) 
- Servo motor for sensor scanning 
- Chassis frame (e.g., 4WD platform) 
- Power supply (battery pack) 
- Jumper wires, screws, and supports

# Features

- Autonomous Navigation: Moves independently without human control.
- Real-Time Obstacle Detection: Detects obstacles and responds instantly.
- Automatic Steering: Changes direction based on obstacle location.
- Accurate Sensing: Scans forward-left and forward-right for better decision making.
- Tested Performance: Demonstrated smooth and reliable navigation.

# Report including circuit diagram

[Arduino_OA_Robot_Report.pdf](https://github.com/user-attachments/files/21579876/Arduino_OA_Robot_Report.pdf)

# Code

#include <Servo.h>
#include <NewPing.h>

#define SERVO_PIN 3
#define ULTRASONIC_SENSOR_TRIG 11
#define ULTRASONIC_SENSOR_ECHO 12
#define MAX_REGULAR_MOTOR_SPEED 75
#define MAX_MOTOR_ADJUST_SPEED 150
#define DISTANCE_TO_CHECK 30

//Right motor
int enableRightMotor=5;
int rightMotorPin1=7;
int rightMotorPin2=8;

//Left motor
int enableLeftMotor=6;
int leftMotorPin1=9;
int leftMotorPin2=10;

NewPing mySensor(ULTRASONIC_SENSOR_TRIG, ULTRASONIC_SENSOR_ECHO, 400);
Servo myServo;
void setup()
{
  // put your setup code here, to run once:
  pinMode(enableRightMotor,OUTPUT);
  pinMode(rightMotorPin1,OUTPUT);
  pinMode(rightMotorPin2,OUTPUT);
  
  pinMode(enableLeftMotor,OUTPUT);
  pinMode(leftMotorPin1,OUTPUT);
  pinMode(leftMotorPin2,OUTPUT);

  myServo.attach(SERVO_PIN);
  myServo.write(90);
  rotateMotor(0,0);   
}

void loop()
{

  int distance = mySensor.ping_cm();

  //If distance is within 30 cm then adjust motor direction as below
  if (distance > 0 && distance < DISTANCE_TO_CHECK)
  {
    //Stop motors
    rotateMotor(0, 0);
    delay(500);  
       
   //Reverse motors
    rotateMotor(-MAX_MOTOR_ADJUST_SPEED, -MAX_MOTOR_ADJUST_SPEED);        
    delay(200);
    
  //Stop motors
    rotateMotor(0, 0);
    delay(500);
    
   //Rotate servo to left    
    myServo.write(180);
    delay(500);

   //Read left side distance using ultrasonic sensor
    int distanceLeft = mySensor.ping_cm();    

   //Rotate servo to right
    myServo.write(0);    
    delay(500);    

  //Read right side distance using ultrasonic sensor   
    int distanceRight = mySensor.ping_cm();

  //Bring servo to center
    myServo.write(90); 
    delay(500);        
    
  if (distanceLeft == 0 )
    {
      rotateMotor(MAX_MOTOR_ADJUST_SPEED, -MAX_MOTOR_ADJUST_SPEED);
      delay(200);
    }
    else if (distanceRight == 0 )
    {
      rotateMotor(-MAX_MOTOR_ADJUST_SPEED, MAX_MOTOR_ADJUST_SPEED);
      delay(200);
    }
    else if (distanceLeft >= distanceRight)
    {
      rotateMotor(MAX_MOTOR_ADJUST_SPEED, -MAX_MOTOR_ADJUST_SPEED);
      delay(200);
    }
    else
    {
      rotateMotor(-MAX_MOTOR_ADJUST_SPEED, MAX_MOTOR_ADJUST_SPEED);
      delay(200);      
    }
    rotateMotor(0, 0);    
    delay(200);     
  }
  else
  {
    rotateMotor(MAX_REGULAR_MOTOR_SPEED, MAX_REGULAR_MOTOR_SPEED);
  }
}


void rotateMotor(int rightMotorSpeed, int leftMotorSpeed)
{
  if (rightMotorSpeed < 0)
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,HIGH);    
  }
  else if (rightMotorSpeed >= 0)
  {
    digitalWrite(rightMotorPin1,HIGH);
    digitalWrite(rightMotorPin2,LOW);      
  }

  if (leftMotorSpeed < 0)
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,HIGH);    
  }
  else if (leftMotorSpeed >= 0)
  {
    digitalWrite(leftMotorPin1,HIGH);
    digitalWrite(leftMotorPin2,LOW);      
  }

  analogWrite(enableRightMotor, abs(rightMotorSpeed));
  analogWrite(enableLeftMotor, abs(leftMotorSpeed));    
}

# How to use

1. Uploaad the code to Arduino UNO using ArduinoIDE
2. Connect the components as shown in the circuit diagram
3. Turn on the switch
