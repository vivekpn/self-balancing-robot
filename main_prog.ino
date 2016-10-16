#include <Wire.h>

// motor pins
#define LEFT_EN 5 
#define RIGHT_EN 6
#define FRONT 2
#define BACK 3

void setup()
{
  Serial.begin(9600);
  Wire.begin();
  
  // motor pins
  pinMode(LEFT_EN,OUTPUT);
  pinMode(RIGHT_EN,OUTPUT);
  pinMode(FRONT,OUTPUT);
  pinMode(BACK,OUTPUT);
  
}
void loop()
{
  float IMU_val[2],Roll,Pitch;
  int motor_pwm;
  IMU_Reading(IMU_val);
  //Roll=IMU_val[0];
  Pitch=IMU_val[1];
  //Serial.print(" IMU_Roll : ");Serial.print(Roll);
  Serial.print(" IMU_Pitch : "); Serial.println(Pitch);
  //  delay(40);
  motor_pwm = PID_fun(Pitch);
  Serial.print(" PWM : "); Serial.println(motor_pwm);
  
  if(motor_pwm > 0) // move front
  {
    digitalWrite(FRONT,HIGH);
    digitalWrite(BACK,LOW);
    analogWrite(LEFT_EN,motor_pwm);
    analogWrite(RIGHT_EN,motor_pwm);    
  }else             // move back
  {
    digitalWrite(FRONT,LOW);
    digitalWrite(BACK,HIGH);
    analogWrite(LEFT_EN,(-motor_pwm));
    analogWrite(RIGHT_EN,(-motor_pwm));    
    
  }
  
}
