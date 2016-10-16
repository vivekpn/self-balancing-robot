#define Kp 1
#define Ki 0
#define Kd 0
#define INTEGRAL_CONSTRAIN 10

int PID_fun(float error)
{
  static float prev_error=0,sum_error=0;
  static int prev_time=0;
  int dt=0,time;
  float corr_anlge;
  int motor_pwm;
  time = millis();
  dt = time-prev_time;
  sum_error +=error;
  sum_error = constrain(sum_error,-INTEGRAL_CONSTRAIN,INTEGRAL_CONSTRAIN); // limit integral value to certain range so that it wont overshoot 
  corr_anlge = (Kp * error) +(Ki * sum_error * dt)+(Kd * (error -prev_error)/dt);
  prev_time = time;
  prev_error = error;
  Serial.print("corr_anlge :");
    Serial.println(corr_anlge);
  
  // convert corr_angle to a value that can be fed to motors
  if((corr_anlge > 0) && (corr_anlge < 30))
  {
  motor_pwm = map(corr_anlge,0,30,100,255);  // motor starts to move at 100
  return motor_pwm; // + ve sign indicates that it has move front
  }
  else if((corr_anlge < 0) && (corr_anlge > -30))
  {
    motor_pwm = map(-corr_anlge,0,30,100,255); // motor starts to move at 100  
    return (-motor_pwm); // - ve sign indicates that it has move back
  }else
  {
    Serial.println("OUT OF CONTROL !!!");
    return 0; // halt the robo
  }
}
