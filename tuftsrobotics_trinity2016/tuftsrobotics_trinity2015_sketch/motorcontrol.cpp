#include "motorcontrol.h"

MotorControl::MotorControl(int cType){
	motor1 = motor2 = NULL;
        controlType = cType;
        r_last = 0;
        l_last = 0;
        lspeed_last = 0;
        rspeed_last = 0;
        I_r = 0;
        I_l = 0;
}

void MotorControl::attach(int dig1, int pwm1, int dig2, int pwm2){
	motor1 = new Motor();
	motor1->attach(dig1,pwm1);
	motor2 = new Motor();
	motor2->attach(dig2,pwm2);
}

void MotorControl::attach(Motor* mot1, Motor* mot2){
	motor1 = mot1;
	motor2 = mot2;
}

void MotorControl::flip(){
	flip1();
	flip2();
}

void MotorControl::flip1(){
	if(motor1!=NULL) motor1->flip();
}

void MotorControl::flip2(){
	if(motor2!=NULL) motor2->flip();
}

void MotorControl::swapMotors(){
  Motor* temp = motor1;
  motor1 = motor2;
  motor2 = temp;
}

void MotorControl::drive(double back_val, double front_val, double inertia){
  
  if(controlType == PROPORTIONAL){
    int leftSpeed  = KP * (back_val - OPT)  + inertia;
    int rightSpeed = KP * (front_val - OPT) + inertia;
    //Serial.println(rightSpeed);
    motor1->drive(leftSpeed);
    motor2->drive(rightSpeed);
    
  }
  else if(controlType == DERIVATIVE){
  /*
   //base errors
    int l_error = (back_val - OPT);
    int r_error = (front_val - OPT);
    
   
    //derivitive error
    int D_l = (l_error-l_last)*KD;
    int D_r = (r_error-r_last)*KD;
    //proportional error
    int P_l  = KP * l_error  + inertia;
    int P_r = KP * r_error + inertia;

    int leftSpeed = P_l + D_l;
    int rightSpeed = P_r + D_r;
    //Serial.println(rightSpeed);
    //drive motors
    motor1->drive(leftSpeed);
    motor2->drive(rightSpeed);
    //keep error values for next iter.
    l_last = l_error;
    r_last = r_error;
    */
      double dist_bw_sensors = 13.5; //cm
      double distFromWall = sqrt(pow(dist_bw_sensors*((back_val+front_val)/2.0),2)/(pow((front_val-back_val),2)+pow(dist_bw_sensors,2)));
      
      double ideal_front_val = back_val + dist_bw_sensors * (OPT - distFromWall)/100;
      
      
        Serial.print("Distance from wall: ");
        Serial.println(distFromWall);

        Serial.print("RF: "); Serial.println(front_val);
        Serial.print("RB: "); Serial.println(back_val);
      
      
      double error = (ideal_front_val - front_val); //Positive = angled toward the wall too much

      int rightSpeed = (KP * error) + inertia;
      int leftSpeed  = (-KP * error) + inertia;

      motor1->drive(leftSpeed);
      motor2->drive(rightSpeed);
    
  }
  else if(controlType == INTEGRAL){
    
     //base error
    //int l_error = (back_val - OPT);
    //int r_error = (front_val - OPT);
    
    int dist_from_opt = OPT - (back_val + front_val)/2; //dist_from_opt negative when too close
      int angle = front_val - back_val; //angle positive when pointed at wall
      int ideal_angle = 0.1 * dist_from_opt; //scale??
      
      int error = (ideal_angle - angle);
    //derivative error
    int D_l = (error-l_last)*KD;
    int D_r = (error-r_last)*KD;
    //porportional error
    int P_l  = KP * error  + inertia;
    int P_r = KP * error + inertia; 
    //integral accumulative error
    I_l += error;
    I_l = I_l*KI;
    I_r += error;
    I_r = I_r*KI;
    
    int leftSpeed = P_l + D_l + I_l;
    int rightSpeed = P_r + D_r + I_r;
    //Serial.println(rightSpeed);
    //drive motors
    motor1->drive(leftSpeed);
    motor2->drive(rightSpeed);

    l_last = error;
    r_last = error;
  }
}

void MotorControl::spin(int spd){
	if(motor1==NULL || motor2==NULL) return;
	motor1->drive(spd);
	motor2->drive(-spd);
}

void MotorControl::brake(){
	motor1->drive(0);
	motor2->drive(0);
}

int MotorControl::sensorToMm(int sensorval){
  return (41395.464 * pow(sensorval, -1.0947));
}
