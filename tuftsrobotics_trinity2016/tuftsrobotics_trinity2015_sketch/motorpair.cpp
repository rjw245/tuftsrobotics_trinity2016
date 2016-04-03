#include "motorpair.h"

MotorPair::MotorPair(){
	motor1 = motor2 = NULL;
}

void MotorPair::attach(int dig1, int pwm1, int dig2, int pwm2){
	motor1 = new Motor();
	motor1->attach(dig1,pwm1);
	motor2 = new Motor();
	motor2->attach(dig2,pwm2);
}

void MotorPair::attach(Motor* mot1, Motor* mot2){
	motor1 = mot1;
	motor2 = mot2;
}

void MotorPair::flip(){
	flip1();
	flip2();
}

void MotorPair::flip1(){
	if(motor1!=NULL) motor1->flip();
}

void MotorPair::flip2(){
	if(motor2!=NULL) motor2->flip();
}

void MotorPair::swapMotors(){
  Motor* temp = motor1;
  motor1 = motor2;
  motor2 = temp;
}

void MotorPair::drive(int spd, int angle){
		if(motor1==NULL || motor2==NULL) return;
		if(angle>30)  angle= 30;
		if(angle<-30) angle=-30;
		int mot1_speed;
		int mot2_speed;
		if(angle<0){
			mot1_speed = (spd * (angle - MIN_ANGLE)) / (ANGLE_RANGE/2);
			mot2_speed = spd;
		}else{
			mot2_speed = (spd * (MAX_ANGLE - angle)) / (ANGLE_RANGE/2);
			mot1_speed = spd;
		}
		motor1->drive(mot1_speed);
		motor2->drive(mot2_speed);
}

void MotorPair::spin(int spd){
	if(motor1==NULL || motor2==NULL) return;
	motor1->drive(spd);
	motor2->drive(-spd);
}

void MotorPair::brake(){
	drive(0,0);
}
