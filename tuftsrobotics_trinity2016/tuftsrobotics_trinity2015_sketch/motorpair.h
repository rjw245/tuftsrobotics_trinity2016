#ifndef DUALMOTOR_H
#define DUALMOTOR_H
#include "Arduino.h"
#include "motor.h"

#define MAX_ANGLE  30
#define MIN_ANGLE -30
#define ANGLE_RANGE (MAX_ANGLE - MIN_ANGLE)

class MotorPair{
	public:
		MotorPair();
		void attach(int dig1, int pwm1, int dig2, int pwm2);
		void attach(Motor* mot1, Motor* mot2);
		void flip();
		void flip1();
		void flip2();

		void swapMotors();

		//Give a speed b/w -255 and 255
		//and an angle between 30 and -30
		//to specify how sharply you want the robot to turn.
		void drive(int spd, int angle);
		
		//Give a speed at which to turn in place b/w -255 and 255
		void spin(int spd);
		void brake();
	private:
		Motor* motor1;
		Motor* motor2;
};
	

#endif DUALMOTOR_H
