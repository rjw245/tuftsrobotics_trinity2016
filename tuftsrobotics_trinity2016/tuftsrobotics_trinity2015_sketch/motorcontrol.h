#ifndef DUALMOTOR_H
#define DUALMOTOR_H
#include "Arduino.h"
#include "motor.h"

//Control types
#define PROPORTIONAL 0
#define DERIVATIVE   1
#define INTEGRAL     2

#define KP           8
#define KD           0.8
#define KDspd        0.1
#define KI           0.3

#define OPT          12 //cm

class MotorControl{
	public:
		MotorControl(int cType);
		void attach(int dig1, int pwm1, int dig2, int pwm2);
		void attach(Motor* mot1, Motor* mot2);
		void flip();
		void flip1();
		void flip2();

		void swapMotors();

		//Give the proportions for left and right motor
                //plus an inertial constant
		void drive(double back_val, double front_val, double inertia);
		
		//Give a speed at which to turn in place b/w -255 and 255
		void spin(int spd);
		void brake();
                int sensorToMm(int sensorval);
	private:
		Motor* motor1;
		Motor* motor2;
                int controlType;
                int l_last;
                int r_last;
                int lspeed_last;
                int rspeed_last;
                float I_l;
                float I_r;
};
	#endif DUALMOTOR_H
