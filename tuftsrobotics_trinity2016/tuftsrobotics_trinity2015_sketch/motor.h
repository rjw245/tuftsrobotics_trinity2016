//MUST BE USED WITH WILL LANGFORD'S ROBOT SHIELD!!!!!
//THIS IS BECAUSE HE TAKES CARE OF SOME MOTOR
//CONTROL IN HARDWARE

#ifndef MOTOR_H
#define MOTOR_H
#include "Arduino.h"

class Motor{
    public:
        Motor();
        void attach(int dig, int pwm);
      	void flip();
      	void drive(int speed);
      	void brake();
	boolean isMovingForward();
        int getSpeed();

    private:
        int digPin;
        int pwmPin;
	boolean flipped;
        boolean isForward;
        int speed;
};
#endif MOTOR_H
