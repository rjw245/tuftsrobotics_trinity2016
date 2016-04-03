#include <NewPing.h>

#include <Servo.h>

#include "motor.h"
#include "motorcontrol.h"
#include "fireSensorArray.h"

//DEFINE ALL PINS HERE
  #define lineSensePin          A15
  #define DIST_AHEAD_ECHO       30
  #define DIST_AHEAD_TRIG       28
  #define DIST_RF_ECHO          26
  #define DIST_RF_TRIG          24
  #define DIST_RB_ECHO          22
  #define DIST_RB_TRIG          23

  #define MAX_DISTANCE          200 //cm, width of hallway

  //Left to right, from robot's POV
  #define fireSensePin5         A8
  #define fireSensePin4         A10
  #define fireSensePin3         A9
  #define fireSensePin2         A11
  #define fireSensePin1         A12

  #define servoPin              2
  #define fireIndicator         38
  #define soundIndicator        34
  #define FFTSIGNAL             11

  //Motor pins
  #define leftMotordig           44
  #define leftMotorpwm           3
  #define rightMotordig          46
  #define rightMotorpwm          4
  

//IS SERIAL COMM NEEDED?
//THIS FUCKS UP TIMING SOM'M BAD
#define DEBUG                 1

#define WAITFOR3800HZ         1

//Possible States
#define STARTPUSHED           8
#define INITIALIZATION        0
#define WALLFOLLOW            1
#define INROOM                2
#define FOUNDFIRE             3
#define RETURNHOME            4
#define PUTOUTFIRE            6
#define ALIGNFIRE             7
#define HOMEREACHED           9
#define DRIVETOFIRE           10


#define INITIALIZATION_TIME   2250

//Wall-follow constants
#define FRONTOBSTACLEDIST     190
#define WALLONRIGHT           130

#define WALLFOLLOW_INERTIA    150

//Line sense & alignment constants
#define LINESENSING_INVERTED  0     //1 = look for black, 0 = look for white

#if LINESENSING_INVERTED
  #define LINESENSED            650
#else
  #define LINESENSED            45
#endif


//Fire sensing constants
#define FIRESENSED            60
#define FIRECLOSE             280
#define FIREANGLETHRESH       4
#define FIRESWEEPTIME         1900
#define FIRESENSE_TRIALS      10
#define SWEEPSPEED            120

//Motor controller object
//(motorcontrol.h)
MotorControl mcontrol(DERIVATIVE);

//Fire sensor object
//(fireSensorArray.h)
FireSensorArray fireSense;

//EXTINGUISHER SERVO CONSTANTS
#define SERVO_LOOSE           40
#define SERVO_TAUT            140


//Motor objects
Motor leftMotor;
Motor rightMotor;

//Servo for canister
Servo pullServo;
NewPing distAhead(DIST_AHEAD_TRIG, DIST_AHEAD_ECHO, MAX_DISTANCE);
NewPing distRF(DIST_RF_TRIG, DIST_RF_ECHO, MAX_DISTANCE);
NewPing distRB(DIST_RB_TRIG, DIST_RB_ECHO, MAX_DISTANCE);

//Start state is STARTPUSHED
int STATE = STARTPUSHED;


void setup() {
  //Set up motors with proper pins
  leftMotor.attach(leftMotordig,leftMotorpwm);
  rightMotor.attach(rightMotordig,rightMotorpwm);
  
  pullServo.attach(servoPin);
  pullServo.write(SERVO_LOOSE);
  
  //Give these motors to the motor controller for wall following
  mcontrol.attach(&leftMotor,&rightMotor);
  
  //Initialize array of fire sense pins
  //and pass it to the fire sensor array object
  int fireSensePins[5] = {fireSensePin1,fireSensePin2,fireSensePin3,fireSensePin4,fireSensePin5};
  fireSense.attach(fireSensePins);

  rightMotor.flip();
  leftMotor.flip();
  
  
  #if DEBUG
    Serial.begin(115200);
  #endif
  
  pinMode(lineSensePin,INPUT);
  pinMode(FFTSIGNAL,INPUT);
  pinMode(fireIndicator,OUTPUT); //Goes high when fire detected
  digitalWrite(fireIndicator,LOW);
  
  pinMode(13,OUTPUT);
  
  //Wait for serial to begin
  #if DEBUG
    while(!Serial);
    Serial.println("Comm ready");
  #endif
  

  #if DEBUG
    Serial.println("Started");
  #endif
}

//These declarations are for line adjustment

/*
boolean linesensed = true;
int linesenseTime = 0;
int lineVal;
*/
//END DECLARATIONS FOR LINE ADJUSTMENT

//These declarations are for fire sensing
int fire_motinertia = 110;//140;
int fire_rot_inertia = 0;
int fire_motcorrection = 6;
int fAngle = 0;
int fStrength = 0;
int fMiddleStrength = 0;
int maxFireStrength = 0;
int curFireStrength = 0;
boolean maxFireFound = false;
long fireStrengthSum = 0;
long fireStrengthAvg = 0;
bool fireSensed = false;
//END DECLARATIONS FOR FIRE SENSING

boolean rotatedAtStart = false;
int diagTimeCount=0;

int numRoomsChecked = 0;

//KEEP TRACK OF RUNTIME
unsigned long time = 0;
unsigned long startTime = millis();
unsigned long stateStart = 0;
unsigned long sweepStartTime = 0;

void loop(){
  pullServo.write(SERVO_LOOSE);
  while(true){
    //statemachine();
    //sensorDiagnostics(); delay(1000);
    //testMotors();
    //testRotation();
    testWallFollow();
    //testFireSensing();
    //testLineSensing();
    //testFireFollow();
    //testServo();
    //testSoundDetect();
    //testFFTComm();
  }
}

void testServo(){
  int pos;
  for(pos = SERVO_LOOSE; pos <= SERVO_TAUT; pos += 1) // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    pullServo.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(8);                       // waits 15ms for the servo to reach the position
    #if DEBUG
      Serial.println(pos);
    #endif
  }
  delay(1000);
  
  for(pos = SERVO_TAUT; pos>=SERVO_LOOSE; pos -= 1)     // goes from 180 degrees to 0 degrees 
  {                                
    pullServo.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(8);                       // waits 15ms for the servo to reach the position 
    #if DEBUG
      Serial.println(pos);
    #endif
  }
  delay(1000);
}


void testFFTComm(){
  if(digitalRead(FFTSIGNAL)){
    while(true){
      testWallFollow();
    }
  }
}

void testFireSensing(){
  #if DEBUG
    Serial.println(fireSense.fireAngle());
    delay(500);
  #endif
}

void testFireFollow(){
  fAngle = fireSense.fireAngle();
  fStrength = fireSense.fireStrength();
  
  #if DEBUG
    Serial.println(fAngle);
  #endif
  
  
  if(abs(fAngle) > FIREANGLETHRESH)
  {
    leftMotor.drive(fire_motinertia + fire_motcorrection*fAngle);
    rightMotor.drive(fire_motinertia - fire_motcorrection*fAngle);
  }

  if(fStrength>=FIRECLOSE){
    leftMotor.brake();
    rightMotor.brake();
    STATE = PUTOUTFIRE;
    stateStart = time;
  }
}


void testWallFollow(){
  int rightfront = distRF.ping() / US_ROUNDTRIP_CM;
  delay(10);
  int rightback = distRB.ping() / US_ROUNDTRIP_CM;
  delay(10);
  int front = distAhead.ping() / US_ROUNDTRIP_CM;
  delay(10);
  mcontrol.drive(rightback,rightfront,WALLFOLLOW_INERTIA);
  
  if(distAhead.ping() / US_ROUNDTRIP_CM > FRONTOBSTACLEDIST){
    rotCCWUntilParallel();
  }
}

void testRotation(){
  rotCW90();
  delay(1000);
  rotCCW90();
  delay(2000);
}

void testLineSensing(){
  #if DEBUG
    Serial.println(analogRead(lineSensePin));
    delay(500);
  #endif
}

void statemachine() {
  //while(1) Serial.println("Test2");
  switch(STATE){
    case STARTPUSHED:
      //Wait for start sound, only if WAITFOR3800HZ not set
      if( !WAITFOR3800HZ || digitalRead(FFTSIGNAL)){ //If sound heard or not waiting for sound or button pressed again
        STATE = INITIALIZATION;
        time = 0;
        startTime = millis();
        stateStart = 0;
      }
      break;
    case INITIALIZATION:
      //Rotate 90 deg CW at start for arbitrary start orientation
      if(!rotatedAtStart){
        rotCW90();
        rotatedAtStart = true;
      }
      
      //Is there something in front of me? If so, rotate CCW
      if(distAhead.ping() / US_ROUNDTRIP_CM > FRONTOBSTACLEDIST){
        
        //Avoid potential dog
        leftMotor.drive(-160);
        rightMotor.drive(-160);
        delay(200);
        leftMotor.brake();
        rightMotor.brake();
        
        rotCCWUntilParallel();
      }
      //Wall follow
      mcontrol.drive(distRB.ping() / US_ROUNDTRIP_CM,distRF.ping() / US_ROUNDTRIP_CM,255);
      
      //After some seconds, initialization is over, so go to next state
      if(time>=INITIALIZATION_TIME){
        STATE = WALLFOLLOW;
        stateStart = time;
        #if DEBUG
          Serial.println("Changed state to WALLFOLLOW");
        #endif
      }
      
      break;
      
    case WALLFOLLOW:
      //Serial.println("WALL FOLLOWING");
      //Look for front obstacles:
      //Is there something in front of me? If so, rotate 90 CCW
      if(distAhead.ping() / US_ROUNDTRIP_CM>FRONTOBSTACLEDIST){
        rotCCWUntilParallel();
      }
      mcontrol.drive(distRB.ping() / US_ROUNDTRIP_CM,distRF.ping() / US_ROUNDTRIP_CM,255);
      
      
      //Look for lines. If found, brake
       #if LINESENSING_INVERTED
         if(analogRead(lineSensePin)>LINESENSED){
       #else
         if(analogRead(lineSensePin)<LINESENSED){
       #endif
           rightMotor.brake();
           leftMotor.brake();
           numRoomsChecked++;
           STATE = INROOM;
           leftMotor.drive(180);
           rightMotor.drive(180);
           delay(450);
           rightMotor.brake();
           leftMotor.brake();
           stateStart = time;
           #if DEBUG
             Serial.println("Changed state to INROOM");
           #endif
         }
       break;
      
    case INROOM:
      fireSensed = false;
      
      sweepStartTime = millis();
      while (millis() - sweepStartTime < FIRESWEEPTIME){
        fireStrengthSum = 0;
        //Switch direction halfway
        if (millis() - sweepStartTime > FIRESWEEPTIME/3){
          leftMotor.drive(-SWEEPSPEED);
          rightMotor.drive(SWEEPSPEED);
        }
        else{
          leftMotor.drive(SWEEPSPEED);
          rightMotor.drive(-SWEEPSPEED);
        }
        
        for(int i = 0; i < FIRESENSE_TRIALS; i++){
          fireStrengthSum += fireSense.fireStrength();
        }
        
        fireStrengthAvg = fireStrengthSum / (FIRESENSE_TRIALS);
        #if DEBUG
          Serial.println(fireStrengthAvg);
        #endif
        
        if (fireStrengthAvg > FIRESENSED){
          STATE = FOUNDFIRE;
          digitalWrite(fireIndicator,HIGH);
          int lspeed = leftMotor.getSpeed();
          int rspeed = rightMotor.getSpeed();
          leftMotor.brake();
          rightMotor.brake();
          delay(300);
          fireSensed = true;
          break;
        }
      }
      #if DEBUG
        Serial.println(fireStrengthAvg);
        delay(400);
      #endif
      
      if(!fireSensed)
      {
        //If we get here, didn't find fire
        leaveRoom();
        STATE = WALLFOLLOW;
      }
      
      break;
      
    case FOUNDFIRE:
      //ROTATE UNTIL FIRE IS DIRECTLY AHEAD
      //DRIVE FORWARD UNTIL FIRE READS SIGNIFICANTLY HIGH
      //TRIGGER CO2
      //STATE = RETURNHOME;
      fAngle = fireSense.fireAngle();
      fMiddleStrength = avgSensorVal(fireSensePin3,FIRESENSE_TRIALS);
      
      
      //if(abs(fAngle)>=7){
        fire_rot_inertia = fire_motcorrection*fAngle;
        if (fire_rot_inertia<100){
          fire_rot_inertia = 100;
        }
        if (fire_rot_inertia>160){
          fire_rot_inertia = 160;
        }
        if(fAngle > FIREANGLETHRESH)
        {
          leftMotor.drive(fire_rot_inertia);
          rightMotor.drive(-fire_rot_inertia);
        }
        else if(fAngle < -FIREANGLETHRESH){
          leftMotor.drive(-fire_rot_inertia);
          rightMotor.drive(fire_rot_inertia);
        }
        else{
          leftMotor.drive(fire_motinertia);
          rightMotor.drive(fire_motinertia);
        }
        
        if(fAngle < FIREANGLETHRESH && fAngle > -FIREANGLETHRESH){
          leftMotor.brake();
          rightMotor.brake();
          STATE = DRIVETOFIRE;
          stateStart = time;
        }
      //}
      
      break;
      
    case DRIVETOFIRE:
    
      fAngle = fireSense.fireAngle();
      fMiddleStrength = avgSensorVal(fireSensePin3,FIRESENSE_TRIALS);
      
      
      
      //if(abs(fAngle)>=7){
        if(fAngle > FIREANGLETHRESH)
        {
          leftMotor.drive(fire_motinertia + fire_motcorrection*fAngle);
          rightMotor.drive(fire_motinertia - fire_motcorrection*fAngle);
        }
        else if(fAngle < -FIREANGLETHRESH){
          leftMotor.drive(fire_motinertia - fire_motcorrection*fAngle);
          rightMotor.drive(fire_motinertia + fire_motcorrection*fAngle);
        }
        else{
          leftMotor.drive(fire_motinertia);
          rightMotor.drive(fire_motinertia);
        }
        
        if(fMiddleStrength>=FIRECLOSE){
          leftMotor.brake();
          rightMotor.brake();
          STATE = PUTOUTFIRE;
          stateStart = time;
        }
        
        //Timeout, so if we get stuck on a wall, we try to extinguish anyway
        if(time - stateStart > 3000){
          STATE = PUTOUTFIRE;
        }
        
    
    break;
      
      
    case PUTOUTFIRE:
      extinguish();
      delay(200);
      fireStrengthSum = 0;
      for(int i=0; i<FIRESENSE_TRIALS; i++){
        fireStrengthSum += fireSense.fireStrength();
        delay(1);
      }
      fireStrengthAvg = fireStrengthSum / FIRESENSE_TRIALS;
      if(fireStrengthAvg<FIRESENSED){
        STATE = RETURNHOME;
      }
      delay(500);
      break;
      
    case RETURNHOME:
      returnHome(numRoomsChecked);
      STATE = HOMEREACHED;
      
    break;
    
    case HOMEREACHED:
    
    break;
  }
  diagTimeCount++;
  if(diagTimeCount>2000){
    sensorDiagnostics();
    diagTimeCount=0;
  }
  
  
  time = millis() - startTime;
}

void sensorDiagnostics(){
  #if DEBUG
    Serial.println("------------Printing robot information------------");
    
    Serial.print("LINE SENSOR:            ");
    Serial.println(analogRead(lineSensePin));
    
    Serial.println();
    
    Serial.print("FIRE SENSOR - 1:              ");
    Serial.println(analogRead(fireSensePin1));
    
    Serial.print("FIRE SENSOR - 2:            ");
    Serial.println(analogRead(fireSensePin2));
    
    Serial.print("FIRE SENSOR - 3:             ");
    Serial.println(analogRead(fireSensePin3));
    
    Serial.print("FIRE SENSOR - 4:             ");
    Serial.println(analogRead(fireSensePin4));
    
    Serial.print("FIRE SENSOR - 5:             ");
    Serial.println(analogRead(fireSensePin5));
    
    Serial.println();
    
    int angle = fireSense.fireAngle();
    Serial.print("FIRE SENSOR - Angle:             ");
    Serial.println(angle);
    
    Serial.print("FIRE SENSOR - Strength:          ");
    Serial.println(fireSense.fireStrength());
    
    Serial.println();
    
    Serial.print("DISTANCE SENSOR - Right-Back:  ");
    Serial.println(distRB.ping() / US_ROUNDTRIP_CM);
    
    Serial.print("DISTANCE SENSOR - Right-Front: ");
    Serial.println(distRF.ping() / US_ROUNDTRIP_CM);
    
    Serial.print("DISTANCE SENSOR - Front:       ");
    Serial.println(distAhead.ping() / US_ROUNDTRIP_CM);

    
    
    Serial.println("--------------------------------------------------");
  #endif
}

void testFireIndicator() {
  digitalWrite(fireIndicator,HIGH);
  delay(500);
  digitalWrite(fireIndicator,LOW);
  delay(500);
}

void testMotors() {
  rightMotor.drive(100);
  #if DEBUG
    Serial.println("Right forward Left forward");
  #endif
  leftMotor.drive(100);
  delay(1000);
  #if DEBUG
    Serial.println("Right forward Left backward");
  #endif
  leftMotor.drive(-100);
  delay(1000);

  rightMotor.drive(-100);
  #if DEBUG
    Serial.println("Right backward Left forward");
  #endif
  leftMotor.drive(100);
  delay(1000);
  #if DEBUG
    Serial.println("Right backward Left backward");
  #endif
  leftMotor.drive(-100);
  delay(1000);
}

//Function for rotation (dead recknoning!)

//Rotate clockwise 90 degrees
void rotCW90(){
  leftMotor.drive(170);
  rightMotor.drive(-230);
  delay(450);
  leftMotor.brake();
  rightMotor.brake();
}

//Rotate counterclockwise 90 degrees
void rotCCW90(){
  leftMotor.drive(-230);
  rightMotor.drive(170);
  delay(450);
  leftMotor.brake();
  rightMotor.brake();
}

void rotCCWUntilParallel(){
  leftMotor.drive(-200);
  rightMotor.drive(150);
  delay(350);
  while(true){
    leftMotor.drive(-200);
    rightMotor.drive(150);
    int rightfront = distRF.ping() / US_ROUNDTRIP_CM;
    int rightback = distRB.ping() / US_ROUNDTRIP_CM;
    int front = distAhead.ping() / US_ROUNDTRIP_CM;
    if(abs(rightfront - rightback) < 50){
      break;
    }
  }
  leftMotor.brake();
  rightMotor.brake();
}

void rotCWUntilParallel(){
  leftMotor.drive(-200);
  rightMotor.drive(150);
  delay(350);
  while(true){
    leftMotor.drive(150);
    rightMotor.drive(-200);
    int rightfront = distRF.ping() / US_ROUNDTRIP_CM;
    int rightback = distRB.ping() / US_ROUNDTRIP_CM;
    int front = distAhead.ping() / US_ROUNDTRIP_CM;
    if(abs(rightfront - rightback) < 50){
      break;
    }
  }
  leftMotor.brake();
  rightMotor.brake();
}

void leaveRoom(){
  //Rotate 180, go forward, rot cw, go forward if possible
  leftMotor.drive(-200);
  rightMotor.drive(180);
  delay(700);
  /*
  while(!isLineSensed()){
    mcontrol.drive(avgSonarVal(distRB,3),avgSonarVal(distRF,3),255);
    if(avgSonarVal(distAhead,3)>FRONTOBSTACLEDIST){
      rotCCWUntilParallel();
    }
  }
  while(isLineSensed()){
    mcontrol.drive(avgSonarVal(distRB,3),avgSonarVal(distRF,3),255);
    if(avgSonarVal(distAhead,3)>FRONTOBSTACLEDIST){
      rotCCWUntilParallel();
    }
  }
  int wallFollowDelayStart = millis();
  while(millis() - wallFollowDelayStart < 500){
    mcontrol.drive(avgSonarVal(distRB,3),avgSonarVal(distRF,3),255);
    if(avgSonarVal(distAhead,3)>FRONTOBSTACLEDIST){
      rotCCWUntilParallel();
    }
  }
  */
  
  leftMotor.drive(250);
  rightMotor.drive(250);
  delay(900);  
  if(distRB.ping() / US_ROUNDTRIP_CM<WALLONRIGHT && distRF.ping()<WALLONRIGHT){
    rotCW90();
    delay(100);
    leftMotor.drive(250);
    rightMotor.drive(250);
    delay(500);
  }
  
  leftMotor.brake();
  rightMotor.brake();
  delay(200);
}

void extinguish(){
  int pos;
  for(pos = SERVO_LOOSE; pos <= SERVO_TAUT; pos += 1) // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    pullServo.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(2);                       // waits 15ms for the servo to reach the position 
  }
  
  while(fireSense.fireStrength() > FIRESENSED){;}
  delay(200);
  
  for(pos = SERVO_TAUT; pos>=SERVO_LOOSE; pos -= 1)     // goes from 180 degrees to 0 degrees 
  {                                
    pullServo.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(2);                       // waits 15ms for the servo to reach the position 
  }
  
  //pullServo.write(SERVO_LOOSE); 
}

int avgSensorVal(int pin, int trials){
  int sum=0;
  for(int i=0; i<trials; i++){
    sum += analogRead(pin);
  }
  return sum/trials;
}

int debounce(int pin){
  int start = millis();
  int debounceDelay = 5;
  int initial = digitalRead(pin);
  while(millis() - start < debounceDelay){
    int cur = digitalRead(pin);
    if (cur!=initial){
      initial = cur;
      start = millis();
    }
  }
  return initial;
}

bool isLineSensed(){
  #if LINESENSING_INVERTED
    return (analogRead(lineSensePin)>LINESENSED);
  #else
    return (analogRead(lineSensePin)<LINESENSED);
  #endif 
}

void returnHome(int numRoomsSearched){
  delay(500);
  rotCCW90();
  leftMotor.drive(200);
  rightMotor.drive(200);
  delay(600);
  rightMotor.brake();
  leftMotor.brake();
  if(numRoomsSearched == 1){
    //exit room
    while(!isLineSensed()){
      mcontrol.drive(distRB.ping() / US_ROUNDTRIP_CM,distRF.ping() / US_ROUNDTRIP_CM,150);
      if(distAhead.ping() / US_ROUNDTRIP_CM>FRONTOBSTACLEDIST){
        rotCCWUntilParallel();
      }
    }
    //stop on line
    rightMotor.brake();
    leftMotor.brake();
    delay(250);
    //drive forward
    leftMotor.drive(150);
    rightMotor.drive(150);
    delay(800);
    //begin wallfollow, stop when hallway on right
    int rightDist = 0;
    while(rightDist<300){
      mcontrol.drive(distRB.ping() / US_ROUNDTRIP_CM, distRF.ping() / US_ROUNDTRIP_CM,150);
      rightDist=0;
      for(int x=0; x<5; x++){
        rightDist+=distRB.ping() / US_ROUNDTRIP_CM /5;
      }
      if(distAhead.ping() / US_ROUNDTRIP_CM > FRONTOBSTACLEDIST){
        rotCCWUntilParallel();
      }
    }
    //stop in middle of hallway
    delay(800);
    rightMotor.brake();
    leftMotor.brake();
    //turn 90 degrees
    delay(250);
    rotCCW90();
    //drive forward then begin wallfollowing
    leftMotor.drive(150);
    rightMotor.drive(150);
    delay(2000);
    
    rightDist=0;
    while(rightDist<300){
      mcontrol.drive(distRB.ping() / US_ROUNDTRIP_CM, distRF.ping() / US_ROUNDTRIP_CM,150);
      if(distAhead.ping() / US_ROUNDTRIP_CM > FRONTOBSTACLEDIST){
        rotCCWUntilParallel();
      }
      rightDist=0;
      for(int x=0; x<5; x++){
        rightDist+=distRB.ping() / US_ROUNDTRIP_CM / 5;
      }
    }
    //stop at hallway entrance
    rightMotor.brake();
    leftMotor.brake();
    delay(1000);
    //drive forward until the pad is hit
    #if LINESENSING_INVERTED
    while(analogRead(lineSensePin)<LINESENSED){
    #else
    while(analogRead(lineSensePin)>LINESENSED){
    #endif
      leftMotor.drive(150);
      rightMotor.drive(150);
    }
    //stop
    rightMotor.brake();
    leftMotor.brake();
    delay(1000);
    //drive forward to center on pad
    leftMotor.drive(150);
    rightMotor.drive(150);
    delay(700);
    
    rightMotor.brake();
    leftMotor.brake();
    
  }
  if(numRoomsChecked == 2){
    
  }
  if(numRoomsChecked == 3){
    
  }
  if(numRoomsChecked == 4){
    
  }
  if(numRoomsChecked == 5){
    
  }
}
