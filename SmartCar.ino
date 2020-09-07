#include <NewPing.h>  //library for distance sensor
#include <pt.h>   // include protothread library


#define FRONT 0
#define LEFT 1
#define RIGHT 2
#define NO 3
#define BACK_RAN 4
#define BACK_LEFT 5
#define BACK_RIGHT 6

static struct pt thread1, thread2; //each protothread needs one of these

bool StopSignal = false;

const int LIGHT_ERROR = 15;
byte SPEEDA = 140; //speed of left motor,max254
byte SPEEDB = 140;  //speed of right motor
const int MAX_DISTANCE_CENTER = 20;//CENTER
const int MAX_DISTANCE_RIGHT = 20;//RIGHT
const int MAX_DISTANCE_LEFT = 20;//LEFT

///////////////////////////////////////////////////////////light sensor define & function/////////////////////////////////////////
//define for light senser, the sensor I bought will return smaller value if it is brighter
//pinA is for left sensor, pinB is for right sensor
const byte LIGHT_PIN_A = 2;
const byte LIGHT_PIN_B = 3;

//return the direction of the light
//0=front, 1=left, 2=right, 3=no light
//todo:1.detect nolight 2.left right error so not turn left and right
byte lightDirection(){
  int A = map(analogRead(LIGHT_PIN_A),0,1023,0,255);
  //int A = analogRead(LIGHT_PIN_A);
  //delay(1);
  int B = map(analogRead(LIGHT_PIN_B),0,1023,0,255);
  //int B = analogRead(LIGHT_PIN_B);
  //delay(1);
  Serial.print("Light:");
  Serial.print(A);
  Serial.print(":");
  Serial.print(B);
  Serial.print(":");
  Serial.println(abs(A-B));
  if ((A == B) | (abs(A-B)<LIGHT_ERROR)){
    return FRONT;
  }else if (A < B){
    return LEFT;
  }else if (A > B){
    return RIGHT;
  }
}
///////////////////////////////////////////////////////////light sensor define & function END/////////////////////////////////////////
///////////////////////////////////////////////////////////distance sensor define & function/////////////////////////////////////////
//defina for distance sensor, A is left side sensor use to detect right side barrier, B is right side sensor
const byte TRIGGER_PIN_LEFT = 6;
const byte ECHO_PIN_LEFT = 7;

const byte TRIGGER_PIN_RIGHT = 12;
const byte ECHO_PIN_RIGHT = 13;

const byte TRIGGER_PIN_CENTER = 2;
const byte ECHO_PIN_CENTER = 3;

const byte ITERATION = 3;   //number o try and get the average value
 
NewPing sonarL(TRIGGER_PIN_LEFT, ECHO_PIN_LEFT,MAX_DISTANCE_LEFT );
NewPing sonarR(TRIGGER_PIN_RIGHT,ECHO_PIN_RIGHT ,MAX_DISTANCE_RIGHT );
NewPing sonarC(TRIGGER_PIN_CENTER,ECHO_PIN_CENTER ,MAX_DISTANCE_CENTER );

//return the value of barrier direction
//0=front, 1=left, 2=right, 3 = no barrier
byte getBarrierDir(){
  int left = sonarL.ping_median(ITERATION);
  //delay(10);
  int right = sonarR.ping_median(ITERATION);
  //delay(10);
  int center = sonarC.ping_median(ITERATION);
  //delay(10);
  Serial.print("Obsclate:");
  Serial.print(left);
  Serial.print(":");
  Serial.print(center);
  Serial.print(":");
  Serial.println(right);
  if (left !=0){
    return LEFT;
  }else if (right !=0){
    return RIGHT;
  }if (center !=0){
    return FRONT;
  }else{
    return NO;
  }
}
///////////////////////////////////////////////////////////distance sensor define & function END/////////////////////////////////////////
///////////////////////////////////////////////////////////motor part define & function/////////////////////////////////////////
//define for motor (A=left B=Right)
const byte ENA = 10;  //motor A
const byte ENB = 11;  //motor B
const byte IN1 = 4;   //motor A
const byte IN2 = 5;   //motor A
const byte IN3 = 8;   //motor B
const byte IN4 = 9;   //motor B
byte dir = 0;  //direction, if 0 then go straight, otherwise 1 

void stopMove(){
  analogWrite(ENA,0);
  analogWrite(ENB,0);
}

void forward(){
  analogWrite(ENA,SPEEDA);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  analogWrite(ENB,SPEEDB);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
}

void backward(){
  analogWrite(ENA,SPEEDA);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  analogWrite(ENB,SPEEDB);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
}

void turnLeft(){
  analogWrite(ENA,SPEEDA);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  analogWrite(ENB,SPEEDB);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
}
void turnRight(){
  analogWrite(ENA,SPEEDA);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  analogWrite(ENB,SPEEDB);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
}
///////////////////////////////////////////////////////////motor part define & function END/////////////////////////////////////////

void setup() {
  // put your setup code here, to run once:
  //motor pin all set to output
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  randomSeed(analogRead(0));//random seeds

  //initialise the two protothread variables
  PT_INIT(&thread1);
  PT_INIT(&thread2);
  
  Serial.begin(9600);

}

/* This function Detect Light and SET motor Speed after 'interval' ms passed */
static int FollowLight(struct pt *pt, int interval) 
{
  static unsigned long timestamp = 0;
  PT_BEGIN(pt);
  while(1) { // never stop 
    /* each time the function is called the second boolean
    *  argument "millis() - timestamp > interval" is re-evaluated
    *  and if false the function exits after that. */
    PT_WAIT_UNTIL(pt, millis() - timestamp > interval );
    timestamp = millis(); // take a new timestamp
    //do somthing
    Serial.println("light");
    if (StopSignal == false){
      byte light = lightDirection();
      if (light == FRONT ) 
      {
          forward();
      }else if (light == LEFT) 
      {
          turnLeft();
      }else if (light == RIGHT) 
      {
          turnRight();
      }
    }
  }
  PT_END(pt);
}

static int DetectObstacle(struct pt *pt, int interval) 
{
  static unsigned long timestamp = 0;
  PT_BEGIN(pt);
  while(1) { // never stop 
    /* each time the function is called the second boolean
    *  argument "millis() - timestamp > interval" is re-evaluated
    *  and if false the function exits after that. */
    PT_WAIT_UNTIL(pt, millis() - timestamp > interval );
    timestamp = millis(); // take a new timestamp
    //do somthing
    
    byte direction = getBarrierDir();
    if (direction == NO)
    {
      // no obscatle
      if (StopSignal == true) //stopped
      {
        StopSignal = false; //set not stopped
        forward();
      }
    }else
    {
      if (StopSignal == false ) //not stopped
      {
        StopSignal = true; //set stop
      }
      stopMove(); //stop first
      delay(100);
      long randNumber = random(1);
      //detect next direction
      byte moveDirection = getMoveDirection(direction);
      //move
      switch(moveDirection)
      {
        case LEFT:
          turnLeft();
          delay(200);
          Serial.println("left");
          break;
        case RIGHT:
          turnRight();
          delay(200);
          Serial.println("right");
          break;
        case BACK_RAN:
          backward();
          delay(200);
          if (randNumber == 0)
          {
            turnLeft();
            delay(200);
            Serial.println("back+left");
          }else
          {
            turnRight();
            delay(200);
            Serial.println("back+right");
          }
          break;
        case BACK_LEFT:
          backward();
          delay(200);
          turnLeft();
          delay(200);
          break;
        case BACK_RIGHT:
          backward();
          delay(200);
          turnRight();
          delay(200);
          break;
      }
      //whatever need to set to forward,otherwise won't move afterward
      //stopMove();
      forward();
    }
    Serial.println("detect obstacle end");
    if (StopSignal == true) //stopped
      {
        Serial.println("obstacle set stop signal false");
        StopSignal = false; //set not stopped
        forward();
      }
  }
  PT_END(pt);
}

byte getMoveDirection(byte obstacle)
{
  //byte light = lightDirection();
  byte light= NO;
  
  if (obstacle == LEFT)
  {
    if (light == FRONT)
    {
      return BACK_RIGHT;
    }else
    {
      return RIGHT;
    }
  }
  
  if (obstacle == RIGHT)
  {
    if (light == FRONT)
    {
      return BACK_LEFT;
    }else
    {
      return LEFT;
    }
  }

  if (obstacle == FRONT)
  {
    switch (light)
    {
      case FRONT:
      case NO:
        return BACK_RAN;
      case LEFT:
        return BACK_LEFT;
      case RIGHT:
        return BACK_RIGHT;
    }
  }
  return BACK_RAN; //this one should never run......
}


void loop() {
  //schedule the two protothreads
  while(1){
  DetectObstacle(&thread2,300);//over 100 is ok
  FollowLight(&thread1, 100); 
  }
  //Serial.println(getBarrierDir());
  //forward();
  //turnLeft();
  
  //turnRight();
  //delay(50);
  
}
