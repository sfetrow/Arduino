/*
 * @Description: In User Settings Edi
 * @Author: your name
 * @Date: 2019-08-12 18:00:25
 * @LastEditTime: 2019-08-27 10:45:29
 * @LastEditors: Please set LastEditors
 */
#include <IRremote.h>
#include <Servo.h>
#include <stdio.h>

#include "HardwareSerial.h"

#include "ArduinoJson-v6.11.1.h" //Use ArduinoJson Libraries

#define f 16736925    // FORWARD  
#define b 16754775    // BACK     
#define l 16720605    // LEFT     
#define r 16761405    // RIGHT    
#define s 16712445    // STOP     
#define KEY1 16738455 //Line Teacking mode       
#define KEY2 16750695 //Obstacles Avoidance mode 

#define KEY_STAR 16728765
#define KEY_HASH 16732845


/*Arduino pin is connected to the IR Receiver*/
#define RECV_PIN 12


/*Arduino pin is connected to the Ultrasonic sensor module*/
#define ECHO_PIN A4
#define TRIG_PIN A5

/*Arduino pin is connected to the Motor drive module*/
#define ENA 5
#define ENB 6
#define IN1 7
#define IN2 8
#define IN3 9
#define IN4 11

#define LED_Pin 13

/*Arduino pin is connected to the IR tracking module*/
#define LineTeacking_Pin_Right 10
#define LineTeacking_Pin_Middle 4
#define LineTeacking_Pin_Left 2

#define LineTeacking_Read_Right !digitalRead(10) //Right
#define LineTeacking_Read_Middle !digitalRead(4) //Middle
#define LineTeacking_Read_Left !digitalRead(2)   //Left

#define carSpeed 180 //PWM(Motor speed/Speed)

Servo servo;             //  Create a DC motor drive object
IRrecv irrecv(RECV_PIN); //  Create an infrared receive drive object
decode_results results;  //  Create decoding object

unsigned long IR_PreMillis;
unsigned long LT_PreMillis;

int rightDistance = 0;  //Right distance
int leftDistance = 0;   //left Distance
int middleDistance = 0; //middle Distance

/*DIY_MotorControl: Motor Control： Motor Speed、Motor Direction、Motor Time*/
uint8_t DIY_MotorSelection;
uint8_t DIY_MotorDirection;

uint16_t DIY_MotorSpeed;
unsigned long DIY_leftMotorControl_Millis;
unsigned long DIY_rightMotorControl_Millis;

/*DIY_CarControl: Car Control：Car moving direction、Car Speed、Car moving time*/
uint8_t DIY_CarDirection;
uint8_t DIY_CarSpeed;
uint16_t DIY_CarTimer;
unsigned long DIY_CarControl_Millis;

uint8_t DIY_CarDirectionxxx;
uint8_t DIY_CarSpeedxxx;

uint16_t DIY_Distance;

String CommandSerialNumber; //

enum FUNCTIONMODE
{
  IDLE,                  /*free*/
  LineTeacking,          /*Line Teacking Mode*/
  ObstaclesAvoidance,    /*Obstacles Avoidance Mode*/
  Bluetooth,             /*Bluetooth Control Mode*/
  IRremote,              /*Infrared Control Mode*/
  DIY_MotorControl,      /*Motor Control Mode*/
  DIY_CarControl,        /*Car Control Mode*/
  DIY_CarControlxxx,     /*Car Control Mode*/
  DIY_ClearAllFunctions, /*Clear All Functions Mode*/
} func_mode = IDLE;      /*Functional mode*/

enum MOTIONMODE
{
  STOP,            /*stop*/
  FORWARD,         /*forward*/
  BACK,            /*back*/
  LEFT,            /*left*/
  RIGHT            /*right*/
} mov_mode = STOP; /*move mode*/

void delays(unsigned long t)
{

  for (unsigned long i = 0; i < t; i++)
  {
    // getBTData();
    getBTData_Plus();//Bluetooth Communication Data Acquisition
    getIRData(); //Infrared Communication Data Acquisition
    delay(1);
  }
}
/*
Acquisition Distance: Ultrasound
*/
unsigned int getDistance(void)
{ //Getting distance
  static unsigned int tempda = 0;
  unsigned int tempda_x = 0;
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  tempda_x = ((unsigned int)pulseIn(ECHO_PIN, HIGH) / 58);
  if (tempda_x < 150)
  {
    tempda = tempda_x;
  }
  else
  {
    tempda = 30;
  }
  return tempda;
}
/*
  Control motor：Car movement forward
*/
void forward(bool debug, int16_t in_carSpeed)
{

  analogWrite(ENA, in_carSpeed);
  analogWrite(ENB, in_carSpeed);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  if (debug)
    Serial.println("Go forward!");
}

/*
  Control motor：Car moving backwards
*/
void back(bool debug, int16_t in_carSpeed)
{

  analogWrite(ENA, in_carSpeed);
  analogWrite(ENB, in_carSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  if (debug)
    Serial.println("Go back!");
}
/*
  Control motor：The car turns left and moves forward
*/
void left(bool debug, int16_t in_carSpeed)
{

  analogWrite(ENA, 250);
  analogWrite(ENB, 250);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  if (debug)
    Serial.println("Go left!");
}
/*
  Control motor：The car turns right and moves forward
*/
void right(bool debug, int16_t in_carSpeed)
{
  analogWrite(ENA, 250);
  analogWrite(ENB, 250);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  if (debug)
    Serial.println("Go right!");
}
/*
  Stop motor control：Turn off the motor drive
*/
void stop(bool debug = false)
{
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
  if (debug)
    Serial.println("Stop!");
}
/*
  Bluetooth serial port data acquisition and control command parsing
*/
void getBTData_Plus(void)
{
  String comdata = "";

  while ((Serial.available() > 0) && (false == comdata.endsWith("}")))
  {
    comdata += char(Serial.read());
    delay(6);
  }
  if ((comdata.length() > 0) && (comdata != "") && (true == comdata.endsWith("}")))
  {
    //Serial.print(comdata);
    //comdata = "{\"N\":\"2\",\"D1\":1}";
    //{"N":2,"D1":1}
    StaticJsonDocument<200> doc;                                //Create a JsonDocument object
    DeserializationError error = deserializeJson(doc, comdata); //Deserialize JSON data
    if (!error)                                                 //Check if deserialization is successful
    {
      int control_mode_N = doc["N"];
      char buf[3];
      uint8_t temp = doc["H"];
      sprintf(buf, "%d", temp);
      CommandSerialNumber = buf; //Get the serial number of the new command

      switch (control_mode_N)
      {
      case 21: /*Ultrasonic module  processing <command：N 21>*/
        DIY_UltrasoundModuleStatus_Plus(doc["D1"]);
        break;
      case 22: /*Trace module data processing <command：N 22>*/
        DIY_TraceModuleStatus_Plus(doc["D1"]);
        break;
      case 1: /*Motion module  processing <command：N 1>*/
        func_mode = DIY_MotorControl;
        DIY_MotorSelection = doc["D1"];
        DIY_MotorDirection = doc["D2"];
        DIY_MotorSpeed = doc["D3"];

        //Serial.print("{ok}");
        Serial.print('{' + CommandSerialNumber + "_ok}");
        break;
      case 4: /*Motion module  processing <command：N 4>*/
        func_mode = DIY_CarControl;
        DIY_CarDirection = doc["D1"];
        DIY_CarSpeed = doc["D2"];
        DIY_CarTimer = doc["T"];
        DIY_CarControl_Millis = millis(); //Get the timestamp
        //Serial.print("{ok}");
        break;
      case 40:
        func_mode = DIY_CarControlxxx;
        DIY_CarDirectionxxx = doc["D1"];
        DIY_CarSpeedxxx = doc["D2"];
        //Serial.print("{ok}");
        Serial.print('{' + CommandSerialNumber + "_ok}");
        break;
      case 5: /*Clear mode  processing <command：N 5>*/
        func_mode = DIY_ClearAllFunctions;
        //Serial.print("{ok}");
        Serial.print('{' + CommandSerialNumber + "_ok}");
        break;
      case 3: /*Remote switching mode  processing <command：N 3>*/
        if (1 == doc["D1"]) // Line Teacking Mode
        {
          func_mode = LineTeacking;
          //Serial.print("{ok}");
          Serial.print('{' + CommandSerialNumber + "_ok}");
        }
        else if (2 == doc["D1"]) //Obstacles Avoidance Mode
        {
          func_mode = ObstaclesAvoidance;
          //Serial.print("{ok}");
          Serial.print('{' + CommandSerialNumber + "_ok}");
        }
        break;
      case 2: /*Remote switching mode  processing <command：N 2>*/

        if (1 == doc["D1"])
        {
          func_mode = Bluetooth;
          mov_mode = LEFT;
          //Serial.print("{ok}");
          Serial.print('{' + CommandSerialNumber + "_ok}");
        }
        else if (2 == doc["D1"])
        {
          func_mode = Bluetooth;
          mov_mode = RIGHT;
          //Serial.print("{ok}");
          Serial.print('{' + CommandSerialNumber + "_ok}");
        }
        else if (3 == doc["D1"])
        {
          func_mode = Bluetooth;
          mov_mode = FORWARD;
          //Serial.print("{ok}");
          Serial.print('{' + CommandSerialNumber + "_ok}");
        }
        else if (4 == doc["D1"])
        {
          func_mode = Bluetooth;
          mov_mode = BACK;
          //Serial.print("{ok}");
          Serial.print('{' + CommandSerialNumber + "_ok}");
        }
        else if (5 == doc["D1"])
        {
          func_mode = Bluetooth;
          mov_mode = STOP;
          //Serial.print("{ok}");
          Serial.print('{' + CommandSerialNumber + "_ok}");
        }
        break;
      default:
        break;
      }
    }
  }
  else if (comdata != "")
  {
    if (true == comdata.equals("f"))
    {
      func_mode = DIY_CarControlxxx;
      DIY_CarDirectionxxx = 3;
      DIY_CarSpeedxxx = 180;
    }
    else if (true == comdata.equals("b"))
    {
      func_mode = DIY_CarControlxxx;
      DIY_CarDirectionxxx = 4;
      DIY_CarSpeedxxx = 180;
    }
    else if (true == comdata.equals("l"))
    {
      func_mode = DIY_CarControlxxx;
      DIY_CarDirectionxxx = 1;
      DIY_CarSpeedxxx = 180;
    }
    else if (true == comdata.equals("r"))
    {
      func_mode = DIY_CarControlxxx;
      DIY_CarDirectionxxx = 2;
      DIY_CarSpeedxxx = 180;
    }
    else if (true == comdata.equals("s"))
    {

      func_mode = Bluetooth;
      mov_mode = STOP;
    }
    else if (true == comdata.equals("1"))
    {
      func_mode = LineTeacking;
    }
    else if (true == comdata.equals("2"))
    {
      func_mode = ObstaclesAvoidance;
    }
  }
}
/*
  Infrared Communication Data Acquisition
*/
void getIRData(void)
{

  if (irrecv.decode(&results))
  {
    IR_PreMillis = millis();
    switch (results.value)
    {
    case f:
      func_mode = IRremote;
      mov_mode = FORWARD;
      break; /*forward*/
    case b:
      func_mode = IRremote;
      mov_mode = BACK;
      break; /*back*/
    case l:
      func_mode = IRremote;
      mov_mode = LEFT;
      break; /*left*/
    case r:
      func_mode = IRremote;
      mov_mode = RIGHT;
      break; /*right*/
    case s:
      func_mode = IRremote;
      mov_mode = STOP;
      break; /*stop*/
    case KEY1:
      func_mode = LineTeacking;
      break; /*Line Teacking Mode*/
    case KEY2:
      func_mode = ObstaclesAvoidance;
      break; /*Obstacles Avoidance Mode*/
    default:
      break;
    }
    irrecv.resume();
  }
}
/*
  Bluetooth remote control mode
*/
void bluetooth_mode()
{
  if (func_mode == Bluetooth)
  {
    switch (mov_mode)
    {
    case FORWARD:
      forward(false, carSpeed);
      break;
    case BACK:
      back(false, carSpeed);
      break;
    case LEFT:
      left(false, carSpeed);
      break;
    case RIGHT:
      right(false, carSpeed);
      break;
    case STOP:
      stop();
      break;
    default:
      break;
    }
  }
}
/*
  Infrared remote control mode
*/
void irremote_mode(void)
{
  if (func_mode == IRremote)
  {
    switch (mov_mode)
    {
    case FORWARD:
      forward(false, carSpeed);
      break;
    case BACK:
      back(false, carSpeed);
      break;
    case LEFT:
      left(false, carSpeed);
      break;
    case RIGHT:
      right(false, carSpeed);
      break;
    case STOP:
      stop();
      break;
    default:
      break;
    }
    if (millis() - IR_PreMillis > 500)
    {
      mov_mode = STOP;
      IR_PreMillis = millis();
    }
  }
}
/*
  Line Teacking Mode
*/
void line_teacking_mode(void)
{
  if (func_mode == LineTeacking)
  {
    if (LineTeacking_Read_Middle)
    { //Detecting in the middle infrared tube

      forward(false, carSpeed); //Control motor：the car moving forward
      LT_PreMillis = millis();
    }
    else if (LineTeacking_Read_Right)
    { //Detecting in the right infrared tube

      right(false, carSpeed); //Control motor：the car moving right
      while (LineTeacking_Read_Right)
      {
        getBTData_Plus();//Bluetooth data acquisition
        //getBTData();
        getIRData(); //Infrared data acquisition
      }
      LT_PreMillis = millis();
    }
    else if (LineTeacking_Read_Left)
    {         //Detecting in the left infrared tube
      left(false, carSpeed); //Control motor：the car moving left
      while (LineTeacking_Read_Left)
      {
        getBTData_Plus();//Bluetooth data acquisition
        //getBTData();
        getIRData(); //Infrared data acquisition
      }
      LT_PreMillis = millis();
    }
    else
    {
      if (millis() - LT_PreMillis > 150)
      {
        stop(); //Stop motor control：Turn off motor drive mode
      }
    }
  }
}
/*
  Obstacles Avoidance Mode
*/
/*f(x) int */
static boolean function_xxx(long xd, long sd, long ed) //f(x)
{
  if (sd <= xd && xd <= ed)
    return true;
  else
    return false;
}
/*Obstacle avoidance*/
void obstacles_avoidance_mode(void)
{
  static uint16_t ULTRASONIC_Get = 0;
  static unsigned long ULTRASONIC_time = 0;
  static boolean stateCar = false;
  static boolean CarED = false;
  static uint8_t switc_ctrl = 0x00;
  static boolean timestamp = true;

  if (func_mode == ObstaclesAvoidance)
  {
    servo.attach(3);
    if (millis() - ULTRASONIC_time > 100)
    {
      ULTRASONIC_Get = getDistance(); //Measuring obstacle distance
      ULTRASONIC_time = millis();
      if (function_xxx(ULTRASONIC_Get, 0, 25)) //If the distance is less than Xcm obstacles
      {
        stateCar = true;
        stop(); //stop
      }
      else
      {
        stateCar = false;
      }
    }
    if (false == CarED)
    {
      if (stateCar == true)
      {
        timestamp = true;
        CarED = true;
        switc_ctrl = 0x00;
        stop();          //stop
        servo.write(30); //sets the servo position according to the  value
        delays(500);
        if (function_xxx(getDistance(), 0, 25)) //How many cm in the front have obstacles?
        {
          switc_ctrl |= 0x01;
          //goto
        }
        else
        {
          switc_ctrl &= (~0x01);
        }
        servo.write(150); //sets the servo position according to the  value
        delays(500);
        if (function_xxx(getDistance(), 0, 25)) //How many cm in the front have obstacles?
        {
          switc_ctrl |= 0x02;
          //goto
        }
        else
        {
          switc_ctrl &= (~0x02);
        }
        servo.write(90); //tell servo to go to position in variable 30
        delays(500);
        if (function_xxx(getDistance(), 0, 25)) //How many cm in the front have obstacles?
        {
          switc_ctrl |= 0x04;
          //goto
        }
        else
        {
          switc_ctrl &= (~0x04);
        }
      }
      else
      {
        forward(false, 180); //Control motor：the car moving forwar
        CarED = false;
      }
    }

    if (true == CarED)
    {
      // Get cpu time
      static unsigned long MotorRL_time;
      if (timestamp == true || millis() - MotorRL_time > 420)
      {
        timestamp = false;
        MotorRL_time = millis();
      }
      if (function_xxx((millis() - MotorRL_time), 0, 400))
      {
        switch (switc_ctrl)
        {
        case 0 ... 1:
          left(false, 150); //Control motor：The car moves forward and left
          break;
        case 2:
          right(false, 150); //Control motor：The car moves forward and right
          break;
        case 3:
          forward(false, 150); //Control motor：the car moving forwar
          break;
        case 4 ... 5:
          left(false, 150); //Control motor：The car moves forward and left
          break;
        case 6:
          right(false, 100); //Control motor：The car moves forward and right
          break;
        case 7:
          back(false, 150); //Control motor：Car backwards
          break;
        }
      }
      else
      {
        CarED = false;
      }
    }
  }
  else
  {
    servo.detach();
    ULTRASONIC_Get = 0;
    ULTRASONIC_time = 0;
  }
}

/*****************************************************Begin@DIY**************************************************************************************/

/*

  N21:command
  DIY mode：Ultrasonic module：App controls module status, module sends data to app
*/
void DIY_UltrasoundModuleStatus_Plus(uint8_t is_get) //Ultrasonic module processing
{
  //uint16_t DIY_Distance = getDistance(); //Ultrasonic module measuring distance

  if (1 == is_get) // is_get Start  true：Obstacles / false:No obstacles
  {
    if (DIY_Distance <= 40)
    {
      // Serial.print("{true}");
      Serial.print('{' + CommandSerialNumber + "_true}");
    }
    else
    {
      //Serial.print("{false}");
      Serial.print('{' + CommandSerialNumber + "_false}");
    }
  }
  else if (2 == is_get) //Ultrasonic is_get data
  {
    char toString[10];
    sprintf(toString, "%d", DIY_Distance);
    // Serial.print(toString);
    Serial.print('{' + CommandSerialNumber + '_' + toString + '}');
  }
}
/*
  N22:command
   DIY mode：Teacking module：App controls module status, module sends data to app
*/
void DIY_TraceModuleStatus_Plus(uint8_t is_get) //Tracking module processing
{
  if (0 == is_get) /*Get traces on the left*/
  {
    if (LineTeacking_Read_Left)
    {
      //Serial.print("{true}");
      Serial.print('{' + CommandSerialNumber + "_true}");
    }
    else
    {
      //Serial.print("{false}");
      Serial.print('{' + CommandSerialNumber + "_false}");
    }
  }
  else if (1 == is_get) /*Get traces on the middle*/
  {
    if (LineTeacking_Read_Middle)
    {
      //Serial.print("{true}");
      Serial.print('{' + CommandSerialNumber + "_true}");
    }
    else
    {
      //Serial.print("{false}");
      Serial.print('{' + CommandSerialNumber + "_false}");
    }
  }
  else if (2 == is_get)
  { /*Get traces on the right*/

    if (LineTeacking_Read_Right)
    {
      //Serial.print("{true}");
      Serial.print('{' + CommandSerialNumber + "_true}");
    }
    else
    {
      //Serial.print("{false}");
      Serial.print('{' + CommandSerialNumber + "_false}");
    }
  }
}

/*
  N1:command
  DIY mode：Sport mode <motor control> Control motor by app
  Input：uint8_t is_MotorSelection,  Motor selection   1：left  2：right  0：all
        uint8_t is_MotorDirection,   Motor steering  1：Forward  2：Reverse 0：stop
        uint8_t is_MotorSpeed,       Motor speed   0-250   
*/
void DIY_MotorControl_Plus(uint8_t is_MotorSelection, uint8_t is_MotorDirection, uint8_t is_MotorSpeed)
{
  static boolean MotorControl = false;

  if (func_mode == DIY_MotorControl) //Motor control mode
  {
    MotorControl = true;
    if (is_MotorSelection == 1 || is_MotorSelection == 0) //Left motor
    {
      if (is_MotorDirection == 1) //Positive rotation
      {
        analogWrite(ENA, is_MotorSpeed);
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
      }
      else if (is_MotorDirection == 2) //Reverse
      {
        analogWrite(ENA, is_MotorSpeed);
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
      }
      else if (is_MotorDirection == 0) 
      {
        digitalWrite(ENA, LOW); //Turn off the motor enable pin
      }
    }
    if (is_MotorSelection == 2 || is_MotorSelection == 0) //Right motor
    {
      if (is_MotorDirection == 1) //Positive rotation
      {
        analogWrite(ENB, is_MotorSpeed);
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
      }
      else if (is_MotorDirection == 2) //Reverse
      {
        analogWrite(ENB, is_MotorSpeed);
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
      }
      else if (is_MotorDirection == 0) 
      {
        digitalWrite(ENB, LOW); //Turn off the motor enable pin
      }
    }
  }
  else
  {
    if (MotorControl == true)
    {
      MotorControl = false;
      digitalWrite(ENA, LOW); //Turn off the motor enable pin
      digitalWrite(ENB, LOW);
    }
  }
}

/*
  N4：command
  DIY mode：<Car control> APP control car
  Time limited
*/
void DIY_CarControl_Plus(uint8_t is_CarDirection, uint8_t is_CarSpeed, uint8_t is_Timer)
{
  static boolean CarControl = false;

  static boolean CarControl_TE = false; //Have time to spend
  static boolean CarControl_return = false;
  if (func_mode == DIY_CarControl) //Car Control Mode
  {
    CarControl = true;
    if (is_Timer != 0) //Setting time cannot be empty
    {
      if ((millis() - DIY_CarControl_Millis) > (is_Timer * 1000)) //check the time
      {
        CarControl_TE = true;
        digitalWrite(ENA, LOW); //Turn off the motor enable pin
        digitalWrite(ENB, LOW);

        if (CarControl_return == false)
        {

          Serial.print('{' + CommandSerialNumber + "_ok}");
          CarControl_return = true;
        }
      }
      else
      {
        CarControl_TE = false; //Have time to spend
        CarControl_return = false;
      }
    }
    if (CarControl_TE == false)
    {
      switch (is_CarDirection)
      {
      case 1: /*Left-Forward Motion Mode*/
        left(false, is_CarSpeed);
        break;
      case 2: /*Right-Forward Motion Mode*/
        right(false, is_CarSpeed);
        break;
      case 3: /*Sport mode forward*/
        forward(false, is_CarSpeed);
        break;
      case 4: /*Sport mode back*/
        back(false, is_CarSpeed);
        break;
      default:
        break;
      }
    }
  }
  else
  {
    if (CarControl == true)
    {
      CarControl_return = false;
      CarControl = false;
      digitalWrite(ENA, LOW); //Turn off the motor enable pin
      digitalWrite(ENB, LOW);
      DIY_CarControl_Millis = 0;
    }
  }
}

/*
  N40：command
  DIY mode：<Car control> APP control car
  No time limit
*/
void DIY_CarControl_Plusxxx(uint8_t is_CarDirection, uint8_t is_CarSpeed)
{
  static boolean CarControl = false;
  if (func_mode == DIY_CarControlxxx) //Car Control Mode
  {
    CarControl = true;
    switch (is_CarDirection)
    {
    case 1: /*Left-Forward Motion Mode*/
      left(false, is_CarSpeed);
      break;
    case 2: /*Right-Forward Motion Mode*/
      right(false, is_CarSpeed);
      break;
    case 3: /*Sport mode forward*/
      forward(false, is_CarSpeed);
      break;
    case 4: /*Sport mode back*/
      back(false, is_CarSpeed);
      break;
    default:
      break;
    }
  }
  else
  {
    if (CarControl == true)
    {
      CarControl = false;
      digitalWrite(ENA, LOW); //Turn off the motor enable pin
      digitalWrite(ENB, LOW);
    }
  }
}

/*
  N5:command
  DIY mode：Clear all features
*/
void DIY_ClearAllFunctionsXXX(void)
{
  if (func_mode == DIY_ClearAllFunctions)
  {

    mov_mode = STOP;
    func_mode = IDLE;
    digitalWrite(ENA, LOW); //Turn off the motor enable pin
    digitalWrite(ENB, LOW);

    /*DIY_MotorControl:Motor Control： Motor Speed、Motor Direction、Motor Time*/
    DIY_MotorSelection = NULL;
    DIY_MotorDirection = NULL;

    DIY_MotorSpeed = NULL;
    DIY_leftMotorControl_Millis = NULL;
    DIY_rightMotorControl_Millis = NULL;

    /*DIY_CarControl:Car Control：Car moving direction、Car Speed、Car moving time*/
    DIY_CarDirection = NULL;
    DIY_CarSpeed = NULL;
    DIY_CarTimer = NULL;
    DIY_CarControl_Millis = NULL;
  }
}

void getDistance_xx(void)
{
  DIY_Distance = getDistance(); //Ultrasonic measurement distance
}

/*****************************************************End@DIY**************************************************************************************/

void setup(void)
{
  Serial.begin(9600);         //initialization
  servo.attach(3, 500, 2400); //500: 0 degree  2400: 180 degree
  servo.write(90);            //sets the servo position according to the 90（middle）
  irrecv.enableIRIn();        //Enable infrared communication NEC

  pinMode(ECHO_PIN, INPUT); //Ultrasonic module initialization
  pinMode(TRIG_PIN, OUTPUT);

  pinMode(IN1, OUTPUT); //Motor-driven port configuration
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(LineTeacking_Pin_Right, INPUT); //Infrared tracking module port configuration
  pinMode(LineTeacking_Pin_Middle, INPUT);
  pinMode(LineTeacking_Pin_Left, INPUT);
}

void loop(void)
{
  DIY_Distance = getDistance(); //Ultrasonic measurement distance
  getBTData_Plus();             //Bluetooth data acquisition
  getIRData();                  //Infrared data acquisition

  bluetooth_mode();           //Bluetooth remote mode
  irremote_mode();            //Infrared NEC remote control mode
  line_teacking_mode();       //Line Teacking Mode
  obstacles_avoidance_mode(); //Obstacles Avoidance Mode

  /*DIY_MotorControl: Motor Control： Motor Speed、Motor Direction、Motor Time*/
  DIY_MotorControl_Plus(DIY_MotorSelection, DIY_MotorDirection, DIY_MotorSpeed); //Control motor steering

  /*  DIY mode：<Car control> APP control car*/
  DIY_CarControl_Plus(DIY_CarDirection, DIY_CarSpeed, DIY_CarTimer); //Control the direction of the car<Time limited>  
  DIY_CarControl_Plusxxx(DIY_CarDirectionxxx, DIY_CarSpeedxxx);      //Control the direction of the car<No Time limited>
  DIY_ClearAllFunctionsXXX();
}
