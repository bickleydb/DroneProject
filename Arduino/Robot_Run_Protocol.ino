#include <Servo.h>

//-----ENCODER VARIABLES-----
//Debounce Vars
volatile unsigned long deBounceR1=0;
volatile unsigned long deBounceR2=0;
volatile unsigned long deBounceL1=0;
volatile unsigned long deBounceL2=0;

//Encoder tracking variables
volatile unsigned int encoderRPos=0;
volatile unsigned int encoderLPos=0;


//-----MOTOR VARIABLES-----
// motor one
int enR = 5;
int in1 = 12;
int in2 = 7;
// motor two
int enL = 6;
int in3 = 13;
int in4 = 8;

int motorSpeedR = 250;
int motorSpeedL = 250;
//-----SERVO VARIABLES-----
int servHorizontal = 10;
int servVertical = 11;
Servo verticalServo;
Servo horizontalServo;
//Position Tracking
int servHPos = 90;
int servVPos = 80;

//-----LASER VARIABLES-----
int laserPin = 4;
bool laserState = false;

static bool firstRunOnly = true;

void setup() {
  Serial.begin (9600);
  // set all the motor control pins to outputs
  pinMode(enR, OUTPUT);
  pinMode(enL, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  //Lasarz
  pinMode(laserPin, OUTPUT);

  //Initializing the servo pins
  horizontalServo.attach(servHorizontal);
  verticalServo.attach(servVertical); 
  
  //Wheel Encoder interrupt Pins
  // encoder pin on interrupt pin 2 - Right Wheel
  // encoder pin on interrupt pin 3 - Left Wheel
  attachInterrupt(digitalPinToInterrupt(2),encoderRight,RISING);
  attachInterrupt(digitalPinToInterrupt(3),encoderLeft,RISING);
}


void toggleLaser() {
   laserState = !laserState;
   if (laserState) {
     digitalWrite(laserPin, HIGH);
   } 
   else {
     digitalWrite(laserPin, LOW);
   }
}

//x,y standard plane and z being the distance calculated by the pie to the object.
//(x, y) will go from 0 to 180 mapping 
//VServo control rules: 0-180 Deg = angle. 90 = straight
// 91-180 = forward/down, 89-0 = backward/up
void servoPie (int x, int y) {
    servHPos = x;
    servVPos = y;
  
    horizontalServo.write(servHPos);
    verticalServo.write(servVPos);
}

void relativeServo (int x, int y) {
   servHPos = servHPos + x;
   servVPos = servVPos + y;

   horizontalServo.write(servHPos);
   verticalServo.write(servVPos);
}
   /*
   * Some sudo code for the pie since its probably easier to 
   * do the computations related to mapping. The arduino servos are limited 
   * to 180deg turns so we can map it like that.
   * 
   * xDis = desiredPixelMovementFromCenter(90/(pixelTotalWidth/2)); //Both of these need to be negative as appropriate 
   * yDis = desiredPixelMovementFromCenter(90/(pixelTotalHeight/2)); // on the image as an x/y plane
   * 
   * x = xDis + 90; //Assuming the negatives will carry over for 
   * y = yDis + 90; //both of these calculations
   * return x, y; //Send it off to the arduino
   * 
   */


//Mapping the z and component to a flat x/y plane parallel to the ground.
//Based off of anticipated pie measurement for laser vector.
void motorPie(int x,int y) 
{

  //Computer x and y distance needed to travel
  //TODO compute it as a straight line

  //The distance traveled per encoder "Ping" (meters)
  float encoderDistance = .0003;

  int encoderXTravel = (x/encoderDistance);
  int encoderYTravel = (y/encoderDistance);

  //Turn left then move the distance
  if (encoderXTravel < 0)
  {
    //The second number needs to be integrated in motorRun method for degree turn
    //Turn Left 90 Deg
    motorRun(2,90); 
    encoderLPos=0;
    encoderRPos=0;
    //now the second variable can be for distance going forward
    motorRun(1,encoderXTravel);
    encoderLPos=0;
    encoderRPos=0;
    //Turn back to starting position
    motorRun(3,90);
    encoderLPos=0;
    encoderRPos=0;
  }
  else
  {
    //The second number needs to be integrated in motorRun method for degree turn
    //Turn Left 90 Deg
    motorRun(3,90); 
    encoderLPos=0;
    encoderRPos=0;
    //now the second variable can be for distance going forward
    motorRun(1,encoderXTravel);
    encoderLPos=0;
    encoderRPos=0;
    //Turn back to starting position
    motorRun(2,90);
    encoderLPos=0;
    encoderRPos=0;
  }

  //Now go forward the specified length
  motorRun(1,encoderYTravel);
  encoderLPos=0;
  encoderRPos=0;
  
  //Turn off motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);  
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}



//----------MOTOR CONTROL----------
// Setting is 1-4 and sets the direction of the robot
//1: forward
//2: Left turn
//3: Right Turn
//4: Backward
// Speed sets the speed of the motors or motor as appropriate
// length = time spent running 
void motorRun(int setting, int modifier) 
{
 switch (setting)
  {
    //in1, in2 = Right motor's direction 1 HIGH, 2 LOW = Forward
    //in3, in4 = Left motor's direction 3 HIGH, 4 LOW = Forward
    case 1:
       encoderLPos=0;
       encoderRPos=0;
       digitalWrite(in1, HIGH);
       digitalWrite(in2, LOW);
       digitalWrite(in3, HIGH);
       digitalWrite(in4, LOW);
       analogWrite(enR, 255);
       analogWrite(enL,255);
      while ((encoderLPos<modifier*53)||(encoderRPos<modifier*53))
      {
        
        if(encoderLPos==encoderRPos)
        {
          digitalWrite(in1, HIGH);
          digitalWrite(in2, LOW);
          digitalWrite(in3, HIGH);
          digitalWrite(in4, LOW);
          analogWrite(enR, 150);
          analogWrite(enL,150);
        }
        else if(encoderLPos>=encoderRPos)
        {
          digitalWrite(in1, HIGH);
          digitalWrite(in2, LOW);
          digitalWrite(in3, HIGH);
          digitalWrite(in4, LOW);
           analogWrite(enR, 150);
          analogWrite(enL,75);
        }
        else if(encoderLPos<encoderRPos)
        {
          digitalWrite(in1, HIGH);
          digitalWrite(in2, LOW);
          digitalWrite(in3, HIGH);
          digitalWrite(in4, LOW);
           analogWrite(enR, 75);
          analogWrite(enL,150);
        }    
      }
      Serial.println(encoderLPos);
       Serial.println(encoderRPos);
     if(encoderLPos<encoderRPos)
      {
        while(encoderLPos<encoderRPos)
        {
          digitalWrite(in1, LOW);
          digitalWrite(in2, LOW);
          digitalWrite(in3, HIGH);
          digitalWrite(in4,LOW);
          analogWrite(enR,0);
          analogWrite(enL,150);
        }     
      }
      else
      {
        while(encoderRPos<encoderLPos)
        {
          digitalWrite(in1, HIGH);
          digitalWrite(in2, LOW);
          digitalWrite(in3, LOW);
          digitalWrite(in4,LOW);
          analogWrite(enR, 150);
          analogWrite(enL,0);
        }
      }
      Serial.println(encoderLPos);
      Serial.println(encoderRPos);
      break;
    case 2:
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        analogWrite(enR, 255);
        analogWrite(enL,255);
        delay(100);
      while ((encoderLPos<(modifier/45)*11) || (encoderRPos<(modifier/45)*11))
      {
        
       if(encoderLPos==encoderRPos)
       { 
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        analogWrite(enR, 180);
        analogWrite(enL,180);
       }
       else if(encoderLPos>encoderRPos)
       { 
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        analogWrite(enR, 180);
        analogWrite(enL,90);
       }
       
        else if(encoderLPos<encoderRPos)
        {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        analogWrite(enR, 90);
        analogWrite(enL,180);
       }
      }
      break;
    case 3:
      digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        analogWrite(enR, 255);
        analogWrite(enL,255);
        delay(100);
      while ((encoderLPos<(modifier/45)*11) || (encoderRPos<(modifier/45)*11))
      {
       if(encoderLPos==encoderRPos)
       { 
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        analogWrite(enR, 180);
        analogWrite(enL,180);
       }
       else if(encoderLPos>encoderRPos)
       {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        analogWrite(enR, 180);
        analogWrite(enL,90);
       }
       else if(encoderLPos<encoderRPos)
       {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        analogWrite(enR, 90);
        analogWrite(enL,180);
       }
      }
      break;
    case 4:
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        analogWrite(enR, 255);
        analogWrite(enL,255);
        delay(100);
     while ((encoderLPos<modifier*53)||(encoderRPos<modifier*53))
      {  
        
      if(encoderLPos==encoderRPos)
       { 
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        analogWrite(enR, 150);
        analogWrite(enL,150);
       }
       else if(encoderLPos>encoderRPos)
       {
       digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        analogWrite(enR, 150);
        analogWrite(enL,75);
       }
       else if(encoderLPos<encoderRPos)
       {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        analogWrite(enR, 75);
        analogWrite(enL,150);
       }
      }
      break;
      case 5:
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        analogWrite(enR, 255);
        analogWrite(enL,255);
        delay(100);
     while ((encoderLPos<=(modifier/15)*3)||(encoderRPos<(modifier/15)*3))
      {  
        
      if(encoderLPos==encoderRPos)
       { 
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        analogWrite(enR, 180);
        analogWrite(enL,180);
       }
       else if(encoderLPos>encoderRPos)
       {
       digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        analogWrite(enR, 180);
        analogWrite(enL,90);
       }
       else if(encoderLPos<encoderRPos)
       {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        analogWrite(enR, 90);
        analogWrite(enL,180);
       }
      }
      break;
    default:
      Serial.println("motorRun was called with an invlaid direction setting");
      //speed = 0;
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);  
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
      analogWrite(enR, 0);
      analogWrite(enL,0);
      break;  
  }
  
  
  
  //Turn off motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);  
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  analogWrite(enR, 0);
  analogWrite(enL,0);
  Serial.print(encoderLPos);
  Serial.print(encoderRPos);
  //left=encoderLPos;
  //right=encoderRPos;
  encoderLPos=0;
  encoderRPos=0; 
}
  
void encoderLeft()
{
  deBounceL1=millis();

  if(deBounceL1-deBounceL2 < 2) 
  {}
  else
  {
    encoderLPos += 1;
    deBounceL2 = deBounceL1;
  }
  
}

void encoderRight()
{
  deBounceR1=millis();

  if(deBounceR1-deBounceR2 < 2) 
  {}
  else
  {
    encoderRPos += 1;
    deBounceR2 = deBounceR1;
  }
}


  //motorRun(int setting, int modifier) | 
  //Setting:  1,2,3,4: Forward,Left,Right,Back
  //Modifier: For left or right == Degree Turn, Forward or Backward == Distance

  //motorPie(int x,int y) 
  //x: X Distance to travel, y: Y Distance to travel

  //servoPie(int x, int y)
  //x,y: Changes the camera to point at this position
  
  //laserpie(); Toggles the laser
void loop() {
  if (firstRunOnly){
    servoPie(90, 80);
    firstRunOnly = false;
  }
  
  if (Serial.available() > 0) {
    char received[6];
    Serial.readBytes(received, 6);
    String movement;
    int movementNum;

      
    switch (received[0]) {
      //Move message received
      //turn; send the degree 90/180/270/360
      case 'm':
        movement = "";
        movement.concat(received[2]);
        movement.concat(received[3]);
        movement.concat(received[4]);
        movement.concat(received[5]);
        movementNum = movement.toInt();
        switch (received[1]){
         case 'l':
            motorRun(2, movementNum);
            break;
          case 'r':
            motorRun(3, movementNum);
            break;
          case 'f':
            motorRun(1, movementNum);
            break;
          case 'b':
            motorRun(4, movementNum);
            break;
          case 'q':
            motorRun(5, movementNum);
            break;
          case 's':
            motorRun(5, movementNum); //Only for 15(19) degrees
            break;
        }
        break;
        
      //Turn message received
    /*  case 't':
        movement = "";
        movement.concat(received[2]);
        movement.concat(received[3]);
        movement.concat(received[4]);
        movement.concat(received[5]);
        movementNum = movement.toInt();
        switch (received[1]){
          case 'l':
            motorRun(2, movementNum);
            break;
          case 'r':
            motorRun(2, movementNum);
            break;
        }
        break;*/


      //Servo message received
      case 's':
        movement = "";
        movement.concat(received[2]);
        movement.concat(received[3]);
        movement.concat(received[4]);
        movement.concat(received[5]);
        movementNum = movement.toInt();
        switch (received[1]){
          case 'u':
            relativeServo(0, (movementNum*-1)); //Ugly way of casting negative int
            break;
          case 'd':
            relativeServo(0, movementNum);
            break;
          case 'l':
            relativeServo((movementNum * -1), 0); //Ugly way of casting negative int
            break;
          case 'r':
            relativeServo(movementNum, 0);
            break;
          case 'x':
            servoPie(90, 80);
            break;
          default:
            break;
        }
        break;

      //Laser message recieved
      case 'l':
        toggleLaser();
        break;
      default :
        break;
    }
  }
}
