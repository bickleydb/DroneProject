#include <Servo.h>

//-----SERVO VARIABLES-----
//Pins
#define servHorizontal 7
#define servVertical 6
#define laserPin 45
bool laserState = false;

Servo horizontalServo;
Servo verticalServo;

//Position Tracking
int servHPos = 120; //90 Deg
int servVPos = 80; //90 Deg

static bool firstRunOnly = true;

void setup(){
 Serial.begin(9600);
 
 horizontalServo.attach(servHorizontal);
 verticalServo.attach(servVertical); 

 pinMode(laserPin, OUTPUT);
 
 pinMode(4, INPUT);
  
 setPinModes();
 attachServ();
}


int THRMIN = 1029; //values for arm - min 1000 - 800
int THRMAX = 2012;
int THRMID = ((THRMAX+THRMIN)/2)*(0.7);
int THR60 = THRMIN + 100;
int THR40 = THRMIN - 100;

int YAWMIN = 1035;
int YAWMAX = 2012; //values for arm - max 2000 2020
int YAWMID = (YAWMAX+YAWMIN)/2;
int YAW60 = YAWMID + 100;
int YAW40 = YAWMID - 100;


int ROLLMIN = 1000;
int ROLLMAX = 2000;
int ROLLMID = (ROLLMAX+ROLLMIN)/2;
int ROLL40 = ROLLMID - 100;
int ROLL60 = ROLLMID + 100;

int PITMIN = 1000;
int PITMAX = 2000;
int PITMID = (PITMAX+PITMIN)/2;
int PIT40 = PITMID - 100;
int PIT60 = PITMID + 800;

int MODEALT = 1550;
int MODESTAB = 1000;
int MODELAND = 2000;

int THR_OUT = 11;
int ROLL_OUT = 13;
int PIT_OUT = 12;
int YAW_OUT = 10;
int ARD_OUT = 8;
int MODE_OUT = 9;

Servo thrServ;
Servo rollServ;
Servo pitServ;
Servo yawServ;
Servo modeServ;

void setPinModes(){
 pinMode(THR_OUT, OUTPUT);
 pinMode(ROLL_OUT, OUTPUT);
 pinMode(YAW_OUT, OUTPUT);
 pinMode(PIT_OUT, OUTPUT);
 pinMode(MODE_OUT, OUTPUT);
}
void attachServ() {
 thrServ.attach(THR_OUT);
 rollServ.attach(ROLL_OUT);
 pitServ.attach(PIT_OUT);
 yawServ.attach(YAW_OUT);
 modeServ.attach(MODE_OUT);
}
void detachServ(){
 thrServ.detach();
 rollServ.detach();
 pitServ.detach();
 yawServ.detach();
 modeServ.detach();
}


void arm(){
 thrServ.writeMicroseconds(THRMIN);
 yawServ.writeMicroseconds(YAWMAX);
 pitServ.writeMicroseconds(PITMID);
 rollServ.writeMicroseconds(ROLLMID);
 modeServ.writeMicroseconds(MODELAND);
 modeServ.writeMicroseconds(MODESTAB);
}

void minAll(){
 thrServ.writeMicroseconds(THRMIN);
 yawServ.writeMicroseconds(YAWMIN);
 rollServ.writeMicroseconds(ROLLMIN);
 pitServ.writeMicroseconds(PITMIN);
 modeServ.writeMicroseconds(MODESTAB);
}

void maxAll(){
 thrMax();
 yawServ.writeMicroseconds(YAWMAX);
 rollServ.writeMicroseconds(ROLLMAX);
 pitServ.writeMicroseconds(PITMAX);
 modeServ.writeMicroseconds(MODELAND);
}

void thrMax(){
 thrServ.writeMicroseconds(THRMAX);
}
/*
void thrMax(){
 thrServ.writeMicroseconds(THRMAX);
}*/

/*void takeoff(){
  //balance();
  //delay(5000);
  arm();
  delay(3000);
  thrMax();
  delay(150);
  //balance();
  //setAltitude();
}*/

void stop(){
 thrServ.writeMicroseconds(THRMID);
 yawServ.writeMicroseconds(YAWMID);
 pitServ.writeMicroseconds(PITMID);
 rollServ.writeMicroseconds(ROLLMID);
}

void forward(){
  pitServ.writeMicroseconds(PIT60); 
}

void backward(){
 pitServ.writeMicroseconds(PIT40);
}

void strafeL(){
 rollServ.writeMicroseconds(ROLL40);
}

void strafeR(){
 rollServ.writeMicroseconds(ROLL60);
}

void turnL(){
 yawServ.writeMicroseconds(YAW40);
}

void turnR(){
 yawServ.writeMicroseconds(YAW60);
}

void balance(){
 thrServ.writeMicroseconds(THRMID);
 yawServ.writeMicroseconds(YAWMID);
 pitServ.writeMicroseconds(PITMID);
 rollServ.writeMicroseconds(ROLLMID);
 modeServ.writeMicroseconds(MODEALT);
}

void land(){
 modeServ.writeMicroseconds(MODELAND);
}

void disarm(){
  thrServ.writeMicroseconds(THRMIN);
  yawServ.writeMicroseconds(YAWMIN);
  delay(8000);
  modeServ.writeMicroseconds(MODESTAB);
}

void setAltitude(){
 modeServ.writeMicroseconds(MODEALT);
}

void cali(){
  minAll();
  delay(2000);
  maxAll();
}

//Top Level servo control method to be called by the pie.
//X and Y are meant to be 0-180Deg.
void servoPie (int x, int y)
{
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

void firstTest(){
  arm();
  delay(1000);
  midAll();
  //modeServ.writeMicroseconds(MODESTAB);
}

void midAll(){
 thrServ.writeMicroseconds(THRMID);
 yawServ.writeMicroseconds(YAWMID);
 rollServ.writeMicroseconds(ROLLMID);
 pitServ.writeMicroseconds(PITMID);
 modeServ.writeMicroseconds(MODELAND);
}

void toggleLaser () 
{
  laserState = !laserState;
  if(laserState) { digitalWrite(laserPin, HIGH); }
  else { digitalWrite(laserPin, LOW); }
}

void loop(){
  //Resetting servos to default positions
  //TODO there seems to be a slight varience here. 
  if(firstRunOnly) 
  {
    servoPie(120, 80);
    firstRunOnly = false;
  }

  
  int x = pulseIn(4, HIGH);
  int right = YAW60;
  //Serial.println(x);
  //Serial.println("Wiating for command");



/**
 * Pi->Arduino Protocol Proposal:
 * 
 * Raspberry Pi sends a 6 byte message that contains the desired drone movements to the Arduino. With this protocol, there are
 * 18 base messages the Pi can send the Arduino as described below.
 * 
 * The char array 'recieved' reads in 6 bytes at a time, although some bytes may not be relevant. Each byte is described as follows:
 *  The first byte signifies which catergory of movement the Arduino is to execute. The second byte, if needed, varies directly with the 
 *  first byte and describes the specific movement to be executed.
 *  
 *   The possible categories and subcategories are:
 *    i = Init Commands, these are the arming, disarming, testing, and any other commands
 *        a = Arm the drone
 *            ia0000
 *        k = Disarm the drone
 *            ik0000
 *        s = Set altitude of the drone
 *            is0000
 *        t = Run first test
 *            it0000
 *    m = Movement Commands, these are the physical x, y movements of the drone
 *        l = Strafe drone left
 *            ml####
 *        r = Strafe drone right
 *            mr####
 *        f = Move drone forward
 *            mf####
 *        b = Move drone backward
 *            mb####
 *        u = Move drone up
 *            mu####
 *        d = Move drone down
 *            md####
 *    t = Turn Commands, these are the physical z movments of the drone
 *        l = Turn drone left
 *            tl####
 *        r = Turn drone right
 *            tr####
 *    l = Laser Commands, these are the laser on/off comamnds
 *            l00000
 *    s = Servo Commands, these are the servo movements
 *        u = Move vertical servo up
 *            su0###
 *        d = Move vertical servo down
 *            sd0###
 *        l = Move horizontal servo left
 *            sl0###
 *        r = Move horizontal servo right
 *            sr0###
 *        x = Reset the servos position
 *            sx0000
 * 
 *  The remaining 4 bytes are to represent the degree of movement, or if no movement is needed they can be ignored.
 *    For the Movement and Turn Commands, the 4 digit number represents the # sent to the flight controller
 *    For the Servo Commands, the '3' digit number (really 4 digits, but the first digit is always a 0... for 90 degrees, the 4 digit is 0090, for 180 degrees, the 4 digit is 0180)
 *   
 */

if (Serial.available() > 0) {
    char received[6];
    String movement;
    int movementNum;
    Serial.readBytes(received, 6);

    switch (received[0]) {
      //Init message received
      case 'i':
        //Serial.println("Init Command: ");
        switch (received[1]){
          case 'a':
              //Serial.println("Arm Drone Command");
              arm();
              break;
          case 'k':
              //Serial.println("Kill Drone Command");
              disarm();
              break;
          case 's':
              //Serial.println("Set Altitude Command");
              setAltitude();
              break;
          case 't':
              //Serial.println("Test Comamnd");
              firstTest();
              break;
          default:
            //Serial.println("Unknown Init Command");
            break;
        }
        break;
      
      //Move message received
      case 'm':
        movement = "";
        movement.concat(received[2]);
        movement.concat(received[3]);
        movement.concat(received[4]);
        movement.concat(received[5]);
        movementNum = movement.toInt();
        //Serial.println("Movement Command:");
        switch (received[1]){
          case 'l':
            //Serial.println("Move Left: " + movementNum);
            strafeL(movementNum);
            break;
          case 'r':
            //Serial.println("Move Right: " + movementNum);
            strafeR(movementNum);
            break;
          case 'f':
            //Serial.println("Move Forward: " + movementNum);
            forward(movementNum);
            break;
          case 'b':
            //Serial.println("Move Backward: " + movementNum);
            backward(movementNum);
            break;
          case 'u':
            //Serial.println("Move Up: " + movementNum);
            thrMax(movementNum);
            break;
          case 'd':
            //Serial.println("Move Down: " + movementNum);
            minAll();
            break;
          default:
            //Serial.println("Unknown Movement Command: " + movementNum);
            break;
        }
        break;
        
      //Turn message received
      case 't':
        movement = "";
        movement.concat(received[2]);
        movement.concat(received[3]);
        movement.concat(received[4]);
        movement.concat(received[5]);
        movementNum = movement.toInt();
        //Serial.println("Turn Command:");
        switch (received[1]){
          case 'l':
            //Serial.println("Turn Left: " + movementNum);
            turnL(movementNum);
            break;
          case 'r':
            //Serial.println("Turn Right: " + movementNum);
            turnR(movementNum);
            break;
          default:
            //Serial.println("Unknown Turn Command: " + movementNum);
            break;
        }
        break;

      //Servo message received
      case 's':
        movement = "";
        movement.concat(received[2]);
        movement.concat(received[3]);
        movement.concat(received[4]);
        movement.concat(received[5]);
        
        movementNum = movement.toInt();
        //Serial.println("Servo Command:");
        switch (received[1]){
          case 'u':
            //Serial.println("Turn Servo Up:");
            //Serial.println(movementNum);
            relativeServo(0, (movementNum * -1)); //Ugly way of casting negative int
            break;
          case 'd':
           // Serial.println("Turn Servo Down:");
            //Serial.println(movementNum);
            relativeServo(0, movementNum);
            break;
          case 'l':
            //Serial.println("Turn Servo Left:");
            //Serial.println(movementNum);
            relativeServo((movementNum * -1), 0); //Ugly way of casting negative int
            break;
          case 'r':
            //Serial.println("Turn Servo Right:");
            //Serial.println(movementNum);
            relativeServo(movementNum, 0);
            break;
          case 'x':
            //Serial.println("Reset Servo");
            servoPie(120, 80);
            break;
          default:
            //Serial.println("Unknown Servo Command:");
            //Serial.println(movementNum);
            break;
        }
        break;

      //Laser message recieved
      case 'l':
        //Serial.println("Lazer Command:");
        toggleLaser();
        break;
      default:
        //Serial.println("Unknown message received");
        break;
    }
}

}

