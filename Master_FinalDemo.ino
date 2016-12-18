/*
  WALL-E SubSystem Test
  Master Board Program
  Arduino Code
  Matthew Landolfa
  Stephanie Boula
*/
#include <Servo.h> //Adds Servo library
#include <Wire.h>

Servo LeftOmni;
Servo RightOmni;
Servo FrontPivot;
//MeArm HAS 4 SERVOS
Servo xServo;  // create servo object, arm base servo - left right motion
Servo yServo;  // create servo object, left side servo - forward backwards motion
Servo zServo;  // create servo object, right side servo - forward backwards motion
Servo clawServo;  // create servo object, end of arm srevo - open,close the claw hand
//Sensors
#define trigPinF 40 // Echo Pin
#define echoPinF 41 // Trigger Pin
#define trigPinL 38 // Echo Pin
#define echoPinL 39 // Trigger Pin
#define trigPinR 42 // Echo Pin
#define echoPinR 43 // Trigger Pin
#define address 0x1E //0011110b, I2C 7bit address of HMC5883

int a = 1; //detection loop control variable
int inRange = 45;
int TargetRange = 12;
const int NoiseReject = 25; //Percentage of reading closeness for rejection filter
long duration, distance, lastDuration, Front, Left, Right;
long FrontRange, LeftRange, RightRange; //dummy variable for approach loop
int xVec, yVec, zVec, RefX, RefY, RefZ; //Magnetic direction vectors
const unsigned int maxDuration = 11650; // around 200 cm, the sensor gets flaky at greater distances.
const long speed_of_sound = 29.1;    // speed of sound microseconds per centimeter
const int MagDistort = 80; //distortion gauge for HMC5883L

void setup() {
  Serial.begin(9600);
  Wire.begin();
  //Wheel Connections
  LeftOmni.attach(8);
  RightOmni.attach(10);
  FrontPivot.attach(9);
  //MeArm Connections
  xServo.attach(4);
  yServo.attach(12);
  zServo.attach(5);
  clawServo.attach(7);
  //Sensor Connections
  pinMode(trigPinF, OUTPUT);
  pinMode(echoPinF, INPUT);
  pinMode(trigPinL, OUTPUT);
  pinMode(echoPinL, INPUT);
  pinMode(trigPinR, OUTPUT);
  pinMode(echoPinR, INPUT);
  //Put the HMC5883 IC into the correct operating mode
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();
}

void loop() {
///////PRIMARY CODE////////
  delay(100);
  Home(); //sets Home positon
  UltraSonicArray(); //Fires HC-SR04 array  for first time
//DETECTION SEQUENCE//
while((Left > inRange) && (Front > inRange) && (Right > inRange) && (a != 0)){
  xServo.detach();
  clawServo.detach();                                                                                          //
  LeftRange = inRange + 1; //populates dummy variables
  FrontRange = 0;
  RightRange = inRange + 1;
  
  for(int x = 0; x < 30; x++){ //Adjustable counting variable to act as 'timer'
    if((Left > inRange) && (Front > inRange) && (Right > inRange)){
      UltraSonicArray();
      LeftBilat(); //Cycles through spinning scan for timer duration
      }
      else {
        x = 100; //auto-violates for-loop if detection occurs
      }
  }
  for(int y = 0; y < 10; y++){ //Adjustable counting variable to act as 'timer'
    if((Left > inRange) && (Front > inRange) && (Right > inRange)){
      UltraSonicArray();
      ForwardBilat(); //Linear travel & scan for timer duarion
      }
      else {
        y = 100; //auto-violates for-loop if detection occurs
      }
  }
}  
//END OF DETECTION SEQUENCE//
  //LEFT SIDE DETECTION//
  //Dummy variable declaration for primary targeting loop
  if((Left <= inRange) && (Front >= inRange) &&(Right >= inRange)){
    LeftRange = 0;
    FrontRange = inRange;
    RightRange = inRange + 1;
    a = 0;
  }
  //Target Approach Loop - Left Side
  if((LeftRange <= inRange) && (FrontRange >= inRange) && (a == 0)){
    if(Front < inRange){
      FrontSonar(); // only interested in front sensor value changing
      ForwardBilat();
      if(Front <= TargetRange){
        StopBot();
        delay(750);
        LeftOmni.detach();
        RightOmni.detach();
        FrontPivot.detach();
        xServo.attach(4);
        clawServo.attach(7);  
        //CLAW GRAB COMMAND SEQUENCE//
        Home(); //re-iterate home position
        delay(1000);
          getHeading(); //gain reference magnetic readings
          delay(750);
          RefX == xVec;
          RefY == yVec;
          RefZ == zVec;
        ExtendArm();
        delay(750);
        DropClaw(); //Vertically lower claw
        delay(1000);
        ExtendClaw(); //extend to grab object
        delay(750);
        Grab(); //grab object in extended position
        delay(1000);
//        ObjectReturn();
//        delay(750);
        ReturnHome(); //retract to home for next reading
        delay(1000);
         getHeading(); //get reading of object distortion
         delay(750);
        if((abs(zVec-RefZ) >= MagDistort)){ //gauge degree of reading distortion
          SortMetal();
          delay(1000);
          MetalDrop();
          delay(1000);
          Home();
          delay(750);
          UltraSonicArray(); //update array variables, for when loop is exited. Object should be gone
          xServo.detach();
          clawServo.detach();
          LeftOmni.attach(8);
          RightOmni.attach(10);
          FrontPivot.attach(9);
          a = 1; //reset loop variable
          LeftRange = inRange + 1; //reset dummy variables
          FrontRange = 0;
          RightRange = inRange + 1;
          delay(500); //delay until reset completion to check violaton of loop
        }
        else{
          SortPlastic();
          delay(1000);
          PlasticDrop();
          delay(1000);
          Home();
          delay(750);
          UltraSonicArray();
          xServo.detach();
          clawServo.detach();
          LeftOmni.attach(8);
          RightOmni.attach(10);
          FrontPivot.attach(9);
          a = 1;
          LeftRange = inRange + 1;
          FrontRange = 0;
          RightRange = inRange + 1;
          delay(500);
        }
        //END OF CLAW GRAB COMMANDS//
      }
    }
    else{
      FrontSonar(); //scan with front and turn if haven't seen object in front yet
      LeftBilat();
    }
  }
  //LEFT SIDE END//
  
  //RIGHT SIDE DETECTION//
  //Dummy variable declaration for primary targeting loop
  if((Left >= inRange) && (Front >= inRange) && (Right <= inRange)){
    RightRange = 0;
    FrontRange = inRange;
    LeftRange = inRange + 1;
    a = 0;
  }
  //Target Approach Loop - Right Side
  if((FrontRange >= inRange) && (RightRange <= inRange) && (a == 0)){
    if(Front < inRange){
      FrontSonar();
      ForwardBilat();
      if(Front <= TargetRange){
        StopBot();
        delay(750);
        LeftOmni.detach();
        RightOmni.detach();
        FrontPivot.detach();
        xServo.attach(4);
        clawServo.attach(7);                                                                         //
        //CLAW GRAB COMMAND SEQUENCE//
        Home();
        delay(1000);
        getHeading();
         delay(750);
         RefX == xVec;
         RefY == yVec;
         RefZ == zVec;
        ExtendArm();
        delay(750);
        DropClaw();
        delay(1000);
        ExtendClaw();
        delay(750);
        Grab();
        delay(1000);
//        ObjectReturn();
//        delay(750);
        ReturnHome();
        delay(1000);
         getHeading();
         delay(750);
        if((abs(zVec-RefZ) >= MagDistort)){
          SortMetal();
          delay(1000);
          MetalDrop();
          delay(1000);
          Home();
          delay(750);
          UltraSonicArray();
          xServo.detach();
          clawServo.detach();
          LeftOmni.attach(8);
          RightOmni.attach(10);
          FrontPivot.attach(9);
          a = 1;
          LeftRange = inRange + 1;
          FrontRange = 0;
          RightRange = inRange + 1;
          delay(500);
        }        
        else{
          SortPlastic();
          delay(1000);
          PlasticDrop();
          delay(1000);
          Home();
          delay(750);
          UltraSonicArray();
          xServo.detach();
          clawServo.detach();
          LeftOmni.attach(8);
          RightOmni.attach(10);
          FrontPivot.attach(9);
          a = 1;
          LeftRange = inRange + 1;
          FrontRange = 0;
          RightRange = inRange + 1;
          delay(500);
        }
        //END OF CLAW GRAB COMMANDS//
      }
    }
    else{
      FrontSonar();
      RightBilat();
      }
  }
  //RIGHT SIDE END//
}

///////////EXTERNAL FUNCTIONS/////////////
void SonarSensor(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  if(duration <= 8) duration = ((inRange + 1) * speed_of_sound * 2);
  if(lastDuration == 0) lastDuration = duration;
  //Compensation parameters for intial start-up
  if(duration > (5 * maxDuration)) duration = lastDuration;
  //Rejects any reading defined to be out of sensor capacity (>1000)
  //Sets the fault reading to the last known "successful" reading
  if(duration > maxDuration) duration = maxDuration;  
  //Caps Reading output at defined maximum distance (~200)
  if((duration - lastDuration) < ((-1) * (NoiseReject / 100) * lastDuration)){
    distance = (lastDuration / 2) / speed_of_sound; //Noise filter for low range drops
  }
  distance = (duration / 2) / speed_of_sound;
  lastDuration = duration; //Stores "successful" reading for filter compensation
}
void FrontSonar() {
  //Runs sonar for all attachments, shortening Loop Inclusion
  SonarSensor(trigPinF, echoPinF);
  Front = distance;
  delay(50); //Delay 50ms before next reading.
}
void UltraSonicArray() {
  //Runs sonar for all attachments, shortening Loop Inclusion
  SonarSensor(trigPinF, echoPinF);
  Front = distance;
  SonarSensor(trigPinL, echoPinL);
  Left = distance;
  SonarSensor(trigPinR, echoPinR);
  Right = distance;
  delay(50);//Delay 50ms before next reading.
}

void getHeading(){ //Fires Magnetic Sensor for Reading
  //Tell the HMC5883L where to begin reading data
  Wire.beginTransmission(address);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
 //Read data from each axis, 2 registers per axis
  Wire.requestFrom(address, 6);
  if(6<=Wire.available()){
    xVec = Wire.read()<<8; //X msb
    xVec |= Wire.read(); //X lsb
    zVec = Wire.read()<<8; //Z msb
    zVec |= Wire.read(); //Z lsb
    yVec = Wire.read()<<8; //Y msb
    yVec |= Wire.read(); //Y lsb
  }
  // Quick Fix from steelgoose
  if (xVec > 32767)
  xVec = xVec - 65536;
  if (yVec > 32767)
  yVec = yVec - 65536;
  if (zVec > 32767)
  zVec = zVec - 65536;
}

// Locomotion routines for directions and stopping
void ForwardBilat() { //Bilateral Forward Motion
  LeftOmni.write(98); //Slow Forward Speed
  RightOmni.write(86); //Slow Forward Speed
  FrontPivot.write(93); //Halted Command
}
void LeftBilat() { //Full Left Turn
  LeftOmni.write(92); //Slow Backward Speed
  RightOmni.write(87); //Slow Forward Speed
  FrontPivot.write(98); //Left Turn Speed
}
void RightBilat() { //Full Right Turn
  LeftOmni.write(97); //Slow Forward Speed
  RightOmni.write(95); //Slow Backward Speed
  FrontPivot.write(88); //Right Turn Speed
}
void StopBot() { //Total Stand-Still
  LeftOmni.write(94); //Halted Command
  RightOmni.write(92); //Halted Command
  FrontPivot.write(93); //Halted Command
}

//MeArm Position Commands
void Home(){
  clawServo.write(130); //Close Claw // maybe change to 140 if cant hold object tight
  xServo.write(90); // Hold Arm Dead-Center
  yServo.write(90); // Hold Arm Back (FULL)
  zServo.write(175); // Hold Claw Up (FULL)
}
void ReturnHome(){
   xServo.write(90); // Hold Arm Dead-Center
  clawServo.write(130); //Close Claw // maybe change to 140 if cant hold object tight
  yServo.write(90); // Hold Arm Back (FULL)
  zServo.write(175); // Hold Claw Up (FULL)
}
void ExtendArm(){
  clawServo.write(130); //Open Claw (MEDIUM)
  xServo.write(95); // Hold Arm Dead-Center
  yServo.write(90); // Extend Arm Down and Out (FULL)increase to 160-170 if not far enough
  zServo.write(120); // Bring Claw Down (5pts short of CCW Limit)
}
void DropClaw(){
  clawServo.write(90); //Open Claw (MEDIUM)
  xServo.write(95); // Hold Arm Dead-Center
  yServo.write(20); // Hold Arm Back (FULL) go to 150 if not far enough
  zServo.write(76); // Bring Claw Down (5pts short of CCW Limit)
}
void ExtendClaw(){
  clawServo.write(90); //Open Claw (MEDIUM)
  xServo.write(95); // Hold Arm Dead-Center
  yServo.write(20); // Extend Arm Down and Out (FULL)increase to 160-170 if not far enough
  zServo.write(76); // Bring Claw Down (5pts short of CCW Limit)
}
void Grab(){
  clawServo.write(140); //Close Claw
  xServo.write(95); // Hold Arm Dead-Center
  yServo.write(20); // Extend Arm Down and Out (FULL)increase to 160-170 if not far enough
  zServo.write(76); // Bring Claw Down (5pts short of CCW Limit)
}
void SortPlastic(){
  clawServo.write(140); //Close Claw
  xServo.write(00); // Sort CW to RIGHT Bin
  yServo.write(120); // Hold Arm Back (FULL)
  zServo.write(175); // Hold Claw Up (FULL)
}
void PlasticDrop(){
  clawServo.write(90); //Open Claw (MEDIUM)
  xServo.write(00); // Sort CW to RIGHT Bin
  yServo.write(120); // Hold Arm Back (FULL)
  zServo.write(175); // Hold Claw Up (FULL)
}
void SortMetal(){
  clawServo.write(140); //Close Claw
  xServo.write(180); // Sort CCW to LEFT Bin
  yServo.write(120); // Hold Arm Back (FULL)
  zServo.write(175); // Hold Claw Up (FULL)
}
void MetalDrop(){
  clawServo.write(90); //Open Claw (MEDIUM)
  xServo.write(180); // Sort CCW to LEFT Bin
  yServo.write(120); // Hold Arm Back (FULL)
  zServo.write(175); // Hold Claw Up (FULL)
}
////// PRINT FOR PROOF CHECKING //////
//  Serial.print("Left = ");
//  Serial.print(Left);
//  Serial.print(" cm, ");
//  Serial.print("Front = ");
//  Serial.print(Front);
//  Serial.print(" cm, ");
//  Serial.print("Right = ");
//  Serial.print(Right);
//  Serial.print(" cm");
//  Serial.println();
  ////////////////
//  Serial.print("xVec = ");
//  Serial.print(xVec);
//  Serial.print(", ");
//  Serial.print("yVec = ");
//  Serial.print(yVec);
//  Serial.print(", ");
//  Serial.print("zVec = ");
//  Serial.print(zVec);
//  Serial.println();
////// END OF PRINT-CHECK //////
