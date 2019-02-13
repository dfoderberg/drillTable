/*
  Drilling Machine for Icon Shear Connectors

  Written by Davis Foderberg
  2019

  Hardware:
  - RobotPower MegaMoto control boards
  - Arduino due
  - 2 pushbuttons
*/
// #include <LiquidCrystal.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);

// LiquidCrystal lcd(48, 49, 50, 51, 52, 53); //48 is RS   49 is E .   50 is DB4 - 53 is DB7

//const int EnablePin = 8;
const int PWMPinAMotor1 = 3;
const int PWMPinBMotor1 = 11; // Motor 2 jumper pins Megamoto
const int PWMPinAMotor2 = 10; //Motor 2 jumper pins Megamoto
const int PWMPinBMotor2 = 9;

// const int currentSensorActuator1 = A1; // motor feedback
// const int currentSensorActuator2 = A0; // motor feedback
// float ampOutM1;
// float ampOutM2;
// float maxAmps = 200; // trips at
// float maxOutputedAmps = 1;
// long directionChangeTimer = 0;
String motorDirection = "none";

// volatile int positionMotor1 = 0; //if the interrupt will change this value, it must be volatile
// float motor1Speed = 0;
// float smoothedMotor1Speed = 0;
// int previousPositionM1 = 0;
// int previousTime = 0;
// const int interuptM1 = 35;

volatile int speedSwitch = 41;
const int startButton = 4;
const int topSwitch = 30;
const int bottomSwitch = 31;
const int pinchSwitch = 32;
// const int HS2Test = 51;

bool topState = false;
bool bottomState = false;
bool pinchState = false;
bool safetyState = false;
bool jamState = false;

const int drill1 = 22;
const int drill2 = 23;
const int drill3 = 24;
const int drill4 = 25;
bool drillsOnState = false;

int hitLimits = 0;
int hitLimitsmax = 10; //values to know if travel limits were reached

long lastfeedbacktime = 0;
int firstfeedbacktimedelay = 750; //first delay to ignore current spike
int feedbacktimedelay = 50;       //delay between feedback cycles
long currentTimefeedback = 0;

int debounceTime = 300;   //amount to debounce
long lastButtonpress = 0; // timer for debouncing
long currentTimedebounce = 0;

int CRawMotor1 = 0; // raw analog input value
int CRawMotor2 = 0;

int Current1BaseValue = 0; // allows you to calibrate current sensor
int countCurrentBase = 0;

//bool dontExtend = false;
bool firstRun = true;
bool fullyRetracted = false; //program logic
bool motorStopState = true;

long printDelayA = millis();
long printDelayB = millis();
int speedSetting = 3;
int speed[4] = {100, 150, 200, 255};
int speedCalibration[4] = {00, 00, 00, 0};
int deltaSpeedCalibration[4] = {30, 30, 30, 8};

int txCount = 0;
int newRx = 10;

long debounceArray[3]{0, 0, 0};
// int CountTest = 0;

void setup()
{
  Serial.begin(9600);
  Serial1.begin(9600);

  //pinMode(EnablePin, OUTPUT);
  pinMode(PWMPinAMotor1, OUTPUT);
  pinMode(PWMPinBMotor1, OUTPUT); //Set motor outputs
  pinMode(PWMPinAMotor2, OUTPUT);
  pinMode(PWMPinBMotor2, OUTPUT); //Set motor outputs
  // pinMode(currentSensorActuator1, INPUT); //set feedback input
  // pinMode(currentSensorActuator2, INPUT); //set feedback input

  // pinMode(interuptM1, INPUT);
  // digitalWrite(interuptM1, LOW);                                         //enable internal pullup resistor
  // attachInterrupt(digitalPinToInterrupt(interuptM1), ISRMotor1, RISING); //Interrupt initialization

  pinMode(drill1, OUTPUT);
  pinMode(drill2, OUTPUT);
  pinMode(drill3, OUTPUT);
  pinMode(drill4, OUTPUT);
  digitalWrite(drill1, LOW);
  digitalWrite(drill2, LOW);
  digitalWrite(drill3, LOW);
  digitalWrite(drill4, LOW);

  pinMode(speedSwitch, INPUT);
  digitalWrite(speedSwitch, HIGH);
  // attachInterrupt(digitalPinToInterrupt(speedSwitch), ISRSpeed, FALLING);

  pinMode(startButton, INPUT);
  pinMode(bottomSwitch, INPUT);
  pinMode(topSwitch, INPUT);
  pinMode(pinchSwitch, INPUT);
  // pinMode(HS2Test, INPUT);

  // digitalWrite(HS2Test, HIGH);
  digitalWrite(startButton, HIGH); // enable internal pullups
  digitalWrite(bottomSwitch, HIGH);
  digitalWrite(topSwitch, HIGH);
  digitalWrite(pinchSwitch, HIGH);
  // analogReadResolution(12);

  // lcd.begin(16, 2); // initalize and set dimensions of lcd.
  lcd.init(); // initialize the lcd
  // Print a message to the LCD.
  lcd.backlight();

  currentTimedebounce = millis();
  currentTimefeedback = 0; //Set initial times

} //end setup

void loop()
{
  // while (true){
  //   motorIn();
  // }
  checkRX();
  if (jamState)
  {
    printLcd("JAM Clr > push S X3");
    int a = 0;
    while (a <= 3)
    {
      if (digitalRead(startButton) == LOW)
      {
        delay(100);
        a++;
      }
    }
    jamState = false;
    // maxOutputedAmps = 1;
  }

  printLcd("begin");
  readInputs();
  if (pinchState)
  {
    //do not start anything
    // display which switches are holding it up
  }
  else
  {
    printLcd("Ready");                   //loop wait for input
    if (digitalRead(startButton) == LOW) // if someone presses start button.
    {
      // Serial.println("start pressed");
      if (!topState) // check if in home position. If not home return to home.
      {
        motorOut();
        while (!topState)
        {
          printLcd("Return Home");
          readInputs();
          if (jamState)
          {
            motorIn();
            delay(500);
            motorStop();
            return;
          }
        }
        motorStop();
      }    // end if
      else // everything is ready start drilling.
      {
        drillFoam(); //runs standard opperation
      }              // end else

    } // end if start is pressed
  }
} //end main loop

void drillFoam()
{
  // start normal foam drill cycle

  // turn on drills
  startDrill();

  // start motor in / down
  motorIn();

  // monitor in / down progress check for jams and when it reaches bottom also check for Pinch bar
  watchMotorIn();

  // bottom switch has been triggered

  // wait for both motors to come to a stop. this will syncronize their position
  while (!motorStopState)
  {                         //bottom switch is pressed now continue down until motor stops.
    printLcd("Transition"); // tell user we are in transistion phase
    readInputs();           // check switch presses
    checkRX();              // check communications with HS board to see if motors have both stopped
    //create a function to see if both motors stop. use this function in stopMotor as well
  }
  jamState = false; // jam likely triggered during transistion. ignore this jam message

  // we are all the way to the bottom and have drilled all holes. Turn of the drills
  stopDrill();

  // begin moving motors out / up
  motorOut();

  // watch motors to make sure they arive at correct top position (top switch or topReSync) also monitor for jams
  watchMotorOut();
} //end cutFoam

void watchMotorOut()
{
  // check if it is returning to the top state or the sync at top
  while (!topState)
  {
    readInputs(); // check physical switches
    checkRX();    // check to see if both motors are running and check to see if jam
    printLcd("Move Out");

    if (jamState)
    {
      motorIn();
      delay(500);
      motorStop();
      return;
    }
  }
  motorStop();
}
void watchMotorIn()
{
  while (!bottomState) //while down
  {
    readInputs(); // check physical switches
    checkRX();    // check to see if both motors are running and check to see if jam
    printLcd("Move In");
    if (pinchState)
    {
      //check our pinch
      stopDrill();
      motorOut();
      delay(1000);
      motorStop();
      delay(50);
      checkRX();
      jamState = false;
      return; // exit down program
    }
    if (jamState)
    {
      motorOut();
      delay(500);
      motorStop();
      //need to set resync motors on to move both motors to topmost position
      return;
    }
  }
}

void printLcd(String funcName)
{

  //Serial.println("funcName");
  printDelayB = millis();
  if (printDelayB - printDelayA > 500)
  { // delay is used so board doesnt refresh too fast
    printDelayA = millis();

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(funcName);
    lcd.setCursor(15, 0);
    lcd.print(speedSetting);
    lcd.setCursor(0, 1);
    lcd.print("TX #");
    lcd.setCursor(5, 1);
    lcd.print(txCount);
    lcd.setCursor(8, 1);
    lcd.print("Stop");
    lcd.setCursor(13, 1);
    lcd.print(motorStopState);
    lcd.setCursor(15, 1);
    lcd.print("RX");
    lcd.setCursor(18, 1);
    lcd.print(newRx);
    // lcd.setCursor(0, 1);
    // lcd.print("A=");
    // lcd.setCursor(2, 1);
    // lcd.print(ampOutM1);
    // lcd.setCursor(8, 1);
    // lcd.print("A=");
    // lcd.setCursor(10, 1);
    // lcd.print(ampOutM2);
    lcd.setCursor(0, 2);
    lcd.print("T=");
    lcd.setCursor(2, 2);
    lcd.print(stateConverter(topState));
    lcd.setCursor(5, 2);
    lcd.print("B=");
    lcd.setCursor(7, 2);
    lcd.print(stateConverter(bottomState));
    lcd.setCursor(10, 2);
    lcd.print("P=");
    lcd.setCursor(12, 2);
    lcd.print(stateConverter(pinchState));
    lcd.setCursor(15, 2);
    lcd.print("J=");
    lcd.setCursor(17, 2);
    lcd.print(stateConverter(jamState));
    // lcd.setCursor(0, 3);
    // lcd.print("HS#=");
    // lcd.setCursor(4, 3);
    // lcd.print(positionMotor1);
    // lcd.setCursor(12, 3);
    // // lcd.print((positionMotor1 - previousPositionM1));
    // lcd.print(smoothedMotor1Speed);

    // lcd.setCursor(0, 3);
    // lcd.print("Max amps=");
    // lcd.setCursor(10, 3);
    // lcd.print(maxOutputedAmps);
  }
}

int stateConverter(bool state)
{
  if (state)
    return 1;
  else
    return 0;
}

void readInputs()
{
  if (digitalRead(topSwitch))
    topState = false;
  else
    topState = true;
  if (digitalRead(bottomSwitch))
    bottomState = false;
  else
    bottomState = true;
  if (digitalRead(pinchSwitch))
    pinchState = false;
  else
    pinchState = true;

  if (checkDebounceArray(1) > 1000)
  {
    if (digitalRead(speedSwitch) == LOW)
    {
      setDebounceArray(1);
      if (speedSetting < 3)
      {
        speedSetting++;
      }
      else
      {
        speedSetting = 0;
      }
    }
  }

  // CRawMotor1 = analogRead(currentSensorActuator1);
  // CRawMotor2 = analogRead(currentSensorActuator2);
  //float analogMotor1Calibrated = CRawMotor1 - Current1BaseValue;
  // float analogMotor1Calibrated = CRawMotor1 - 175;
  // float analogMotor2Calibrated = CRawMotor2 - 175;
  // float precision = 3.3 / 4096;
  // float voltPerAmp = .0375;

  // ampOutM1 = ((analogMotor1Calibrated * precision) / voltPerAmp);
  // ampOutM2 = ((analogMotor2Calibrated * precision) / voltPerAmp);
  // if ((millis() - directionChangeTimer) >= 2000)
  // {
  //   if (ampOutM1 > maxOutputedAmps)
  //   {
  //     maxOutputedAmps = ampOutM1;
  //   }
  //   if (ampOutM2 > maxOutputedAmps)
  //   {
  //     maxOutputedAmps = ampOutM2;
  //   }
  // }
  // if (maxOutputedAmps > maxAmps)
  // {
  //   jamState = true;
  // }
}

void motorOut()
{
  //write a smooth runup script
  // monitor sync

  //tx motorSpeed maybe send a signal like 254 so that i know its full speed in up. because its not a possible setting
  motorDirection = "out";
  // directionChangeTimer = millis(); // used for amps may delete later
  analogWrite(PWMPinAMotor1, 255);
  analogWrite(PWMPinBMotor1, 0); //move motor
  analogWrite(PWMPinAMotor2, 255);
  analogWrite(PWMPinBMotor2, 0); //move motor

} //end motorForward

void motorIn() //could be simplified
{
  //write a smooth runup script

  // tx motorSpeed
  motorDirection = "in";
  // directionChangeTimer = millis(); // used for amps may delete later
  analogWrite(PWMPinAMotor1, 0);
  analogWrite(PWMPinBMotor1, speed[speedSetting]); //move motor
  analogWrite(PWMPinAMotor2, 0);
  analogWrite(PWMPinBMotor2, speed[speedSetting]); //move motor

} //end motorBack

void motorStop()
{
  //check to see if motor1 and motor 2 did stop may need a small delay
  motorDirection = "none";
  analogWrite(PWMPinAMotor1, 0);
  analogWrite(PWMPinBMotor1, 0);
  analogWrite(PWMPinAMotor2, 0);
  analogWrite(PWMPinBMotor2, 0);

} //end stopMotor

void startDrill(int drillNumber)
{
  drillsOnState = true;
  switch (drillNumber)
  {
  case 1:
    digitalWrite(drill1, HIGH);
    break;
  case 2:
    digitalWrite(drill2, HIGH);
    break;
  case 3:
    digitalWrite(drill3, HIGH);
    break;
  case 4:
    digitalWrite(drill4, HIGH);
    break;
  }
}

void stopDrill()
{
  drillsOnState = false;
  digitalWrite(drill1, LOW);
  digitalWrite(drill2, LOW);
  digitalWrite(drill3, LOW);
  digitalWrite(drill4, LOW);
}

void checkRX()
{

  //will need to differentiate motors?
  // char letter;
  int num = 100;
  if (Serial1.available())
  {
    txCount++;
    num = Serial1.read();
    newRx = num;
    switch (num)
    {
    case 2:
      jamState = true;
      break;
    case 0:
      motorStopState = true;
      break;
    case 1:
      motorStopState = false;
      break;
    default:

      break;
    }
  }
}

void setDebounceArray(int index)
{
  debounceArray[index] = millis();
}

int checkDebounceArray(int index)
{
  return (millis() - debounceArray[index]);
}

// void lcdPrintError(String error)
// {
//   printDelayB = millis();
//   if (printDelayB - printDelayA > 500)
//   { // delay is used so board doesnt refresh too fast
//     printDelayA = millis();
//     lcd.clear();
//     lcd.setCursor(0, 0);
//     lcd.print(error);
//     // lcd.setCursor(0, 1);
//     // lcd.print("A=" );
//   }
// }

// void calculateLoadSpeed()
// {
//   float previousMotor1Speed = motor1Speed;
//   long tempTime = millis();
//   int tempPosition = positionMotor1;
//   float timeDelta = tempTime - previousTime;
//   float positionDelta = abs(tempPosition - previousPositionM1);
//   motor1Speed = positionDelta / timeDelta * 1000;
//   smoothedMotor1Speed = ((motor1Speed + previousMotor1Speed) / 2);
//   previousTime = tempTime;
//   previousPositionM1 = tempPosition;
// }

// void ISRMotor1()
// {
//   if (motorDirection == "out")
//   {
//     positionMotor1 = positionMotor1 + 1;
//   }
//   else if (motorDirection == "in")
//   {
//     positionMotor1 = positionMotor1 - 1;
//   }

// } //end Interrupt Service Routine (ISR)

// void ISRSpeed()
// {
//   if (speedSetting < 3)
//   {
//     speedSetting++;
//   }
//   else
//   {
//     speedSetting = 0;
//   }
// }
