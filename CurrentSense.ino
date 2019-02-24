/*
  Drilling Machine for Icon Shear Connectors

  Written by Davis Foderberg
  2019

  Hardware:
  - RobotPower MegaMoto control boards
  - Arduino mega
  
*/
// #include <LiquidCrystal.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);

// LiquidCrystal lcd(48, 49, 50, 51, 52, 53); //48 is RS   49 is E .   50 is DB4 - 53 is DB7

//const int EnablePin = 8;
const int PWMPinAMotor1 = 5;  // actuator near buttons
const int PWMPinBMotor1 = 11; // Motor 2 jumper pins Megamoto
const int PWMPinAMotor2 = 10; //Motor 2 jumper pins Megamoto
const int PWMPinBMotor2 = 9;  //actuator far side

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

volatile int speedSwitch = 27;
const int startButton = 25;
const int topSwitch = 28;
const int syncTopSwitch = 26;
const int bottomSwitch = 30;
// const int pinchSwitch = 32;
const int manualUp = 29;
const int manualDown = 31;
const int menuButton = 33;
// const int HS2Test = 51;

bool topState = false;
bool bottomState = false;
// bool pinchState = false;
bool safetyState = false;
bool jamState = false;
// bool manualState = false;
bool manualUpState = false;
bool manualDownState = false;

int menuState = 0;
bool readyToDrillState = true;
bool manualModeState = false;

// const int drill1 = 22;
// const int drill2 = 23;
// const int drill3 = 24;
// const int drill4 = 25;
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
int speed[5] = {75, 100, 150, 200, 255}; // was 100 150 200 255
// int speedCalibration[4] = {0, 0, 0, 0};
// int deltaSpeedCalibration[4] = {30, 30, 30, 8};

int txCount = 0;
char newRx = 'Z';
char receivedChar = 'Z';

long debounceArray[5]{0, 0, 0, 0, 0};
// int CountTest = 0;

void setup()
{
  Serial.begin(9600);
  Serial1.begin(57600);

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

  // pinMode(drill1, OUTPUT);
  // pinMode(drill2, OUTPUT);
  // pinMode(drill3, OUTPUT);
  // pinMode(drill4, OUTPUT);
  // digitalWrite(drill1, LOW);
  // digitalWrite(drill2, LOW);
  // digitalWrite(drill3, LOW);
  // digitalWrite(drill4, LOW);

  pinMode(speedSwitch, INPUT);
  digitalWrite(speedSwitch, HIGH);
  // attachInterrupt(digitalPinToInterrupt(speedSwitch), ISRSpeed, FALLING);

  pinMode(startButton, INPUT);
  pinMode(bottomSwitch, INPUT);
  pinMode(topSwitch, INPUT);
  // pinMode(pinchSwitch, INPUT);
  pinMode(manualUp, INPUT);
  pinMode(manualDown, INPUT);
  pinMode(menuButton, INPUT);
  // pinMode(HS2Test, INPUT);

  // digitalWrite(HS2Test, HIGH);
  digitalWrite(startButton, HIGH); // enable internal pullups
  digitalWrite(bottomSwitch, HIGH);
  digitalWrite(topSwitch, HIGH);
  // digitalWrite(pinchSwitch, HIGH);
  digitalWrite(manualUp, HIGH);
  digitalWrite(manualDown, HIGH);
  digitalWrite(menuButton, HIGH);
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

  checkRX();    // check to see if other board sent any messages
  readInputs(); // read all switch inputs
  checkMenuButton();
  if (jamState)
  {
    printLcd("JAM Clr > push S X3");
    int a = 0;
    while (a <= 3)
    {
      while (digitalRead(startButton) != LOW)
      {
        //stuck till button press
      }
      while (digitalRead(startButton) != HIGH)
      {
        // stuck till button release
      }
      delay(10); // short delay
      a++;       // count up 1
      // if (digitalRead(startButton) == LOW)
      // {
      //   delay(100);
      //   a++;
      // }
    }
    jamState = false; //exit a jam state
  }

  if (manualModeState)
  {
    runManualMode();
  }

  // if (pinchState)
  // {
  //   //do not start anything
  //   // display which switches are holding it up
  // }
  // else
  // {
  // printManualLcd();
  printLcd("Ready to Drill");                   //loop wait for input
  if (digitalRead(startButton) == LOW) // if someone presses start button.
  {
    // Serial.println("start pressed");
    if (!topState) // check if in home position. If not home return to home.
    {
      motorOut();
      while (!topState)
      {
        checkRX();
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
  // }
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
  if (jamState)
    return;
  // wait for both motors to come to a stop. this will syncronize their position
  while (!motorStopState)
  { //bottom switch is pressed now continue down until motor stops.

    //need to monitor for pinch!!!!!!!!!!!!!!!!!!!!!!!!!!!

    printLcd("Transition"); // tell user we are in transistion phase
    readInputs();           // check switch presses
    checkRX();              // check communications with HS board to see if motors have both stopped
    //create a function to see if both motors stop. use this function in stopMotor as well
  }
  delay(200);
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
    printLcd("Return to Unload");

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
void watchMotorIn() // return does not bring you out of the drill function maybe need to return a value if it errors
{
  while (!bottomState) //while down
  {
    readInputs(); // check physical switches
    checkRX();    // check to see if both motors are running and check to see if jam
    printLcd("Drilling");
    // if (pinchState)
    // {
    //   //check our pinch
    //   stopDrill();
    //   motorOut();
    //   delay(1000);
    //   motorStop();
    //   delay(50);
    //   checkRX();
    //   jamState = false;
    //   return; // exit down program
    // }
    if (jamState)
    {
      motorOut();
      delay(1000);
      motorStop();
      //need to set resync motors on to move both motors to topmost position
      return;
    }
  }
}

void runManualMode()
{
  while (manualModeState)
  {
    checkMenuButton();
    readInputs();
    printManualLcd();
    if (manualDownState)
    {
      motorIn();
    }
    else if (manualUpState)
    {
      motorOut();
    }
    else
    {
      motorStop();
    }
  }
  motorStop();
}
void checkMenuButton()
{

  if (checkDebounceArray(3) > 1000)
  {

    if (digitalRead(menuButton) == LOW)
    {
      setDebounceArray(3);
      menuState++;
    }
  }
  int num = (menuState % 2);
  switch (num)
  {
  case 0:
    readyToDrillState = true;
    manualModeState = false;
    //mode for check opperations
    break;
  case 1:
    readyToDrillState = false;
    manualModeState = true;
    break;
  // case 2:
  //   motorStopState = false;
  //   break;
  default:
    readyToDrillState = true;
    manualModeState = false;
    menuState = 0;
    break;
  }
}

void printLcd(String funcName)
{

  //Serial.println("funcName");
  printDelayB = millis();
  if (printDelayB - printDelayA > 250)
  { // delay is used so board doesnt refresh too fast
    printDelayA = millis();

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(funcName);
    lcd.setCursor(0, 1);
    lcd.print("Speed Setting = ");
    lcd.setCursor(16, 1);
    lcd.print(speedSetting);
    // lcd.setCursor(0, 2);
    // lcd.print("T=");
    // lcd.setCursor(2, 2);
    // lcd.print(stateConverter(topState));
    // lcd.setCursor(5, 2);
    // lcd.print("B=");
    // lcd.setCursor(7, 2);
    // lcd.print(stateConverter(bottomState));
    // lcd.setCursor(10, 2);
    // lcd.print("P=");
    // lcd.setCursor(12, 2);
    // lcd.print(stateConverter(pinchState));
    if (jamState)
    {
      lcd.setCursor(0, 2);
      lcd.print("Jammed Clear Machine");
      // lcd.setCursor(17, 2);
      // lcd.print(stateConverter(jamState));
    }

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

void printManualLcd()
{

  //Serial.println("funcName");
  printDelayB = millis();
  if (printDelayB - printDelayA > 250)
  { // delay is used so board doesnt refresh too fast
    printDelayA = millis();

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Manual Mode");
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
    // lcd.setCursor(0, 3);
    // lcd.print("RX");
    // lcd.setCursor(5, 3);
    // lcd.print(newRx);
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
    // lcd.setCursor(10, 2);
    // lcd.print("P=");
    // lcd.setCursor(12, 2);
    // lcd.print(stateConverter(pinchState));
    lcd.setCursor(15, 2);
    lcd.print("J=");
    lcd.setCursor(17, 2);
    lcd.print(stateConverter(jamState));
    lcd.setCursor(0, 3);
    lcd.print("U=");
    lcd.setCursor(2, 3);
    lcd.print(stateConverter(manualUpState));
    lcd.setCursor(5, 3);
    lcd.print("D=");
    lcd.setCursor(7, 3);
    lcd.print(stateConverter(manualDownState));
    lcd.setCursor(10, 3);
    lcd.print("M=");
    lcd.setCursor(12, 3);
    lcd.print(menuState);
    lcd.setCursor(15, 3);
    lcd.print("m=");
    lcd.setCursor(17, 3);
    lcd.print(stateConverter(manualModeState));

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
  // if (digitalRead(pinchSwitch))
  //   pinchState = false;
  // else
  //   pinchState = true;

  if (digitalRead(manualUp))
    manualUpState = false;
  else
    manualUpState = true;
  if (digitalRead(manualDown))
    manualDownState = false;
  else
    manualDownState = true;

  if (checkDebounceArray(1) > 1000)
  {
    if (digitalRead(speedSwitch) == LOW)
    {
      setDebounceArray(1);
      if (speedSetting < 4)
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

void startDrill()
{
  drillsOnState = true;
}

void stopDrill()
{
  drillsOnState = false;
}

void checkRX()
{
  if (Serial1.available() > 0)
  {
    receivedChar = Serial1.read();
    newRx = receivedChar;
    if (receivedChar == 'J')
    {
      jamState = true;
    }
    else if (receivedChar == 's')
    {
      motorStopState = true;
    }
    else if (receivedChar == 'm')
    {
      motorStopState = false;
    }
  }
  // txCount++;
  // num = Serial1.read();
  // newRx = num;
  // switch (num)
  // {
  // case 2:
  //   jamState = true;
  //   break;
  // case 0:
  //   motorStopState = true;
  //   break;
  // case 1:
  //   motorStopState = false;
  //   break;
  // default:

  //   break;
  // }
  // }
}

void setDebounceArray(int index)
{
  debounceArray[index] = millis();
}

long checkDebounceArray(int index)
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
