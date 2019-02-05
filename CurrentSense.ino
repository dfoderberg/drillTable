/*
  Code to monitor the current amp draw of the actuator, and to cut power if it
  rises above a certain amount.

  Written by Progressive Automations
  August 19th, 2015

  Hardware:
  - RobotPower MegaMoto control boards
  - Arduino Uno
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

const int currentSensorActuator1 = A1; // motor feedback
const int currentSensorActuator2 = A0; // motor feedback
float ampOutM1;
float ampOutM2;
int maxAmps = 1.3; // trips at
bool jamState = false;

const int startButton = 4;
const int topSwitch = 30;
const int bottomSwitch = 31;
const int pinchSwitch = 32;

bool topState = false;
bool bottomState = false;
bool pinchState = false;
bool safetyState = false;

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

long printDelayA = millis();
long printDelayB = millis();

void setup()
{
  Serial.begin(9600);

  //pinMode(EnablePin, OUTPUT);
  pinMode(PWMPinAMotor1, OUTPUT);
  pinMode(PWMPinBMotor1, OUTPUT); //Set motor outputs
  pinMode(PWMPinAMotor2, OUTPUT);
  pinMode(PWMPinBMotor2, OUTPUT);         //Set motor outputs
  pinMode(currentSensorActuator1, INPUT); //set feedback input
  pinMode(currentSensorActuator2, INPUT); //set feedback input

  pinMode(drill1, OUTPUT);
  pinMode(drill2, OUTPUT);
  pinMode(drill3, OUTPUT);
  pinMode(drill4, OUTPUT);
  digitalWrite(drill1, LOW);
  digitalWrite(drill2, LOW);
  digitalWrite(drill3, LOW);
  digitalWrite(drill4, LOW);

  pinMode(startButton, INPUT);
  pinMode(bottomSwitch, INPUT);
  pinMode(topSwitch, INPUT);
  pinMode(pinchSwitch, INPUT);

  digitalWrite(startButton, HIGH); // enable internal pullups
  digitalWrite(bottomSwitch, HIGH);
  digitalWrite(topSwitch, HIGH);
  digitalWrite(pinchSwitch, HIGH);
  analogReadResolution(12);

  // lcd.begin(16, 2); // initalize and set dimensions of lcd.
  lcd.init(); // initialize the lcd
  // Print a message to the LCD.
  lcd.backlight();

  currentTimedebounce = millis();
  currentTimefeedback = 0; //Set initial times

  maxAmps = 15; //set max limit

} //end setup

void loop()
{
  // while (true){
  //   motorIn();
  // }
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
  int drillsStarted = 0;
  int drillStartTime = millis();
  motorIn();
  while (!bottomState) //while down
  {
    //start drills 1 at a time
    if (drillsStarted < 4 && millis() > (drillStartTime + 250 * drillsStarted))
    {
      startDrill(drillsStarted + 1);
      drillsStarted++;
    }

    readInputs();
    if (pinchState)
    {
      //check our pinch and doors
      //check for faulty amp reading
      stopDrill();
      motorOut();
      delay(1000);
      motorStop();
      return; // exit down program
    }
    printLcd("Move In");
  }

  drillStartTime = millis(); // used to set the timer used to count down till drills turn off
                             // stopDrill();

  motorOut();
  while (!topState)
  {
    printLcd("Move Out");
    readInputs();
    if (drillsOnState && millis() > (1000 + drillStartTime)) //turn drills off after 1 second delay
    {
      stopDrill();
    }
    // if (checkSafetySwitches())
    // {
    //   //check our pinch and doors
    //   //check for faulty amp reading
    //   return;
    // }
  }
  motorStop();
} //end cutFoam

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
    lcd.setCursor(0, 1);
    lcd.print("A=");
    lcd.setCursor(2, 1);
    lcd.print(ampOutM1);
    lcd.setCursor(8, 1);
    lcd.print("A=");
    lcd.setCursor(10, 1);
    lcd.print(ampOutM2);
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
  }
}

int stateConverter(bool state)
{
  if (state)
    return 1;
  else
    return 0;
}

void lcdPrintError(String error)
{
  printDelayB = millis();
  if (printDelayB - printDelayA > 500)
  { // delay is used so board doesnt refresh too fast
    printDelayA = millis();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(error);
    // lcd.setCursor(0, 1);
    // lcd.print("A=" );
  }
}

// bool checkSafetySwitches()
// {
//   int pinchState = digitalRead(pinchSwitch);
//   if (pinchState == LOW)
//   {
//     safetyState = true;
//     motorStop();
//     stopDrill();
//     lcdPrintError("Pinch");
//     return true;
//   }
//   else
//     safetyState = false;
//   return false;
// }

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

  CRawMotor1 = analogRead(currentSensorActuator1);
  CRawMotor2 = analogRead(currentSensorActuator2);
  //float analogMotor1Calibrated = CRawMotor1 - Current1BaseValue;
  float analogMotor1Calibrated = CRawMotor1 - 175;
  float analogMotor2Calibrated = CRawMotor2 - 175;
  float precision = 3.3 / 4096;
  float voltPerAmp = .0375;

  ampOutM1 = ((analogMotor1Calibrated * precision) / voltPerAmp);
  ampOutM2 = ((analogMotor2Calibrated * precision) / voltPerAmp);
  if (ampOutM1 > maxAmps|| ampOutM2 > maxAmps){
    jamState = true;
  }
}
// const int startButton = 4;
// const int topSwitch = 30;
// const int bottomSwitch = 31;
// const int pinchSwitch = 32;

// bool topState = true;
// bool bottomState = false;
// bool safetyState = true;

void motorOut()
{
  //write a smooth runup script
  // monitor sync
  analogWrite(PWMPinAMotor1, 255);
  analogWrite(PWMPinBMotor1, 0); //move motor
  analogWrite(PWMPinAMotor2, 255);
  analogWrite(PWMPinBMotor2, 0); //move motor

  getFeedback();

  //firstRun = false;

} //end motorForward

void motorIn() //could be simplified
{
  //write a smooth runup script
  analogWrite(PWMPinAMotor1, 0);
  analogWrite(PWMPinBMotor1, 255); //move motor
  analogWrite(PWMPinAMotor2, 0);
  analogWrite(PWMPinBMotor2, 255); //move motor

  getFeedback();

} //end motorBack

void motorStop()
{
  analogWrite(PWMPinAMotor1, 0);
  analogWrite(PWMPinBMotor1, 0);
  analogWrite(PWMPinAMotor2, 0);
  analogWrite(PWMPinBMotor2, 0);

  //digitalWrite(EnablePin, LOW);
  firstRun = true; //once the motor has stopped, reenable firstRun to account for startup current spikes

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
void getFeedback()
{
  CRawMotor1 = analogRead(currentSensorActuator1);
  Serial.println(CRawMotor1);
  //.075/2 amps per volt

  /* if (CRawMotor1 == 0 && hitLimits < hitLimitsmax) hitLimits = hitLimits + 1;
    else hitLimits = 0; // check to see if the motor is at the limits and the current has stopped

    if (hitLimits == hitLimitsmax && rightlatch == HIGH)
    {
    rightlatch = LOW;
    fullyRetracted = true;
    Serial.println("all the way back");
    }//end if

    else if (hitLimits == hitLimitsmax && leftlatch == HIGH)
    {
    leftlatch = LOW;
    hitLimits = 0;
    Serial.println("all the way front");
    }//end if

    if (CRawMotor1 > maxAmps)
    {
    dontExtend = true;
    leftlatch = LOW; //stop if feedback is over maximum
    }//end if
  */
  lastfeedbacktime = millis(); //store previous time for receiving feedback
} //end getFeedback
