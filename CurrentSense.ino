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
#include <LiquidCrystal.h>
LiquidCrystal lcd(48, 49, 50, 51, 52, 53); //48 is RS   49 is E .   50 is DB4 - 53 is DB7

const int EnablePin = 8;
const int PWMPinAMotor1 = 3;
const int PWMPinBMotor1 = 11; // pins for Megamoto
const int startButton = 4;
const int topButton = 30;
const int bottomButton = 31;

const int drill1 = 22;
const int drill2 = 23;
const int drill3 = 24;
const int drill4 = 25;
bool drillsOnState = false;


const int currentSensorActuator1 = A0;  // motor feedback


int hitLimits = 0;
int hitLimitsmax = 10;//values to know if travel limits were reached
bool topLimit = false;
bool bottomLimit = false;

long lastfeedbacktime = 0;
int firstfeedbacktimedelay = 750; //first delay to ignore current spike
int feedbacktimedelay = 50; //delay between feedback cycles
long currentTimefeedback = 0;

int debounceTime = 300; //amount to debounce
long lastButtonpress = 0; // timer for debouncing
long currentTimedebounce = 0;

int CRaw = 0;      // raw A/D value

int maxAmps = 0; // trip limit

//bool dontExtend = false;
bool firstRun = true;
bool fullyRetracted = false;//program logic

long printDelayA = millis();
long printDelayB = millis();

void setup()
{
  Serial.begin(9600);

  pinMode(drill1, OUTPUT);
  pinMode(drill2, OUTPUT);
  pinMode(drill3, OUTPUT);
  pinMode(drill4, OUTPUT);
  digitalWrite(drill1, LOW);
  digitalWrite(drill2, LOW);
  digitalWrite(drill3, LOW);
  digitalWrite(drill4, LOW);


  pinMode(EnablePin, OUTPUT);
  pinMode(PWMPinAMotor1, OUTPUT);
  pinMode(PWMPinBMotor1, OUTPUT);//Set motor outputs

  pinMode(startButton, INPUT);
  pinMode(bottomButton, INPUT);
  pinMode(topButton, INPUT);

  digitalWrite(startButton, HIGH);// enable internal pullups
  digitalWrite(bottomButton, HIGH);
  digitalWrite(topButton, HIGH);
  analogReadResolution(12);

  lcd.begin(16, 2); // initalize and set dimensions of lcd.

  pinMode(currentSensorActuator1, INPUT);//set feedback input

  currentTimedebounce = millis();
  currentTimefeedback = 0;//Set initial times

  maxAmps = 15;//set max limit

}//end setup

void loop()
{
  if (digitalRead(startButton) == LOW) // if someone presses start button.
  {
    Serial.println("start pressed");
    if (!topLimit) // check if in home position. If not home return to home.
    {

      printLcd("Return Home");
      motorUp();
      if (digitalRead(topButton) == LOW) {
        topLimit = true;
      }
    } // end if
    else  // everything is ready start drilling.
    {
      //check our pinch and doors
      //check for faulty amp reading

      drillFoam();//runs standard opperation
    } // end else

  }// end if start is pressed
}//end main loop

void drillFoam()
{
  // start normal foam drill cycle

  // turn on drills
  topLimit = false;
  int drillsStarted = 0;
  int drillStartTime = millis();
  while (!bottomLimit) //while down
  {

    if (drillsStarted < 4 && millis() > (drillStartTime + 250 * drillsStarted))
    {
      startDrill(drillsStarted + 1);
      drillsStarted++;
    }
    CRaw = analogRead(currentSensorActuator1);

    printLcd("Move Out");

    motorDown();

  }

  bottomLimit = false;  // its here because it has reached bottom switch. now we can switch the state because we are about to call move up
  drillStartTime = millis();
  stopDrill();
  while (!topLimit)
  {
    if(drillsOnState && millis() > (1000 + drillStartTime)){
      stopDrill();
    }
    //turn drills off after 1 second delay
    printLcd("Move In");

    motorUp();
    // move motor up
  }
  motorStop();
  topLimit = true;
  //turn drills off?
  //monitor limit
  //exitdrill foam set up all variables.

}//end cutFoam


void printLcd(String funcName)
{
  //Serial.println("funcName");
  printDelayB = millis();
  if (printDelayB - printDelayA > 400) {
    printDelayA = millis();

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(funcName);
    lcd.setCursor(0, 1);
    lcd.print("current = " );
    lcd.setCursor(10, 1);
    lcd.print(CRaw);
  }
}

void motorDown()
{
  // monitor current return bool true == error
  //monitor safety return bool true == error
  // monitor sync
  //monitor limit return bool true == limit == 0 10x
  // Serial.println("Moving forwards");
  //digitalWrite(EnablePin, HIGH);
  analogWrite(PWMPinAMotor1, 255);
  analogWrite(PWMPinBMotor1, 0);//move motor
  //if (firstRun == true) delay(firstfeedbacktimedelay);
  //else delay(feedbacktimedelay); //small delay to get to speed
  if (digitalRead(bottomButton) == LOW) {
    Serial.println("bottom button");
    bottomLimit = true;
  }

  getFeedback();

  //firstRun = false;

}//end motorForward

void motorUp ()//could be simplified
{
  analogWrite(PWMPinAMotor1, 0);
  analogWrite(PWMPinBMotor1, 255);//move motor

  getFeedback();
  //monitor limit return bool true == limit == 0 10x
  if (digitalRead(topButton) == LOW) {
    topLimit = true;
  }

}//end motorBack

void motorStop()
{
  analogWrite(PWMPinAMotor1, 0);
  analogWrite(PWMPinBMotor1, 0);

  //digitalWrite(EnablePin, LOW);
  firstRun = true;//once the motor has stopped, reenable firstRun to account for startup current spikes

}//end stopMotor


void startDrill(int drillNumber)
{
  drillsOnState = true;
  switch (drillNumber) {
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

void stopDrill() {
  drillsOnState = false;
  digitalWrite(drill1, LOW);
  digitalWrite(drill2, LOW);
  digitalWrite(drill3, LOW);
  digitalWrite(drill4, LOW);
}
void getFeedback()
{
  CRaw = analogRead(currentSensorActuator1);
  Serial.println(CRaw);

  /* if (CRaw == 0 && hitLimits < hitLimitsmax) hitLimits = hitLimits + 1;
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

    if (CRaw > maxAmps)
    {
    dontExtend = true;
    leftlatch = LOW; //stop if feedback is over maximum
    }//end if
  */
  lastfeedbacktime = millis();//store previous time for receiving feedback
}//end getFeedback
