// Photogrammetry Table code
// Original code By Brian Brocken
// Adapted Ryan Hashiro's code for I2C LCD
// Adapted Phildil's code for HC-05 bluetooth
// Adapted for Nikon DSLR 10 pins shutter release connector
// Dual isolation opto module added for Camera safety
// The phone must first be paired with the HC05 and must have CameraPro and its optional BT plugin installed.
// Then before each session the phone should be nearby, should be receiving over BT and the CameraPro app should be open in the foreground.

// Adapted by Yann Vautrin for Nikon 10 pin remote shutter release
// Thanks to aarg and J-M-L from Arduino user forum for helping with the code!

// Nikon D300 10 pin shutter release connector map (might work with other Nikon cameras too)
// pin 1  RX Data TTL Level 1
// pin 2  Batt Voltage
// pin 3  Meter on = 5V, off = 0V
// pin 4  Shutter release control
// pin 5  ?
// pin 6  Signal GND
// pin 7  Power GND
// pin 8  TX Data TTL Level 1
// pin 9  Autofocus/Meter on
// pin 10 ?

/*

  Original code By Brian Brocken
  Contribution By Ryan Hashiro: 
  1. Added library calls and setups for an LCD with I2C backpack.
  2. Added fast value change routines for the number of photogrammetry pics, motor speed, and cinematic turns.  Holding joystick deflection for > 1 sec enters fast change mode.  Change rate is proportional to amount of deflection.  For the motor speed change is fixed at the slow speed since there's only 17 values to go through.
  3. Added cancel routines for the photogrammetry and cinematic modes.  Holding the joystick button down for > 2 sec kicks you out of the operation modes.

*/

#include <LiquidCrystal.h>  // use this library for the standard (non I2C) LCD
#include <Stepper.h>
#include "progressbar.h"       // custom chars for progress bar  - NB this file is in the sketch dir

LiquidCrystal lcd(1, 2, 4, 5, 6, 7);  // Use these pins for the 1602 lcd


// Constants for the size of the LCD screen * /
const int LCD_NB_ROWS = 2;
const int LCD_NB_COLUMNS = 16;

const int SW_pin = 8; // digital pin connected to switch output
const int X_pin = A0; // analog pin connected to X output
const int Y_pin = A1; // analog pin connected to Y output

const int FocusPin   = 3; // digital pin connected to Camera focus pin
const int ShutterPin = 13; // digital pin connected to Camera shutter release pin

int MenuNr = 0;   // Menu number
int PhotoNo = 2;  // The amount of photos that have to be taken
bool Flag1 = 0;   // This flag is only active during 1 program cycle (prevents constantly adding/subtracting 1 to the menu number when the joystick is pushed to the side) 
bool Flag2 = 0;   // This flag is only active during 1 program cycle (prevents constantly adding/subtracting 2 to the photo number when the joystick is pushed up or down)
bool Flag3 = 0;   // This flag is only active during 1 program cycle (prevents constantly adding/subtracting 1 to the RPM when the joystick is pushed up or down)
bool Flag4 = 0;   // This flag is only active during 1 program cycle (prevents constantly adding/subtracting 1 to the turn number when the joystick is pushed to the side)
bool Flag5 = 0;   // This flag is only active during 1 program cycle (prevents constantly adding/subtracting 1 to the RPM when the joystick is pushed up or down)
bool Flag6 = 0;   // This flag is only active during 1 program cycle to clear the lcd
int SwMenu = 0;   // Switch menu (Sub menu's in the main menu's)
bool BtnFlag = 0; // This flag is only active during 1 program cycle (prevents constantly adding of 1 to SwMenu when button is pressed)
int totalPhotos = 60;  // The total number of photos to be taken in Interval Mode - default to 60 - this is displayed and made editable via joystick
static byte progressPercent = 0; // the percentage counter (used by the display bar)
bool takingVideo = false;
bool pulledFocus = false;

// RH - added variables for fast change & cancel modes
int FastChng = 0;  // indicates fast change value mode.  0 = off, 1 = delay mode, 2 = fast changing mode
const unsigned long FastDelay = 1000;  // delay mode time (before values change fast)
const unsigned long ShortInt = 100;  // short fast change interval
const unsigned long LongInt = 300;  // long fast change interval
const unsigned long BtnDelay = 2000;  // delay for button press to cancel operations.  Note this is an approximate delay, since stepper motor
                                     // suspends all program execution until motor finishes its move
unsigned long SetTime = 0; // time value for fast change & button cancel modes.  Used to calculate time intervals
bool BtnCancelFlag = 0; // This flag is used to detect when button is pressed for canceling operations
bool MaxSwMenu = 0;  // This flag is used for detecting when the maximum SwMenu is reached
bool CinCancelFlag = 0;  // This flag is used to trigger cinematic cancel.  1 = cancel cinematic operation
int StepPoll = 480;  // number of motor steps to poll for cinematic cancel (at 15 rpm)
int Cntr = 0;  // step counter for cinematic motor cancel
// RH - end of added variables

const int stepsPerRevolution = 2048;  // change this to fit the number of steps per revolution
int FullRev = 14336;                  // 1 full revolution of the big gear -> Small-Big gear ratio is 7:1
int rolePerMinute = 15;               // Adjustable range of 28BYJ-48 stepper is 0~17 rpm
int PhotoTaken = 0;                   // Amount of photo's that have been taken
int StepPerPhoto;                     // Amount of steps per photo (calculated -> see MenuNr 0/SwMenu 2)
int TurnNr = 1;                       // Amount of turns
int CurrentTurn = 0;                  // Stores current turn number
int Steps = 0;                        // Amount of individual steps the stepper motor has to turn

Stepper myStepper(stepsPerRevolution, 9, 11, 10, 12);  // Use these pins for the stepper motor

void setup()
{
lcd.begin(16, 2);                   //Lcd setup for standard (non I2C) LCD connection
setup_progressbar(); // set up the groovy progress bar   see https://www.carnetdumaker.net/articles/faire-une-barre-de-progression-avec-arduino-et-liquidcrystal/#barre-de-progression-v3

pinMode(SW_pin, INPUT);             //Set pushbutton as input
digitalWrite(SW_pin, HIGH);         //Set SW_pin High

pinMode(3, OUTPUT);                 //Set FocusPin as output
pinMode(13, OUTPUT);                 //Set ShutterPin as output

myStepper.setSpeed(rolePerMinute);  //Set RPM of steppermotor

  lcd.setCursor(4, 0);                //Startup screen start
  lcd.print("Welcome!");              //      """""      //
  lcd.setCursor(1, 1);                //      """""      //
  lcd.print("Software: V1.6");        //      """""      //
  delay(3000);                        //      """""      //
  lcd.clear();                        //      """""      //
  lcd.setCursor(0, 0);                //      """""      //
  lcd.print("Designed by");           //      """""      //
  lcd.setCursor(0, 1);                //      """""      //
  
  lcd.print("Brian Brocken");         //      """""      //
  delay(2000);                        //      """""      //
  lcd.clear();                        //Startup screen end
}

void loop() {

  int XValue = analogRead(X_pin);     // Read the analog value from The X-axis from the joystick
  int YValue = analogRead(Y_pin);     // Read the analog value from The Y-axis from the joystick
  int SwValue = digitalRead(SW_pin);  // Read the digital value from The Button from the joystick

  if (MenuNr < 0){  //This sets the min number of menu's
    MenuNr = 0;
  }
  else if ( MenuNr > 2){  //This sets the max numbers of menu's
    MenuNr = 2;
  }
  
  if (XValue < 400 && Flag1 == 0 && SwMenu == 0){  //if the joystick is pushed to the right side and flag1 is 0 then 1 will be added to the menu number (purpose of the flag -> see comment Flags above)
    MenuNr = MenuNr + 1; 
    Flag1 = 1;
    lcd.clear();
  }

  if (XValue > 600 && Flag1 == 0 && SwMenu == 0){  //if the joystick is pushed to the left side and flag1 is 0 then 1 will be subtracted from the menu number (purpose of the flag -> see comment Flags above)
    MenuNr = MenuNr - 1;
    Flag1 = 1;
    lcd.clear();
  }
  
  if (XValue > 399 && XValue < 599 && Flag1 == 1){  //if joystick is at neutral possition, flag1 is set to 0 (purpose of the flag -> see comment Flags above)
    Flag1 = 0;
  }


  if (SwValue == 0 && BtnFlag == 0 && MaxSwMenu == 0){  //if the button is pressed and the flag is 0 -> add 1 to menu 
    SwMenu = SwMenu + 1;
    BtnFlag = 1;
    BtnCancelFlag = 0;
    lcd.clear();
  }

  if (SwValue == 1 && BtnFlag == 1){  //if the button is not pressed and the flag is 0 -> Reset the flag (purpose of the flag -> see comment Flags above)
    BtnFlag = 0;
  }
  

//***********************************************Menu0***********************************************//

  if (MenuNr == 0){  //Menu0 program

    if (SwMenu == 0){ //Menu 0 selected
      lcd.setCursor(0, 0);
      lcd.print("Photogrammetry");
    }
  
    if (SwMenu == 1){ //entered menu 0
      lcd.setCursor(0, 0);
      lcd.print("No of photos");
      lcd.setCursor(7, 1);
      lcd.print(PhotoNo);

      if (FastChng == 0) {  // RH - if not in fast change mode, update set time
        SetTime = millis();
      }
      
      if (YValue < 400 && Flag2 == 0){ //joystick up -> Add 2 to number of photo's
        PhotoNo = PhotoNo + 2;
        Flag2 = 1;
        FastChng = 1;
        lcd.clear();
      }
      if (YValue > 600 && Flag2 == 0){ //joystick down -> Subtract 2 from number of photo's
        PhotoNo = PhotoNo - 2;
        Flag2 = 1;
        FastChng = 1;
        lcd.clear();
      }
      if (YValue > 399 && YValue < 599 && Flag2 == 1){  //if the Y-axis is back at it's neutral possition -> Flag3 = 0 -> Prevents constant adding or subtracting of 2
        Flag2 = 0;
        FastChng = 0;
      }

      if (YValue < 400 && FastChng == 1) {  // RH - if joystick up is held for > the fast delay time, then enter fast change mode
        if ((millis() - SetTime) > FastDelay) {
          FastChng = 2;
          SetTime = millis();  // update the set time
        }
      }

      if (YValue > 600 && FastChng == 1) {  // RH - if joystick down is held for > the fast delay time, then enter fast change mode
        if ((millis() - SetTime) > FastDelay) {
          FastChng = 2;
          SetTime = millis();  // update the set time
        }
      }

      if (YValue < 400 && FastChng == 2) {  // RH - we are in fast change mode, so we want to update the values quickly, proportional between
        if ((millis() - SetTime) > (LongInt - (400 - YValue) * (LongInt - ShortInt) / 400)) {  // ShortInt and LongInt based on joystick deflection
          PhotoNo = PhotoNo + 2;
          SetTime = millis();
          lcd.clear();
        }
      }

      if (YValue > 600 && FastChng == 2) {  // RH - same for joystick down in fast change mode
        if ((millis() - SetTime) > (LongInt - (YValue - 600) * (LongInt - ShortInt) / 400)) {
          PhotoNo = PhotoNo - 2;
          SetTime = millis();
          lcd.clear();
        }
      }

      if (PhotoNo < 2){    //Min allowable Nr of photo's
        PhotoNo = 2;
      }
      if (PhotoNo > 200){  //Max allowable Nr of photo's
        PhotoNo = 200;
      }
    }

    if (SwMenu == 2){ //Program started

      MaxSwMenu = 1;

      if (!pulledFocus)
      {
        pullFocus(); // attempt to focus the phone camera once per run (could be made more frequent if required)
        pulledFocus = true;
      }
      
      lcd.setCursor(0, 0);
      // lcd.print("Program started");
      
      lcd.print("Photo No: " + String(PhotoTaken + 1));
      lcd.setCursor(0, 1);

      float photoPercent = 100 / (float)PhotoNo; // the percentage of the display bar taken up by each photo
      StepPerPhoto = FullRev / PhotoNo;  //Calculate amount of steps per photo

      // increment the progress bar
      progressPercent = progressPercent + round(photoPercent);
      //   Serial.println("Progress bar at " + String(progressPercent));
      draw_progressbar(false, progressPercent);
      

      if (PhotoTaken < PhotoNo){          
      myStepper.setSpeed(rolePerMinute);  //Set motor speed
      myStepper.step(StepPerPhoto);       //move the calculated amount of steps
      takePhoto(); // Attempt to Take Photo (currently no way of checking if it worked)
      PhotoTaken++;                       //Add 1 to photos taken
    
      delay(1000);
      }

      if (PhotoTaken == PhotoNo){  //If the amount of photos taken is equal to the amount of photos that have to be taken -> Program finished
        lcd.setCursor(0, 0);
        lcd.print("Program finished");
        delay(3000);
        lcd.clear();  //Rest parameters
        PhotoTaken = 0;
        PhotoNo = 2;
        SwMenu = 0;
        MaxSwMenu = 0;
        Steps = 0;
        progressPercent = 0;
        pulledFocus = false;
      }

      // RH NOTE - cancelling works but delay is longer than expected since stepper motor movement is blocking
      if (SwValue == 0 && BtnCancelFlag == 0) {  // if button is pressed, start timing for cancel
        BtnCancelFlag = 1;
        SetTime = millis();
      }

      if (SwValue == 1 && BtnCancelFlag == 1) {  // RH - if button released before BtnDelay, then reset flag
        BtnCancelFlag = 0;
      }

      if (SwValue == 0 && BtnCancelFlag == 1 && ((millis() - SetTime) > BtnDelay)) {  // RH - button has been held for > BtnDelay, so cancel operation
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Program cancel");
        delay(3000);
        lcd.clear();
        PhotoTaken = 0;
        PhotoNo = 2;
        SwMenu = 0;
        MaxSwMenu = 0;
        Steps = 0;
        BtnCancelFlag = 0;
        progressPercent = 0;
        pulledFocus = false;
      }
    }
  }


//***********************************************Menu1***********************************************//

  if (MenuNr == 1){  //Menu1 program
    
    if (SwMenu == 0){  //Menu1 selected
      lcd.setCursor(0, 0);
      lcd.print("Cinematic");
      CinCancelFlag = 0;
      Cntr = 0;
    }

    if (SwMenu == 1){  //Entered menu1 - sub menu1
      lcd.setCursor(0, 0);
      lcd.print("motor speed");
      lcd.setCursor(7, 1);
      lcd.print(rolePerMinute);

      if (FastChng == 0) {  // RH - if not in fast change mode, update set time
        SetTime = millis();
      }
      
      if (YValue < 400 && Flag3 == 0){ // joystick up -> Add 1 RPM
        rolePerMinute = rolePerMinute + 1;
        Flag3 = 1;
        FastChng = 1;
        lcd.clear();
      }
      if (YValue > 600 && Flag3 == 0){ // joystick down -> Subtract 1 RPM
        rolePerMinute = rolePerMinute - 1;
        Flag3 = 1;
        FastChng = 1;
        lcd.clear();
      }
      if (YValue > 399 && YValue < 599 && Flag3 == 1){  //if the Y-axis is back at it's neutral possition -> Flag3 = 0 -> Prevents constant adding or subtracting of 1
        Flag3 = 0;
        FastChng = 0;
      }

      if (YValue < 400 && FastChng == 1) {  // RH - if joystick up is held for > the fast delay time, then enter fast change mode
        if ((millis() - SetTime) > FastDelay) {
          FastChng = 2;
          SetTime = millis();  // update the set time
        }
      }

      if (YValue > 600 && FastChng == 1) {  // RH - if joystick down is held for > the fast delay time, then enter fast change mode
        if ((millis() - SetTime) > FastDelay) {
          FastChng = 2;
          SetTime = millis();  // update the set time
        }
      }

      if (YValue < 400 && FastChng == 2) {  // RH - we are in fast change mode, so we want to update the values at the LongInt interval
        if ((millis() - SetTime) > LongInt) {  // (using fixed LongInt since not that many values to cycle through)
          rolePerMinute = rolePerMinute + 1;
          SetTime = millis();
          lcd.clear();
        }
      }

      if (YValue > 600 && FastChng == 2) {  // RH - same for joystick down in fast change mode
        if ((millis() - SetTime) > LongInt) {
          rolePerMinute = rolePerMinute - 1;
          SetTime = millis();
          lcd.clear();
        }
      }
      
      if (rolePerMinute < 1){  //Min allowable RPM
        rolePerMinute = 1;
      }
      if (rolePerMinute > 17){  //Max allowable RPM
        rolePerMinute = 17;
      }
    }

    if (SwMenu == 2){  //Entered menu1 - sub menu2
      lcd.setCursor(0, 0);
      lcd.print("Nr of turns");
      lcd.setCursor(7, 1);
      lcd.print(TurnNr);

      if (FastChng == 0) {  // RH - if not in fast change mode, update set time
        SetTime = millis();
      }
      
      if (YValue < 400 && Flag4 == 0){ // joystick up -> Add 1 turn
        TurnNr = TurnNr + 1;
        Flag4 = 1;
        FastChng = 1;
        lcd.clear();
      }
      if (YValue > 600 && Flag4 == 0){ // joystick down -> Subtract 1 turn
        TurnNr = TurnNr - 1;
        Flag4 = 1;
        FastChng = 1;
        lcd.clear();
      }
      if (YValue > 399 && YValue < 599 && Flag4 == 1){  //if the Y-axis is back at it's neutral possition -> Flag3 = 0 -> Prevents constant adding or subtracting of 1
        Flag4 = 0;
        FastChng = 0;
      }

      if (YValue < 400 && FastChng == 1) {  // RH - if joystick up is held for > the fast delay time, then enter fast change mode
        if ((millis() - SetTime) > FastDelay) {
          FastChng = 2;
          SetTime = millis();  // update the set time
        }
      }

      if (YValue > 600 && FastChng == 1) {  // RH - if joystick down is held for > the fast delay time, then enter fast change mode
        if ((millis() - SetTime) > FastDelay) {
          FastChng = 2;
          SetTime = millis();  // update the set time
        }
      }

      if (YValue < 400 && FastChng == 2) {  // RH - we are in fast change mode, so we want to update the values quickly, proportional between
        if ((millis() - SetTime) > (LongInt - (400 - YValue) * (LongInt - ShortInt) / 400)) {  // ShortInt and LongInt based on joystick deflection
          TurnNr = TurnNr + 1;
          SetTime = millis();
          lcd.clear();
        }
      }

      if (YValue > 600 && FastChng == 2) {  // RH - same for joystick down in fast change mode
        if ((millis() - SetTime) > (LongInt - (YValue - 600) * (LongInt - ShortInt) / 400)) {
          TurnNr = TurnNr - 1;
          SetTime = millis();
          lcd.clear();
        }
      }
      
      if (TurnNr < 1){  //Min allowable amount of turns
        TurnNr = 1;
      }
      if (TurnNr > 200){  //Max allowable amount of turns
        TurnNr = 200;
      }
    }

    if (SwMenu == 3){  //Program started
      MaxSwMenu = 1;
      StepPoll = 32 * rolePerMinute;  // RH - set polling rate StepPoll = number of steps per second (roughly)
      lcd.setCursor(0, 0);
      lcd.print("Program started");
      lcd.setCursor(0, 1);
      lcd.print("Turns done: ");
      lcd.print(CurrentTurn);

      if (CurrentTurn < TurnNr){ 
        myStepper.setSpeed(rolePerMinute);
        // Start Taking Video Here
        if (!takingVideo)
        {
          pullFocus();
//          startVideo();
          takingVideo = true;
          pulledFocus = true;
          takePhoto();
        }
        Cntr = 0;  // RH - initialize step counter
        while ((Cntr <= FullRev) && (CinCancelFlag == 0)) { 
          myStepper.step(StepPoll);  // RH - breaking up plate rotation into roughly 1 sec intervals to check for cancel
          Cntr = Cntr + StepPoll;
          SwValue = digitalRead(SW_pin);
          if (SwValue == 0 && BtnCancelFlag == 0) {  // RH - check if button is pressed, if so start timing for cancel
            BtnCancelFlag = 1;
            SetTime = millis();
          }
          if (SwValue == 1 && BtnCancelFlag == 1) {  // RH - if button released before BtnDelay, then reset button flag
            BtnCancelFlag = 0;
          }
          if (SwValue == 0 && BtnCancelFlag == 1 && ((millis() - SetTime) > BtnDelay)) {  // RH - button has been held for > BtnDelay, so cancel operation
            CinCancelFlag = 1;
            CurrentTurn = TurnNr;  
          }
        }
        if (CinCancelFlag == 0) {  // RH - operation not cancelled, so continue
          myStepper.step(FullRev + StepPoll - Cntr);  // RH - need to move remainder of steps for 1 full turn due to kickout from while loop
          CurrentTurn = CurrentTurn + 1;
          lcd.setCursor(0, 1);
          lcd.print("Turns done: ");
          lcd.print(CurrentTurn);
        }
      }

      if (CurrentTurn == TurnNr){  //If the current turn is equal to the amount of thurns that need to be turned -> program finished (RH - or cancelled)
        // Stop Taking Video Here
//        stopVideo();
        lcd.setCursor(0, 0);
        if (CinCancelFlag == 0) {
          lcd.print("Program finished");
        }
        else {
          lcd.clear();
          lcd.print("Program cancel");
        }
        delay(3000);
        lcd.clear();  //Reset
        CurrentTurn = 0;
        PhotoNo = 1;
        rolePerMinute = 15;
        SwMenu = 0;
        MaxSwMenu = 0;
        CinCancelFlag = 0;
        Steps = 0;
        takingVideo = false;
        pulledFocus = false;
      }
    }
  }

//***********************************************Menu2***********************************************//

  if (MenuNr == 2){  //Menu2 selected
    
    if (SwMenu == 0){
      lcd.setCursor(0, 0);
      lcd.print("Manual control");
    }

     if (SwMenu == 1){  //Entered menu2
      lcd.setCursor(0, 0);
      lcd.print("motor speed");
      lcd.setCursor(7, 1);
      lcd.print(rolePerMinute);

      if (FastChng == 0) {  // RH - if not in fast change mode, update set time
        SetTime = millis();
      }
      
      if (YValue < 400 && Flag5 == 0){ // joystick up -> Add 1 RPM
        rolePerMinute = rolePerMinute + 1;
        Flag5 = 1;
        FastChng = 1;
        lcd.clear();
      }
      if (YValue > 600 && Flag5 == 0){ // joystick down -> Subtract 1 RPM
        rolePerMinute = rolePerMinute - 1;
        Flag5 = 1;
        FastChng = 1;
        lcd.clear();
      }
      if (YValue > 399 && YValue < 599 && Flag5 == 1){  //if the Y-axis is back at it's neutral possition -> Flag3 = 0 -> Prevents constant adding or subtracting of 1
        Flag5 = 0;
        FastChng = 0;
      }

      if (YValue < 400 && FastChng == 1) {  // RH - if joystick up is held for > the fast delay time, then enter fast change mode
        if ((millis() - SetTime) > FastDelay) {
          FastChng = 2;
          SetTime = millis();  // update the set time
        }
      }

      if (YValue > 600 && FastChng == 1) {  // RH - if joystick down is held for > the fast delay time, then enter fast change mode
        if ((millis() - SetTime) > FastDelay) {
          FastChng = 2;
          SetTime = millis();  // update the set time
        }
      }

      if (YValue < 400 && FastChng == 2) {  // RH - we are in fast change mode, so we want to update the values at the LongInt interval
        if ((millis() - SetTime) > LongInt) {  // (using fixed LongInt since not that many values to cycle through)
          rolePerMinute = rolePerMinute + 1;
          SetTime = millis();
          lcd.clear();
        }
      }

      if (YValue > 600 && FastChng == 2) {  // RH - same for joystick down in fast change mode
        if ((millis() - SetTime) > LongInt) {
          rolePerMinute = rolePerMinute - 1;
          SetTime = millis();
          lcd.clear();
        }
      }
      
      if (rolePerMinute < 1){  //Min allowable RPM
        rolePerMinute = 1;
      }
      if (rolePerMinute > 17){  //Max allowable RPM
        rolePerMinute = 17;
      }

      if (XValue < 400 ){  //if the joystick is pushed to the right side and the neutral flag is 0 then 1 will be added to the menu number (purpose of the flag -> see comment Flag1 above)
        Steps = Steps + 1;
        myStepper.setSpeed(rolePerMinute);
        myStepper.step(Steps);
        lcd.setCursor(14, 1);
        lcd.print("->");
        Flag6 = 1;
      }

      if (XValue > 600 ){  //if the joystick is pushed to the left side and the neutral flag is 0 then 1 will be subtracted from the menu number (purpose of the flag -> see comment Flag1 above)
        Steps = Steps + 1;
        myStepper.setSpeed(rolePerMinute);
        myStepper.step(-Steps);
        lcd.setCursor(0, 1);
        lcd.print("<-");
        Flag6 = 1;
      }

      if (XValue > 399 && XValue < 599){  //if the Y-axis is back at it's neutral possition -> Flag3 = 0 -> Prevents constant adding or subtracting of 1
        Steps = 0;
        if (Flag6 == 1){
          lcd.clear();
          Flag6 = 0;  //This flag is only active during 1 program cycle to clear the lcd
        }
      }
     }

     if (SwMenu == 2){  //Leave menu 2 when button is pressed
       lcd.clear();
       CurrentTurn = 0;
       PhotoNo = 1;
       rolePerMinute = 15;
       SwMenu = 0;
       Steps = 0;
     }
  }
}

// Nikon Camera Routines
void takePhoto()
{
  digitalWrite(3, HIGH);
  digitalWrite(13, HIGH);
  delay(500); // to allow for lag
  digitalWrite(3, LOW);
  digitalWrite(13, LOW);
}

void pullFocus()

{
   digitalWrite(3, HIGH);
   delay(500); // to allow for lag
   digitalWrite(3, LOW);
  }

/*
 * Configuration function of the LCD screen for the progress bar.
 * Uses all custom characters from 0 to 8.
 */
void setup_progressbar()
{
  // Stores custom characters in the LCD screen memory
  lcd.createChar(0, START_DIV_4_OF_4);
  lcd.createChar(1, DIV_0_OF_8);
  lcd.createChar(2, DIV_8_OF_8);
  lcd.createChar(3, END_DIV_0_OF_4);
  // The other characters are configured dynamically
}

/*
 * Dynamic configuration function of the LCD screen for the progress bar.
 *
 * @param bank The number of the character bank to configure.
 */
void switch_progressbar_bank(byte bank)
{
  // IMPORTANT: It is necessary to do a lcd.clear () or an lcd.setCursor () after each change of bank.
  // Change character bank
  switch (bank)
  {
  case 0:
    lcd.createChar(4, START_DIV_0_OF_4);
    lcd.createChar(5, START_DIV_1_OF_4);
    lcd.createChar(6, START_DIV_2_OF_4);
    lcd.createChar(7, START_DIV_3_OF_4);
    break;

  case 1:
    lcd.createChar(4, DIV_1_OF_8);
    lcd.createChar(5, DIV_2_OF_8);
    lcd.createChar(6, DIV_3_OF_8);
    lcd.createChar(7, DIV_4_OF_8);
    break;

  case 2:
    lcd.createChar(4, DIV_4_OF_8);
    lcd.createChar(5, DIV_5_OF_8);
    lcd.createChar(6, DIV_6_OF_8);
    lcd.createChar(7, DIV_7_OF_8);
    break;

  case 3:
    lcd.createChar(4, END_DIV_1_OF_4);
    lcd.createChar(5, END_DIV_2_OF_4);
    lcd.createChar(6, END_DIV_3_OF_4);
    lcd.createChar(7, END_DIV_4_OF_4);
    break;
  }
}

/**Function drawing the progress bar. **@param displayFigure Whether to show percentage as numeric on line 0 **@param percent The percentage to display.*/
void draw_progressbar(bool displayFigure, byte percent)
{
  // Optionally displays the new value in numerical form on the first line */
  if (displayFigure)
  {
    lcd.home();
    lcd.print(percent);
    lcd.print(F("%"));
    // NB The two spaces at the end of the line allow you to delete the percentage figures
    // previous when going from a two or three digit value to a two or three digit value.
  }

  if (percent > 100)
  {
    percent = 100;
  } // stop erroneous displays at over 100%
  // Move the cursor to the second line */
  lcd.setCursor(0, 1);

  // Map the range(0 ~100) to the range(0 ~LCD_NB_COLUMNS * 2 * 4 - 2 * 4) */
  byte nb_columns = map(percent, 0, 100, 0, LCD_NB_COLUMNS * 2 * 4 - 2 * 4);
  // Each character displays 2 vertical bars 4 pixels high, but the first and last character displays only one.

  // Draw each character in the line */
  for (byte i = 0; i < LCD_NB_COLUMNS; ++i)
  {

    if (i == 0)
    { // First box

      //Displays the starting char according to the number of columns */
      if (nb_columns > 4)
      {
        lcd.write((byte)0); // Tank start 4/4
        nb_columns -= 4;
      }
      else if (nb_columns == 4)
      {
        lcd.write((byte)0); // Tank start 4/4
        nb_columns = 0;
      }
      else
      {
        switch_progressbar_bank(0);
        lcd.setCursor(i, 1);
        lcd.write(nb_columns + 4); // Start tank N / 4
        nb_columns = 0;
      }
    }
    else if (i == LCD_NB_COLUMNS - 1)
    { // Last box

      // Displays the end char according to the number of columns */
      if (nb_columns > 0)
      {
        switch_progressbar_bank(3);
        lcd.setCursor(i, 1);
        lcd.write(nb_columns + 3); // Fine tank N / 4
      }
      else
      {
        lcd.write(3); // Fine tank 0/4
      }
    }
    else
    { // Other boxes

      // Displays the correct char according to the number of columns */
      if (nb_columns == 0)
      {
        lcd.write(1); // Char div 0/8
      }
      else if (nb_columns >= 8)
      {
        lcd.write(2); // Char div 8/8
        nb_columns -= 8;
      }
      else if (nb_columns >= 4 && nb_columns < 8)
      {
        switch_progressbar_bank(2);
        lcd.setCursor(i, 1);
        lcd.write(nb_columns); // Char div N / 8
        nb_columns = 0;
      }
      else if (nb_columns < 4)
      {
        switch_progressbar_bank(1);
        lcd.setCursor(i, 1);
        lcd.write(nb_columns + 3); // Char div N / 8
        nb_columns = 0;
      }
    }
  }
}    
