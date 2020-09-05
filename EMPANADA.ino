// EMPANADA SOFTWARE
// MAKE SURE MAIN POWER IS NOT PLUGGED IN AND THE SD CARD IS INSERTED BEFORE POWERING

// May require  sudo chmod 666 /dev/ttyACM0 to get permissions
// to write to Arduino (on my personal laptop, though it is a mess, to be fair)

// Arduino IDE also requires Java >8, so make sure this is correct, otherwise the Serial Monitor may not work

// This allows us to use the stepper motor
// Installed from IDE Library interface
#include <AccelStepper.h>

// This allows us to write to the lcd screen very easily
// Installed from: https://github.com/adafruit/Adafruit-RGB-LCD-Shield-Library
#include <Adafruit_RGBLCDShield.h>

// This allows us to use the strain gauge cell
// Installed from: https://www.robotshop.com/en/strain-gauge-load-cell-amplifier-shield-2ch.html
#include <WheatstoneBridge.h>

// Not sure about these yet
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <utility/Adafruit_MCP23017.h>

// This allows us to use the clock on the RST board (where the SD card goes)
// Installed from IDE Library interface
#include <RTClib.h>

// This allows us to write to the SD card on the RST board
// Installed from IDE Library interface
#include <SD.h>


//////////////////////////
// VARIABLE DEFINITIONS //
//////////////////////////
// For pins, see PIN SETUP section

// General program attributes

// The version of the program
// This should never be more than 6 characters to properly fit on the screen
const String version = "1.0";

// LCD Screen
// The brightness of the text can be adjusted using the potentiometer
// next to the reset button btw
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();

// SD Card
RTC_PCF8523 rtc; // Define the real time clock variable
const int LOG_INTERVAL = 1000;
const int ECHO_TO_SERIAL = 0;
const int WAIT_TO_START = 0;
// This is the actual file object that we will use to write
File logFile;
// We'll define this later, since we have to make sure we're not overwriting another log
String logFileName;

// Strain gauge cell initial calibration values
// I am not sure what the units on any of these values are, but I believe they are taken
// from the examples available for the Wheatstone bridge library
const int CST_STRAIN_IN_MIN = 350;       // Raw value calibration lower point
const int CST_STRAIN_IN_MAX = 650;       // Raw value calibration upper point
const int CST_STRAIN_OUT_MIN = 0;        // Weight calibration lower point
const int CST_STRAIN_OUT_MAX = 1000;     // Weight calibration upper point

// Initialize the load cell here
// This means we don't need to use the `linearCalibration` method in setup()
// A0 is the port that the cell is running on and can be any of the following values: A0, A1, A2, A3
WheatstoneBridge wsb(A0, CST_STRAIN_IN_MIN, CST_STRAIN_IN_MAX, CST_STRAIN_OUT_MIN, CST_STRAIN_OUT_MAX);

// This is the number of samples we average for the force measurement, since it
// would be too much to take a measurement on each step (would look very noisy)
int forceMeasurementAverageResolution = 100;


// Setup the stepper motor through the H-bridge
// We use the wiring setup (colors of wires are in parenthesis):
// H-Bridge   |   Arduino
//    IN1             6 (Black)
//    IN2             7 (White)
//    IN3             8 (Gray)
//    IN4             9 (Purple)
AccelStepper stepper(AccelStepper::FULL4WIRE, 6, 7, 8, 9, true);

// Not sure what this variable is TODO
const int stepsPerRevolution = 200;
// This value is going need some explaining, but I would like to say first that
// we insert the probe at constant speed, just to be completely clear.
// The reason we need an acceleration is because the stepper library has four functions for
// stepping the motor, two of which are intended to be used at constant speed, which is what
// we would like. Unfortunately, these aren't cooperating with me, so my solution is to 
// use the stepper with acceleration/decceleration, but to have that value be so high, that
// the stepper actually does move at constant speed. Kinda hacky, but it works
// For this reason, the speed that the stepper runs at can be set using the
// setMaxSpeed() function NOT the setSpeed() function.
// I have tested this with serveral speeds and the speed has been constant the entire time, even
// on the first/last steps, so this shouldn't be an issue
const int stepperAcceleration = 50000; // Arbitrary, other than being much larger than stepperSpeed

// Set the initial value of the stepper to be 0, but this will be calibrated later
int stepperPosition = 0;
int stepperSpeed = 5; // Default speed is 5 mm/s

// The amount to change the speed each button press
const int speedChangeIncrement = 1;

// The number of steps to descend when the down button is pushed
const int numStepsToDescend = 690;

// We want our results to be in reasonable units, not steps/some unit time, so we convert to mm/s using
// this value. I found this value experimentally by measuring how long it takes the cart to travel a
// given distance. For more info, see my LabBlog post on May 30, 2020
// When setting the speed, do: setSpeed(int(someValueInMMS / stepToSpeedConversion));
const float stepToSpeedConversion = .1710f;

// How fast the stepper moves up during the reset routine
const int resetRoutineStepperSpeed = int(10 / stepToSpeedConversion); // 8 mm/s
const int resetRoutineDescendPosition = -10; //  Descend 25 steps from the top

//////////////////////////
//      PIN SETUP       //
//////////////////////////

// This is the pin to look for the SD card on
const int CHIP_SELECT = 10;

// The pins for the H-bridge (and stepper motor) are 6, 7, 8, 9, but aren't defined as variables

const int RESET_ROUTINE_BUTTON = 26;

void setup() {
  Serial.begin(9600);
  Serial.println("Start");

  pinMode(CHIP_SELECT, OUTPUT);
  pinMode(RESET_ROUTINE_BUTTON, INPUT);
  // start up LCD by providing the dimensions of the screen
  lcd.begin(16, 2);

  // Set the date/time of the real time clock to be that of the
  // connected computer. This doesn't matter too much, since it
  // will likely be reset upon losing power, and we only care about
  // time differences, not the absolute time. More or less arbitrary
  if (!rtc.initialized() || rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // Now start up the SD Card reader/writer
  if (!SD.begin(CHIP_SELECT)) {
    Serial.println("SD Card initialization failed, check CHIP_SELECT value, as well as SD Card formatting (must be FAT32/FAT16)");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Error:");
    lcd.setCursor(0, 1);
    lcd.print("Check SD card");
    delay(3000);
  }

  // Unlike the previous iteration of this program, the log file is now determined
  // when the button is pressed, so that each trial is recorded to a separate file
  
  // Setup the motor
  // First we set the current motor position to be zero, but this will end up
  // being adjusted later, so that the probe starts at the same point every trial
  stepper.setCurrentPosition(stepperPosition);
  // See the note in the definition of stepper about speeds/accelerations
  stepper.setAcceleration(stepperAcceleration);
  stepper.setMaxSpeed(int(stepperSpeed / stepToSpeedConversion));

}

void loop() {
  
  // We can specify where to start printing text with setCursor
  lcd.setCursor(0, 0); // (Column, Row), (x, y)
  lcd.print("Speed selection:");
  lcd.setCursor(0, 1);
  lcd.print(" <  " + String(stepperSpeed) + " mm/s  >");
  
  uint8_t buttons = lcd.readButtons();

  // Check if any buttons are pressed:
  // We use the up and down buttons to start the stepper moving up or down
  // and the left and right to select the speed.
  // The select button is used to run the reset routine
  if (buttons) {

    // Run the reset routine
    if (buttons & BUTTON_UP) {
      // First we raise the cart until it clicks the button at the top
      int stopButtonState = 0;
      stepper.setMaxSpeed(resetRoutineStepperSpeed);

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Reset routine:");
      lcd.setCursor(0, 1);
      lcd.print("Finding top");
      
      while (stopButtonState != HIGH) {
        stopButtonState = digitalRead(RESET_ROUTINE_BUTTON);

        // Move up a little
        stepper.move(1);
        stepper.runToPosition();
      }

      // Reset the position
      stepperPosition = 0;
      stepper.setCurrentPosition(0);

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Reset routine:");
      lcd.setCursor(0, 1);
      lcd.print("Descending");
      
      // Now move back down
      stepper.runToNewPosition(resetRoutineDescendPosition);
      stepperPosition = resetRoutineDescendPosition;

      // Put the speed back to what it was
      stepper.setMaxSpeed(int(stepperSpeed / stepToSpeedConversion));

      // Show a little completion message
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Reset complete");
      delay(1000);
      lcd.clear();
    }

    // Make the probe descend
    if (buttons & BUTTON_DOWN) {

        // First make sure that the SD card is inserted and all set to go
        if (!SD.begin(CHIP_SELECT)) {
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Error:");
          lcd.setCursor(0, 1);
          lcd.print("Check SD card");
          delay(3000);
          return;
        }
        // Setup the log file we will be writing the force data to
        // Has the format <speed>mms-<number>.CSV where number will start at 0 and move up

        bool foundLogFile = false;
        // Only goes up to 100 since have 100 log files is already way too many
        for (int i = 0; i < 100; i++) {
          String logNum = String(i);
          // Format the number properly
          if (logNum.length() == 1)
            logNum = "0" + logNum;
      
          // Put it all together
          logFileName = String(stepperSpeed) + "mms-" + logNum + ".CSV";
          
          // Now check if this file already exists
          if (!SD.exists(logFileName)) {
            logFile = SD.open(logFileName, FILE_WRITE);
            foundLogFile = true;
            break;
          }
        }

        if (!foundLogFile) {
          Serial.println("Too many log files, please delete some!");
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Error:");
          lcd.setCursor(0, 1);
          lcd.print("Too many logs");
          delay(3000);
          return;
        }



        // Assuming we found the log, we set the target position      
        stepperPosition -= numStepsToDescend;
        stepper.moveTo(stepperPosition);
        
        // Display a message on the LCD
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Descending...");
    }
    
    // Decrease the speed
    if (buttons & BUTTON_LEFT) {
      stepperSpeed -= speedChangeIncrement;
      stepper.setMaxSpeed(int(stepperSpeed / stepToSpeedConversion));
      lcd.clear();
    }

    // Increase the speed
    if (buttons & BUTTON_RIGHT) {
      stepperSpeed += speedChangeIncrement;
      stepper.setMaxSpeed(int(stepperSpeed / stepToSpeedConversion));
      lcd.clear();
    }

    // Show the help menu
    if (buttons & BUTTON_SELECT) {
     
      // Show version
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("EMPANADA");
      lcd.setCursor(0, 1);
      lcd.print("Firmware");
      lcd.setCursor(15 - version.length(), 1); // 15 since the display has 16 pixels - 1 for the "v"
      lcd.print("v" + version);

      delay(5000);

      // Show controls
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("UP-Reset routine");
      lcd.setCursor(0, 1);
      lcd.print("DOWN-Descend");

      delay(5000);

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("LEFT-speed -");
      lcd.setCursor(0, 1);
      lcd.print("RIGHT-speed +");

      delay(5000);
    }
  }

  bool needToCloseLogFile = false;

  if (stepper.targetPosition() != stepper.currentPosition()) {
    // Keep track of what step we are on, and define an array that we 
    // can average over to find the force
    uint8_t stepCount = 0;
    long averageForceTempArr[forceMeasurementAverageResolution];
    // Keep track of when the start time was
    long startTime = millis();

    // Since we are averaging, we want the time we record to be at the
    // center of the averaging samples, so we need to keep track of
    // when the last average was
    long lastAverageTime = 0;
  
    // Loop until we reach the target destination
    while (stepper.targetPosition() != stepper.currentPosition()) {
      // Have the motor take a step at the constant speed we have defined before
      stepper.run();
      // Read the force from the wheatstone bridge
      averageForceTempArr[stepCount % forceMeasurementAverageResolution] = wsb.measureForce();
      
      // We don't want to take a force measurement every step, so we downsample a bit
      if (stepCount % forceMeasurementAverageResolution == 0 && stepCount > 0) {
        // Calculate the average time
        long currentAverageTime =  (millis() - startTime) - ((millis() - startTime) - lastAverageTime) / 2;
        // Write the time,force
        logFile.println(String(float(currentAverageTime) / 1000.f) + ", " + String(average(averageForceTempArr, forceMeasurementAverageResolution)));
        lastAverageTime = currentAverageTime;
        //Serial.println(average(averageForceTempArr, forceMeasurementAverageResolution));
        //Serial.println(stepper.speed());
        
      }
  
      // Increment the step count
      stepCount++;
    }
    
    // Close the log file, which should flush it as well
    logFile.close();
  }

 }

// This function is used to take the average of the force sensor readings, since measuring
// the force every step would be a little too many points
long average(long arr[], int l) {
  long sum = 0;
  for (int i = 0; i < l; i++) {
    sum += arr[i];
  }
  return sum / long(l);
 }
