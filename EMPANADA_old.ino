 // EMPANADA SOFTWARE
 // MAKE SURE MAIN POWER IS NOT PLUGGED IN AND THE SD CARD IS INSERTED BEFORE POWERING

#include <AccelStepper.h>
#include <Adafruit_RGBLCDShield.h>
#include <EEPROM.h>
#include <HX711.h>
//#include <WheatstoneBridge.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <utility/Adafruit_MCP23017.h>
#include "DHT.h"
#include "SD.h"
#include "RTClib.h"

 // initialize SD card
RTC_PCF8523 RTC;
#define LOG_INTERVAL 1000 
#define ECHO_TO_SERIAL 0
#define WAIT_TO_START 0
const int chipSelect = 10; 
File logfile;
 char filename[] = "LOGGER00.CSV";
 
 // initialize LCD
Adafruit_RGBLCDShield lcd = Adafruit_RGBLCDShield();
#define TEAL 0x6

 // initialize sensors
#define DHTPIN 2     
#define DHTTYPE DHT22   
Adafruit_LIS3DH lis = Adafruit_LIS3DH();
DHT dht(DHTPIN, DHTTYPE);

 // Initial calibration values
const int CST_STRAIN_IN_MIN = 350;       // Raw value calibration lower point
const int CST_STRAIN_IN_MAX = 650;       // Raw value calibration upper point
const int CST_STRAIN_OUT_MIN = 0;        // Weight calibration lower point
const int CST_STRAIN_OUT_MAX = 1000;     // Weight calibration upper point

const int CST_CAL_FORCE_MIN = 0;
const int CST_CAL_FORCE_MAX = 32000;
const int CST_CAL_FORCE_STEP = 50;
const int CST_CAL_FORCE_STEP_LARGE = 500;

 // Initialize the load cell
WheatstoneBridge wsb(A1, CST_STRAIN_IN_MIN, CST_STRAIN_IN_MAX, CST_STRAIN_OUT_MIN, CST_STRAIN_OUT_MAX);

 // define variables for motor
AccelStepper stepper1 (AccelStepper::FULL4WIRE, 6,7,8,9, true);
const int stepsPerRevolution = 200;
int steps = -600;
int speedset = 0;

 // define pins
const int button = 17;
const int shutter = 49;

 // define variables for button
int buttonState = 0;         
int lastButtonState = 0;     
int buttonPushCounter = EEPROM.read(0);

 // timing variables
float t1;
float t2;
float tm;
float t_up1;
float t_up2;
float interval;
const int pause = 5000;

 // iterables
int j = 0;
int k = 0;

void setup() {

  EEPROM.write(0,0);
  
  pinMode(button, INPUT);
  pinMode(shutter, OUTPUT);
  pinMode(chipSelect, OUTPUT);

   // start up SD card file
  Serial.begin(9600);
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
  }
  char filename[] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
      filename[6] = i/10 + '0';
      filename[7] = i%10 + '0';
      if (! SD.exists(filename)) {
        logfile = SD.open(filename, FILE_WRITE); 
      break;
     }
  }

   // start up LCD screen
  lcd.begin(16, 2); 
  lcd.setBacklight(TEAL);
   
   // calibration of load cell
  int cal_adc_low = CST_STRAIN_IN_MIN;
  int cal_adc_high = CST_STRAIN_IN_MAX;
  int cal_force_low = CST_STRAIN_OUT_MIN;
  int cal_force_high = CST_STRAIN_OUT_MAX;
  
  wsb.linearCalibration(cal_adc_low, cal_adc_high, cal_force_low, cal_force_high);
  
   // start up accelerometer
  Wire.begin();  
  if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
   while (1);
  }
  lis.setRange(LIS3DH_RANGE_4_G);

   // start up temp and humidity
  dht.begin();
  
   // initialize motor
  stepper1.setMaxSpeed(300);
  stepper1.setCurrentPosition(0);
  
}

void loop() {
  buttonState = digitalRead(button);
  
  uint8_t buttons = lcd.readButtons();
  
  if (buttons) {
    if (buttons & BUTTON_LEFT) {
      buttonPushCounter = buttonPushCounter - 1;
      delay(300);
      }
    if (buttons & BUTTON_DOWN) {
      buttonPushCounter = 0;
      delay(300);
      }
    if (buttons & BUTTON_RIGHT) {
      buttonPushCounter = buttonPushCounter + 1;
      delay(300);
      }
    if (buttons & BUTTON_UP) {
      t_up1 = millis();
      t_up2 = millis();
      while (abs(t_up1-t_up2) < 3000)  {
        t_up1 = millis();
        stepper1.setSpeed(230); 
        stepper1.moveTo(700);
        stepper1.runSpeedToPosition();
       }
        stepper1.setCurrentPosition(0);
      }
     }
    
  if (buttonState != lastButtonState) { 
    if (buttonState == HIGH) {
        
        buttonPushCounter++;
        EEPROM.write(0,buttonPushCounter); 
        
         // start recording
        digitalWrite(shutter,HIGH);
        delay(50);
        digitalWrite(shutter,LOW);
        delay(1500);

         // Read temperature and humidity
        float h = dht.readHumidity();
        float f = dht.readTemperature(true);

        float x[170];
        float y[170];
        float z[170];
        float strain_force[170];
        float strain_adc[170];
        
        lcd.setCursor(0,1);
        lcd.print(buttonPushCounter); 
        
        steps = -600;
        
        // Martian
        if (buttonPushCounter == 1){
          speedset = 70;
        }
        if (buttonPushCounter == 2){
          speedset = 210;
        }
        if (buttonPushCounter == 3){
          speedset = 130;
        }
        
        // Lunar
        if (buttonPushCounter == 4){
          speedset = 70;
        }
         if (buttonPushCounter == 5){
          speedset = 210;
        }
        
        // Zero 
        if (buttonPushCounter == 6 or buttonPushCounter == 16 or buttonPushCounter == 26){
          speedset = 70;
        }
        if (buttonPushCounter == 7 or buttonPushCounter == 17 or buttonPushCounter == 27){
          speedset = 210;
        }
        if (buttonPushCounter == 8 or buttonPushCounter == 18 or buttonPushCounter == 28){
          speedset = 150;
        }
        if (buttonPushCounter == 9 or buttonPushCounter == 19 or buttonPushCounter == 29){
          speedset = 130;
        }
        if (buttonPushCounter == 10 or buttonPushCounter == 20 or buttonPushCounter == 30){
          speedset = 170;
        }
        if (buttonPushCounter == 11 or buttonPushCounter == 21 or buttonPushCounter == 31){
          speedset = 90;
        }
        if (buttonPushCounter == 12 or buttonPushCounter == 22 or buttonPushCounter == 32){
          speedset = 230;
        }
        if (buttonPushCounter == 13 or buttonPushCounter == 23 or buttonPushCounter == 33){
          speedset = 110;
        }
        if (buttonPushCounter == 14 or buttonPushCounter == 24 or buttonPushCounter == 34){
          speedset = 190;
        }
        if (buttonPushCounter == 15 or buttonPushCounter == 25 or buttonPushCounter == 35){
          speedset = 210;
        }
        
        interval = (pause + (steps/speedset)*-1000 + (steps/230)*-1000);
        
        t1 = millis();
        t2 = millis();
        j = 0;
        
        while (abs(t1-t2) < interval+1000){
          t2 = millis();
          tm = t2-t1;
          
          stepper1.setSpeed(speedset); 
          stepper1.moveTo(steps);
          stepper1.runSpeedToPosition();
          
          if (tm > (interval - (steps/230)*-1000)){
            steps = 0;
            speedset = 230;
          }
          
          if (int(tm) % 100 == 0){
            strain_force[j] = wsb.measureForce();
            strain_adc[j] = wsb.getLastForceRawADC();
         
            lis.read();
            sensors_event_t event; 
            lis.getEvent(&event);
            x[j] = event.acceleration.x;
            y[j] = event.acceleration.y;
            z[j] = event.acceleration.z;
            
            j = j+1;
          }
        
        }
        // stop recording
      digitalWrite(shutter,HIGH);
      delay(50);
      digitalWrite(shutter,LOW);
      
      logfile = SD.open(filename, FILE_WRITE);
      logfile.println("");
      logfile.println(buttonPushCounter);
      logfile.print("Humidity: ");
      logfile.print(h);
      logfile.print(" %\t"); logfile.print(", ");
      logfile.print("Temperature: ");
      logfile.print(f);
      logfile.println(" *F\t");
      logfile.print("X Acc: "); logfile.print("  Y Acc:  "); logfile.print("  Z Acc: "); logfile.print(" Raw ADC: "); logfile.println(" Strain Force: ");
      
      k = 0;
      while (k<j){
        logfile.print(x[k]); logfile.print("    ");
        logfile.print(y[k]); logfile.print("    ");
        logfile.print(z[k]); logfile.print("    ");
        logfile.print(strain_adc[k]); logfile.print("    ");
        logfile.print(strain_force[k]); logfile.println("    ");
        k = k+1;
      }
      
      logfile.close();
      
      }
    }
  lastButtonState = buttonState;

   // constant display of acc and buttoncount
  lis.read();
  sensors_event_t event; 
  lis.getEvent(&event);

  lcd.setCursor(0,0);
  lcd.print(event.acceleration.x);
  lcd.setCursor(5,0);
  lcd.print(event.acceleration.y);
  lcd.setCursor(10,0);
  lcd.print(event.acceleration.z);
  lcd.setCursor(0,1);
  lcd.print(buttonPushCounter);
  
 }
