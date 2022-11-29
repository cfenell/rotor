// Rotor controller for satellite tracking
// Written for SK2GJ UHF/VHF antennas
// Carl-Fredrik Enell SM2YHP 2022
// fredrik@i-kiruna.se

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <FiniteStateMachine.h>
#include <SimpleKalmanFilter.h>

// Pin configuration
#define AZ_SENS A0
#define EL_SENS A1
#define BRAKE_OUT 2
#define CCW_OUT 4
#define CW_OUT 7
#define DOWN_OUT 8
#define UP_OUT 12

// Angle calibrations
#define AZOFFSET 0.0
#define AZREADMIN 0.0
#define AZREADMAX 1019.0
#define ELOFFSET 90.0
#define ELREADMIN 0.0
#define ELREADMAX 1021.0
#define TOL 2.0

// 2x16 I2C LCD, default address 0x27
LiquidCrystal_I2C lcd(0x27,16,2);

// Motor states
State CW = State(startCW);
State CCW = State(startCCW);
State DOWN = State(startDown);
State UP = State(startUp);                                                                                                                                            
State STOP_AZ = State(stopAz);
State STOP_EL = State(stopEl); 
State BRAKE_ON = State(brakeOn);
State BRAKE_OFF = State(brakeOff);

// Initiate FSMs
FSM Azimuth = FSM(STOP_AZ);
FSM Brake = FSM(BRAKE_ON);
FSM Elevation = FSM(STOP_EL);

// Set up filter
SimpleKalmanFilter AzFilt(1, 1, 0.01);
SimpleKalmanFilter ElFilt(1, 1, 0.01);

// Global variables
float AzRead;  // Read azimuth AD counts
float AzCalc; // Corrected azimuth
float AzSet;  // Set azimuth 
float ElRead;
float ElCalc;
float ElSet;
// To keep track of motor run times
unsigned long azRun=0;
unsigned long elRun=0;
// Serial in
char inChar=(char)"*";
char prevChar=(char)"*";
String numStr="";
// Text output
char lcdOut[17];
char AzReadOut[5];
char AzCalcOut[4];
char AzSetOut[4];
char ElReadOut[5];
char ElCalcOut[4];
char ElSetOut[4];

// Angle calibrations
float CtoA(float counts) {
  float az;
  az=360.0*(counts-AZREADMIN)/AZREADMAX;
  az-=AZOFFSET;
  if(az<0) az+=360.0;
  if(az>360) az-=360.0;   
  return az;
}

float CtoE(float counts) {
  float el;
  el=180.0*(counts-ELREADMIN)/ELREADMAX;
  el-=ELOFFSET;
  return el;
}


void setup() {
  // Prepare outputs
  pinMode(CCW_OUT, OUTPUT);
  pinMode(CW_OUT, OUTPUT);
  pinMode(DOWN_OUT, OUTPUT);
  pinMode(UP_OUT, OUTPUT);
  pinMode(BRAKE_OUT, OUTPUT);
  // Init angles to current at startup
  AzCalc=CtoA(analogRead(AZ_SENS));
  AzSet=AzCalc;
  ElCalc=CtoE(analogRead(EL_SENS));
  ElSet=ElCalc;
  // Prepare LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  // Start serial communication 
  Serial.begin(9600);
  numStr.reserve(8);
  // Set up timer 2
  TIMSK2 = (TIMSK2 & B11111110) | 0x01; //Enable overflow interrupt
  TCCR2B = (TCCR2B & B11111000) | 0x07; //Divisor 1024 -> Overflow interval ~16 ms
}

// Main program
void loop() {
  // ADC to angles
  AzCalc=CtoA(AzRead);
  ElCalc=CtoE(ElRead);
  // Compare set and measured angles
  if(AzCalc-AzSet > TOL) {
    if(Azimuth.isInState(STOP_AZ)) Azimuth.transitionTo(CCW);
  }
  else if(AzCalc-AzSet < -TOL) {
    if(Azimuth.isInState(STOP_AZ)) Azimuth.transitionTo(CW);
  }
  else if(abs(AzCalc-AzSet)<=TOL) {
    if(Azimuth.isInState(CCW) || Azimuth.isInState(CW)) Azimuth.transitionTo(STOP_AZ);
  }
  if(ElCalc-ElSet > TOL) {
    if(Elevation.isInState(STOP_EL)) Elevation.transitionTo(DOWN);
  }
  else if(ElCalc-ElSet < -TOL) {
    if(Elevation.isInState(STOP_EL)) Elevation.transitionTo(UP);
  }
  else if (abs(ElCalc-ElSet)<=TOL) {
    if(Elevation.isInState(DOWN) || Elevation.isInState(UP)) Elevation.transitionTo(STOP_EL);
  }
  // Run the state machines
  Azimuth.update();
  Elevation.update();

  // Safety check that motors have not been running too long
  if(!Azimuth.isInState(STOP_AZ)){
    if(millis()-azRun > 60000){
      Azimuth.transitionTo(STOP_AZ);
      Azimuth.update();
      lcd.clear();
      lcd.print("* AZ PANIC OFF *");
    }
  }
  if(!Elevation.isInState(STOP_EL)) {
    if(millis()-elRun > 60000){
      Elevation.transitionTo(STOP_EL);
      Elevation.update();
      lcd.clear();
      lcd.print("* EL PANIC OFF *");
    }
  }
  
  // Update the display
  dtostrf(AzRead,4,0,AzReadOut);
  dtostrf(AzCalc,3,0,AzCalcOut);
  dtostrf(AzSet,3,0,AzSetOut);
  sprintf(lcdOut, "AC%s A%s S%s",AzReadOut,AzCalcOut,AzSetOut);
  lcd.setCursor(0,0);
  lcd.print(lcdOut);
  dtostrf(ElRead,4,0,ElReadOut);
  dtostrf(ElCalc,3,0,ElCalcOut);
  dtostrf(ElSet,3,0,ElSetOut);
  sprintf(lcdOut,"EC%s E%s S%s",ElReadOut,ElCalcOut,ElSetOut);
  lcd.setCursor(0,1);
  lcd.print(lcdOut);
}// End of main loop

// Read pots at timer interrupt
ISR(TIMER2_OVF_vect) 
{ 
  int val;
  // Read angle
  val=analogRead(AZ_SENS); // 0-5V divider
  AzRead=AzFilt.updateEstimate((float)val); 
  val=analogRead(EL_SENS); // 0-5 V divider
  ElRead=ElFilt.updateEstimate((float)val); 
}
                                                                                                                                                                      
// Parse serial input
void serialEvent(){
  while(Serial.available()){
    inChar=(char)Serial.read();
    if(prevChar=='A' and inChar=='Z'){
      // Azimuth change request
      numStr="";
      while(Serial.available()) {
         inChar = (char)Serial.read();
         numStr+=inChar;
         if(inChar=='.'){
           //Protocol mandates exactly one decimal place
           inChar = (char)Serial.read();
           numStr+=inChar;
           AzSet=numStr.toFloat();
           if(AzSet<0.0) AzSet+=360.0;
           if(AzSet>360.0) AzSet-=360.0;
           break;
         }
      }  
    }
    if(prevChar=='E' and inChar=='L'){
      // Elevation change request
      numStr="";
      while(Serial.available()) {
         inChar = (char)Serial.read();
         numStr+=inChar;
         if(inChar=='.'){
           inChar = (char)Serial.read();
           numStr+=inChar;
           ElSet=numStr.toFloat();
           if(ElSet<0.0) ElSet=0.0;
           if(ElSet>90.0) ElSet=90.0;
           break;
         }
      }  
    }
    prevChar=inChar;
  } // end while
} //end serialEvent

// State functions
void startCCW(){
  // Release the brake
  if(Brake.isInState(BRAKE_ON)){
    Brake.transitionTo(BRAKE_OFF);
    Brake.update();
    delay(500);
  }
  // On inverted logic
  digitalWrite(CCW_OUT, LOW);
  // Keep track of run time
  azRun=millis();
}

void startCW(){
  if(Brake.isInState(BRAKE_ON)){
    Brake.transitionTo(BRAKE_OFF);
    Brake.update();
    delay(100);
  }
  digitalWrite(CW_OUT, LOW);
  azRun=millis();
}

void startDown(){
  // On inverted logic
  digitalWrite(DOWN_OUT, LOW);
  elRun=millis();
}

void startUp(){
  // On inverted logic
  digitalWrite(UP_OUT, LOW);
  elRun=millis();
}
                          
void stopAz(){
  // Off inverted logic
  digitalWrite(CCW_OUT, HIGH);
  digitalWrite(CW_OUT, HIGH);
  // Activate brake
  if(Brake.isInState(BRAKE_OFF)){
    delay(500);
    Brake.transitionTo(BRAKE_ON);
    Brake.update();
  }
}

void stopEl(){
  digitalWrite(DOWN_OUT, HIGH);
  digitalWrite(UP_OUT, HIGH);
}

void brakeOn(){
  digitalWrite(BRAKE_OUT, HIGH); //Relay -solenoid off
}

void brakeOff(){
  digitalWrite(BRAKE_OUT, LOW); //Solenoid on releases brake
}
