// Rotor controller, keyboard command 
// Written for SK2GJ UHF/VHF antennas
// Carl-Fredrik Enell SM2YHP 2022
// fredrik@i-kiruna.se

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <FiniteStateMachine.h>

// Pin configuration
#define BRAKE_OUT 2
#define CCW_OUT 4
#define CW_OUT 7
#define DOWN_OUT 8
#define UP_OUT 12

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


int wantedState;

void setup() {
  // Prepare outputs
  pinMode(CCW_OUT, OUTPUT);
  pinMode(CW_OUT, OUTPUT);
  pinMode(DOWN_OUT, OUTPUT);
  pinMode(UP_OUT, OUTPUT);
  pinMode(BRAKE_OUT, OUTPUT);
  // Prepare LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("Off");
  // Start serial communication 
  Serial.begin(9600);
}

// Main program
void loop() {
  if (wantedState==108) {
    if (Azimuth.isInState(STOP_AZ)) {
      Azimuth.transitionTo(CCW);
      lcd.clear();
      lcd.print("Turning CCW");
    }
  }
  else if (wantedState==114) {
    if (Azimuth.isInState(STOP_AZ)) {
      Azimuth.transitionTo(CW);
      lcd.clear();
      lcd.print("Turning CW");
    }
  }
  else {
    if (Azimuth.isInState(CCW) || Azimuth.isInState(CW)) {
      Azimuth.transitionTo(STOP_AZ);
      lcd.clear();
      lcd.print("Off");
    }
  } 
  if (wantedState==100) {
    if (Elevation.isInState(STOP_EL)) {
      Elevation.transitionTo(DOWN);
      lcd.clear();
      lcd.print("Moving down");
    }
  }
  else if (wantedState==117) {
    if (Elevation.isInState(STOP_EL)) {
      Elevation.transitionTo(UP);
      lcd.clear();
      lcd.print("Moving up");
    }
  }
  else {
    if (Elevation.isInState(DOWN) || Elevation.isInState(UP)) {
      Elevation.transitionTo(STOP_EL);
      lcd.clear();
      lcd.print("Off");
    }
  }
  // Run the state machines
  Azimuth.update();
  Elevation.update();  
}// End of main loop

                                                                                                                                                                      
// Parse serial input
void serialEvent(){
  while(Serial.available()){
    wantedState=Serial.read();
  } // end while
} // end serialEvent

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
}

void startCW(){
  if(Brake.isInState(BRAKE_ON)){
    Brake.transitionTo(BRAKE_OFF);
    Brake.update();
    delay(100);
  }
  digitalWrite(CW_OUT, LOW);
}

void startDown(){
  // On inverted logic
  digitalWrite(DOWN_OUT, LOW);
}

void startUp(){
  // On inverted logic
  digitalWrite(UP_OUT, LOW);
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
