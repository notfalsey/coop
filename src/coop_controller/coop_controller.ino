#include <Wire.h>
#include <avr/wdt.h>

//#define DEBUG

#ifdef DEBUG
 #define DEBUG_PRINTLN(x) Serial.println(x)
 #define DEBUG_PRINT(x) Serial.print(x)
#else
 #define DEBUG_PRINTLN(x)
 #define DEBUG_PRINT(x)
#endif

// pin assignments
const int PHOTO_CELL_PIN = A0; 
const int TEMP_PIN = A1;
const int OVERRIDE_SWITCH_PIN = 2;
const int OVERRIDE_DIR__PIN = 3;
const int BOTTOM_SWITCH_PIN = 4;    
const int TOP_SWITCH_PIN = 5;       
const int ENABLE_MOTOR_PIN = 6;     
const int CLOSE_MOTOR_PIN = 7;  
const int OPEN_MOTOR_PIN = 8;   
const int CLOSED_LED_PIN = 9;

// I2C bus address
#define SLAVE_ADDRESS 0x05

// command constants
const unsigned long COMMAND_TIMEOUT = 5 * 60 * 1000; // 5 minutes
const byte CMD_ECHO = 0;
const byte CMD_RESET = 1;
const byte CMD_READ_TEMP = 2;
const byte CMD_READ_LIGHT = 3;
const byte CMD_READ_DOOR = 4;
const byte CMD_SHUT_DOOR = 5;
const byte CMD_OPEN_DOOR = 6;
const byte CMD_AUTO_DOOR = 7;
const byte CMD_UPTIME = 8;

// global variables
bool remoteClose = false;
bool remoteOpen = false;
bool resetRequested = false;

const int RESPONSE_SIZE = 4;
byte response[RESPONSE_SIZE]; 

unsigned long lastCommandTime = millis();

#define AREF_VOLTAGE 5.0

// ************************************** the setup **************************************

void setup(void) {
  Serial.begin(9600); // initialize serial port hardware
  DEBUG_PRINTLN("Coop Controller starting...");
  
  // coop door motor
  pinMode (ENABLE_MOTOR_PIN, OUTPUT);           // enable motor pin = output
  pinMode (CLOSE_MOTOR_PIN, OUTPUT);   // motor close direction pin = output
  pinMode (OPEN_MOTOR_PIN, OUTPUT);    // motor open direction pin = output
 
  // coop door leds
  pinMode (CLOSED_LED_PIN, OUTPUT);              // enable coopDoorClosedLed = output
  digitalWrite(CLOSED_LED_PIN, LOW);
 
  // coop door override switches
  pinMode (OVERRIDE_SWITCH_PIN, INPUT);
  digitalWrite(OVERRIDE_SWITCH_PIN, LOW);
  pinMode (OVERRIDE_DIR__PIN, INPUT);
  digitalWrite(OVERRIDE_DIR__PIN, HIGH);
 
  // bottom switch
  pinMode(BOTTOM_SWITCH_PIN, INPUT);                  // set bottom switch pin as input
  digitalWrite(BOTTOM_SWITCH_PIN, HIGH);              // activate bottom switch resistor
 
  // top switch
  pinMode(TOP_SWITCH_PIN, INPUT);                     // set top switch pin as input
  digitalWrite(TOP_SWITCH_PIN, HIGH);                 // activate top switch resistor

  DEBUG_PRINTLN("Joining i2c bus...");
  Wire.begin(SLAVE_ADDRESS);

  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);

  DEBUG_PRINTLN("i2c ready.");

  setupWatchdog();

  DEBUG_PRINTLN("Coop Controller ready.");
}

// ******** watchdog timer setup **********
void setupWatchdog(void) {
  DEBUG_PRINTLN("Setting up watchdog timer ...");
  cli();
  wdt_reset();
  // Enter Watchdog Configuration mode:
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  // Configure watchdog:
  WDTCSR  = (0<<WDIE)|(1<<WDE)|(1<<WDP3)|(0<<WDP2)|(0<<WDP1)|(1<<WDP0);
  sei();
  DEBUG_PRINTLN("Watchdog timer set.");
}
 
// ************************************** functions **************************************
 
int getSwitchState(unsigned long lastDebounceTime, int prevState, int pin) {
  const unsigned long DEBOUNCE_DELAY = 100;
  unsigned long currentTime = millis();
  int switchState = prevState;
 
  if ((currentTime - lastDebounceTime) > DEBOUNCE_DELAY) {     // delay for consistent readings
    lastDebounceTime = currentTime;
    int pinVal = digitalRead(pin);       // read input value and store it in val
    int pinVal2 = digitalRead(pin);      // read input value again to check or bounce
 
    if (pinVal == pinVal2) {             // make sure we have 2 consistant readings
      if (pinVal != prevState) {         // the switch state has changed!
        switchState = pinVal;
        
        Serial.print ("Pin Value changed to: ");  
        DEBUG_PRINTLN(switchState);         
        if(switchState == 0) {
          DEBUG_PRINTLN("switch is closed.");
        }
      }
    }
  }
  return switchState;
}

int isDoorOpen() {
  static int switchState = -1;
  static unsigned long lastDebounceTime = 0;
  switchState = getSwitchState(lastDebounceTime, switchState, TOP_SWITCH_PIN);
  return switchState == 0;
}

int isDoorClosed() {
  static int switchState = -1;
  static unsigned long lastDebounceTime = 0;
  switchState = getSwitchState(lastDebounceTime, switchState, BOTTOM_SWITCH_PIN);
  return switchState == 0;
}
 
// stop the coop door motor
void stopCoopDoorMotor() {
  //DEBUG_PRINTLN("Stopping motor");
  digitalWrite (CLOSE_MOTOR_PIN, LOW);      // turn off motor close direction
  digitalWrite (OPEN_MOTOR_PIN, LOW);       // turn on motor open direction
  digitalWrite (ENABLE_MOTOR_PIN, LOW);     // disable motor
}
 
// close the coop door motor (motor dir close = clockwise)
void closeCoopDoor() {
  const unsigned long RUN_AFTER_DOWN = 4000; 
  static boolean prevDoorClosed = false;
  static unsigned long timeClosed = 0;
  boolean doorWasClosed = true;
  boolean doorIsClosed = isDoorClosed();
  unsigned long currentTime = millis();
  
  
  if(doorIsClosed && !prevDoorClosed) {
    timeClosed = currentTime;
    DEBUG_PRINT("Set time closed: ");
    DEBUG_PRINTLN(timeClosed);
  }
  if(!doorIsClosed) {
    doorWasClosed = false;
    digitalWrite (CLOSE_MOTOR_PIN, HIGH);     // turn on motor close direction
    digitalWrite (OPEN_MOTOR_PIN, LOW);       // turn off motor open direction
    digitalWrite (ENABLE_MOTOR_PIN, HIGH);    // enable motor
  }
  // if bottom reed switch is closed for enough time 
  // to let slack out of rope and allow levers to lock
  // but only if the controller has not just started otherwise 
  // the motor will run in the down direction at start up 
  // after it is already down and locked
  if (doorIsClosed && (currentTime - timeClosed) > RUN_AFTER_DOWN || (currentTime < (RUN_AFTER_DOWN + 1000))) {
    stopCoopDoorMotor();
    if(!doorWasClosed) { // we just now closed it
        DEBUG_PRINTLN("Coop Door Closed");
        DEBUG_PRINT("Time since closed: ");
        DEBUG_PRINTLN(currentTime - timeClosed);
    }
  }
  prevDoorClosed = doorIsClosed;
}
 
// open the coop door (motor dir open = counter-clockwise)
void openCoopDoor() {
  boolean doorWasOpen = true;
  boolean doorIsOpen = isDoorOpen();
  
  if(!doorIsOpen) {
    doorWasOpen = false;
    digitalWrite(CLOSE_MOTOR_PIN, LOW);       // turn off motor close direction
    digitalWrite(OPEN_MOTOR_PIN, HIGH);       // turn on motor open direction
    digitalWrite(ENABLE_MOTOR_PIN, HIGH);              // enable motor
  }
  if (doorIsOpen) {                    // if top reed switch is closed
    stopCoopDoorMotor();
    if (!doorWasOpen) {
      DEBUG_PRINTLN("Coop Door open");
    }
  }
}
 
// do the coop door
void doCoopDoor() {
  static int overrideSwitchPinVal = 0;
  static int overrideDirSwitchPinVal = 0;
  int prevOverrideSwitchPinVal = overrideSwitchPinVal;
  int prevOverrideDirSwitchPinVal = overrideDirSwitchPinVal;

  overrideSwitchPinVal = digitalRead(OVERRIDE_SWITCH_PIN);
  if(overrideSwitchPinVal == 1) {
    overrideDirSwitchPinVal = digitalRead(OVERRIDE_DIR__PIN);
    if(prevOverrideSwitchPinVal != overrideSwitchPinVal) {
      DEBUG_PRINT("Override switch value changed. override=");    
      DEBUG_PRINTLN(overrideSwitchPinVal);
      if (prevOverrideDirSwitchPinVal != overrideDirSwitchPinVal) {
        DEBUG_PRINT("Override switch direction changed. direction=");
        DEBUG_PRINTLN(overrideDirSwitchPinVal);
      }
      
    }
    if(overrideDirSwitchPinVal == 1) {
      openCoopDoor();   
    } else {
      closeCoopDoor();  
    }
  } else if(remoteOpen) {
    openCoopDoor();  
  } else {
    closeCoopDoor();   
  }
  if(isDoorClosed()) {
    digitalWrite(CLOSED_LED_PIN, HIGH);
  } else {
    digitalWrite(CLOSED_LED_PIN, LOW);
  }
}
 
// ************************************** the loop **************************************
 
void loop() {
  doCoopDoor();
  // reset the watchdog timer unless we have received a command to reset the device
  // ot we have not received any command within the command timeout period
  if(!resetRequested && (millis() - lastCommandTime) < COMMAND_TIMEOUT) {
    wdt_reset();  
  } else {
    DEBUG_PRINTLN("Resetting...");
  }
}

void setResponse(unsigned long number) {
  DEBUG_PRINT("Sending ");
  response[0] = number >> 24;
  DEBUG_PRINT(response[0]);
  DEBUG_PRINT(",");
  response[1] = (number >> 16) & 0x000000FF;
  DEBUG_PRINT(response[1]);
  DEBUG_PRINT(",");
  response[2] = (number >> 8) & 0x000000FF;
  DEBUG_PRINT(response[2]);
  DEBUG_PRINT(",");
  response[3] = number & 0x000000FF;
  DEBUG_PRINTLN(response[3]);
}

void handleEchoCommand(int bytesToRead) {
  if(bytesToRead > RESPONSE_SIZE) {
    DEBUG_PRINTLN("Error requested too many bytes");
  } else {
    for(int i = 0; i < bytesToRead; i++) {
      response[i] = Wire.read();
    }
  }
}

void handleResetCommand() {
  // send response first before we reset
  setResponse(0);
  delay(1000);
  resetRequested = true;
}

void handleReadTempCommand() {
  unsigned long reading = analogRead(TEMP_PIN);
  float voltage = reading * AREF_VOLTAGE / 1024.0;
  float celsius = (voltage - 0.48) * 100;
  float fahrenheit = celsius * 1.8 + 32.0; 
  DEBUG_PRINT("Read ");
  DEBUG_PRINTLN(reading);
  DEBUG_PRINT("Volts: ");
  DEBUG_PRINT(voltage);
  DEBUG_PRINT(", Celsius: ");
  DEBUG_PRINT(celsius);
  DEBUG_PRINT(", Fahrenheit: ");
  DEBUG_PRINTLN(fahrenheit);
  setResponse(reading);
}

void handleReadLightCommand() {
  unsigned long reading = analogRead(PHOTO_CELL_PIN);
  DEBUG_PRINT("Read ");
  DEBUG_PRINTLN(reading);
  setResponse(reading);
}

void handleReadDoorCommand() {
  DEBUG_PRINT("Sending ");
  // initialize to "in transition" or unknown
  unsigned long doorState = 1;
  if(isDoorOpen()) {
    doorState = 0;
  } else if(isDoorClosed()) {
    doorState = 2;
  }
  setResponse(doorState);
}

void handleCloseDoorCommand() {
  remoteClose = true;
  remoteOpen = false;
  setResponse(0);
}

void handleOpenDoorCommand() {
  remoteClose = false;
  remoteOpen = true;
  setResponse(0);
}

void handleAutoDoorCommand() {
  remoteClose = false;
  remoteOpen = false;
  setResponse(0);
}

void handleUptimeCommand() {
  setResponse(millis());
}

void handleCommand(byte command, int bytesToRead) {
    // commands
    
    switch(command) {
      case CMD_ECHO:
         DEBUG_PRINTLN("Received echo command");
         handleEchoCommand(bytesToRead);
         break;
      case CMD_RESET:
         DEBUG_PRINTLN("Received reset command");
         handleResetCommand();
         break;
      case CMD_READ_TEMP:
         DEBUG_PRINTLN("Received read temp command");
         handleReadTempCommand();
         break;
      case CMD_READ_LIGHT:
         DEBUG_PRINTLN("Received read light command");
         handleReadLightCommand();
         break;
      case CMD_READ_DOOR:
         DEBUG_PRINTLN("Received read door command");
         handleReadDoorCommand();
         break;  
      case CMD_SHUT_DOOR:
         DEBUG_PRINTLN("Received close door command");
         handleCloseDoorCommand();
         break;  
      case CMD_OPEN_DOOR:
         DEBUG_PRINTLN("Received open door command");
         handleOpenDoorCommand();
         break;
      case CMD_AUTO_DOOR:
         DEBUG_PRINTLN("Received auto door command");
         handleAutoDoorCommand();
         break;
      case CMD_UPTIME:
         DEBUG_PRINTLN("Received uptime command");
         handleUptimeCommand();
         break;
      default: 
        DEBUG_PRINT("Unrecognized command: ");
        DEBUG_PRINTLN(command);
    }
    
    while(Wire.available()) {
      DEBUG_PRINT("Clearing wire data: ");
      DEBUG_PRINTLN(Wire.read());
    }
}

void onReceive(int byteCount) {
  DEBUG_PRINT("onReceive byteCount: ");
  DEBUG_PRINTLN(byteCount);
  lastCommandTime = millis();
  if(byteCount > 0 && Wire.available()) {
    handleCommand(Wire.read(), byteCount-1);
  }
}

void onRequest() {
  DEBUG_PRINT("onRequest: sending ");
  int numWritten = Wire.write(response, RESPONSE_SIZE);
}
