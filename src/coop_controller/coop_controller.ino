#include <Wire.h>
#include <avr/wdt.h>

#define DEBUG

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
const int OVERRIDE_DIR_PIN = 3;
const int BOTTOM_SWITCH_PIN = 4;    
const int TOP_SWITCH_PIN = 5;       
const int ENABLE_MOTOR_PIN = 6;     
const int CLOSE_MOTOR_PIN = 7;  
const int OPEN_MOTOR_PIN = 8;   
const int CLOSED_LED_PIN = 9;

// I2C bus address
#define SLAVE_ADDRESS 0x05

// command constants
const unsigned long COMMAND_TIMEOUT = 60 * 1000; // 1 minute
const byte CMD_ECHO = 0;
const byte CMD_RESET = 1;
const byte CMD_READ_TEMP = 2;
const byte CMD_READ_LIGHT = 3;
const byte CMD_READ_DOOR = 4;
const byte CMD_SHUT_DOOR = 5;
const byte CMD_OPEN_DOOR = 6;
const byte CMD_AUTO_DOOR = 7;
const byte CMD_UPTIME = 8;
const byte CMD_READ_MODE = 9;

// global variables
bool remoteClose = false;
bool remoteOpen = false;
bool resetRequested = false;

const int RESPONSE_SIZE = 4;
byte response[RESPONSE_SIZE]; 

unsigned long lastCommandTime = millis();
unsigned long DEBOUNCE_DELAY = 100;

#define AREF_VOLTAGE 5.0

// ************************************** the setup **************************************

void setup(void) {
  Serial.begin(9600); // initialize serial port hardware
  DEBUG_PRINTLN("Coop Controller starting...");
  
  // coop door motor
  pinMode (ENABLE_MOTOR_PIN, OUTPUT);  // enable motor pin = output
  pinMode (CLOSE_MOTOR_PIN, OUTPUT);   // motor close direction pin = output
  pinMode (OPEN_MOTOR_PIN, OUTPUT);    // motor open direction pin = output
  stopCoopDoorMotor();
 
  // coop door leds
  pinMode (CLOSED_LED_PIN, OUTPUT);        // enable coopDoorClosedLed = output
  digitalWrite(CLOSED_LED_PIN, LOW);
 
  // coop door override switches
  pinMode (OVERRIDE_SWITCH_PIN, INPUT);
  digitalWrite(OVERRIDE_SWITCH_PIN, LOW);
  pinMode (OVERRIDE_DIR_PIN, INPUT);
  digitalWrite(OVERRIDE_DIR_PIN, HIGH);
 
  // bottom switch
  pinMode(BOTTOM_SWITCH_PIN, INPUT);       // set bottom switch pin as input
  digitalWrite(BOTTOM_SWITCH_PIN, HIGH);   // activate bottom switch resistor
 
  // top switch
  pinMode(TOP_SWITCH_PIN, INPUT);          // set top switch pin as input
  digitalWrite(TOP_SWITCH_PIN, HIGH);      // activate top switch resistor

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
int isDoorOpen() {
  static int switchState = digitalRead(TOP_SWITCH_PIN);
  static unsigned long lastDebounceTime = millis();
  int reading = digitalRead(TOP_SWITCH_PIN);
  static int lastReading = reading;

  if(reading != lastReading) {
    DEBUG_PRINTLN("Diff reading");
    lastDebounceTime = millis();    
  }

  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) { 
    if(reading != switchState) {
      switchState = reading;
      if(switchState == 0) {
        DEBUG_PRINTLN("Top switch CLOSED");
      } else {
        DEBUG_PRINTLN("Top switch OPENED");
      }
    }
  }
  
  lastReading = reading;
  return switchState == 0;
}

int isDoorClosed() {
  static int switchState = digitalRead(BOTTOM_SWITCH_PIN);
  static unsigned long lastDebounceTime = millis();
  int reading = digitalRead(BOTTOM_SWITCH_PIN);
  static int lastReading = reading;

  if(reading != lastReading) {
    DEBUG_PRINTLN("Diff reading");
    lastDebounceTime = millis();    
  }

  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) { 
    if(reading != switchState) {
      switchState = reading;
      if(switchState == 0) {
        DEBUG_PRINTLN("Bottom switch CLOSED");
      } else {
        DEBUG_PRINTLN("Bottom switch OPENED");
      }
    }
  }
  
  lastReading = reading;
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
  boolean justStarted = currentTime < (RUN_AFTER_DOWN + 1000);
  boolean hasAllowedSlack = (currentTime - timeClosed) > RUN_AFTER_DOWN;
  if (doorIsClosed && (hasAllowedSlack || justStarted)) {
    stopCoopDoorMotor();
    if(!doorWasClosed && !justStarted) { // we just now closed it
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
    //DEBUG_PRINTLN("Activating motor in open direction");
    digitalWrite(CLOSE_MOTOR_PIN, LOW);       // turn off motor close direction
    digitalWrite(OPEN_MOTOR_PIN, HIGH);       // turn on motor open direction
    digitalWrite(ENABLE_MOTOR_PIN, HIGH);     // enable motor
  }
  if (doorIsOpen) {                    // if top reed switch is closed
    stopCoopDoorMotor();
    if (!doorWasOpen) {
      DEBUG_PRINTLN("Coop Door open");
    }
  }
}

// true if in override mode; false otherwise
boolean isOverrideMode() {
  static boolean lastValue = false;
  boolean newValue = digitalRead(OVERRIDE_SWITCH_PIN) == 1;
  if(lastValue != newValue) {
    if(newValue == true) {
      DEBUG_PRINTLN("Override mode changed to ON");  
    } else {
      DEBUG_PRINTLN("Override mode changed to OFF");  
    }
  }
  lastValue = newValue;
  return newValue;
}

// true if override toggle switch is down; false otherwise
boolean isOverrideDirToggleUp() {
  static boolean lastValue = false;
  boolean newValue = digitalRead(OVERRIDE_DIR_PIN) == 1;
  if(lastValue != newValue) {
    if(newValue == true) {
      DEBUG_PRINTLN("Override toggle direction changed to UP");  
    } else {
      DEBUG_PRINTLN("Override toggle direction changed to DOWN");  
    }
  }
  lastValue = newValue;
  return newValue;
}
 
// do the coop door
void doCoopDoor() {
  static int overrideDirUp = isOverrideDirToggleUp();
  
  if(isOverrideMode()) {
    if(isOverrideDirToggleUp()) {
      openCoopDoor();   
    } else {
      closeCoopDoor();  
    }
  } else if(remoteOpen) {
    openCoopDoor();  
  } else if(remoteClose) {
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
  // initialize to "in transition" or unknown
  unsigned long doorState = 1;
  if(isDoorOpen()) {
    doorState = 0;
  } else if(isDoorClosed()) {
    doorState = 2;
  }
  setResponse(doorState);
}

void handleReadModeCommand() {
  setResponse(isOverrideMode());
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
      case CMD_READ_MODE:
         DEBUG_PRINTLN("Received read mode command");
         handleReadModeCommand();
         break;  
      default: 
        DEBUG_PRINT("Unrecognized command: ");
        DEBUG_PRINTLN(command);
    }
    
    while(Wire.available()) {
      DEBUG_PRINT("Clearing wire data: ");
      DEBUG_PRINTLN(Wire.read());
    }
    
    // we were healthy enough to process an entire command, so reset the watchdog
    if(!resetRequested) {
      wdt_reset(); 
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
