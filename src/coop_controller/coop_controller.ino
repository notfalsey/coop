#include <Wire.h>

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

// light states
const byte DARK = 0;
const byte INCONSISTENT = 1;
const byte LIGHT = 2;

// I2C bus address
#define SLAVE_ADDRESS 0x05

// global variables
bool remoteClose = false;
bool remoteOpen = false;

#define AREF_VOLTAGE 5.0


// ************************************** the setup **************************************

void setup(void) {
  Serial.println("Coop Controller starting...");
  
  Serial.begin(9600); // initialize serial port hardware

  //analogReference(EXTERNAL);
  
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

  Wire.begin(SLAVE_ADDRESS);

  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);

  Serial.println("Coop Controller ready.");
}
 
// ************************************** functions **************************************
 
// photocell to read levels of exterior light
// will either return dark, inconsistent, or light
// if there are two consecutive readings that are consistent, then that reading will be returned (light or dark); 
// otherwise it will return inconsistent, which means it may be in transition or there may have been 
// an inaccurate reading
byte isLight(boolean immediate) { 
  const unsigned long PHOTO_CELL_DELAY = 300000;   // milliseconds
  const int LIGHT_THRESHOLD = 500; 
  const int SAMPLES_PER_READING = 10;
  static boolean prevReading = false; 
  static boolean lastReading = false; 
  static unsigned long lastPhotocellReadingTime = 0;
  unsigned long currentTime = millis();
  int photocellReading = 0;
  // if the delay time has passed or we are just starting or we want an immediate reading
  if (lastPhotocellReadingTime == 0 || immediate || ((currentTime - lastPhotocellReadingTime) > PHOTO_CELL_DELAY)) {
    for(int i = 0; i < SAMPLES_PER_READING; i++) {
      photocellReading += analogRead(PHOTO_CELL_PIN);  
    }
    photocellReading /= SAMPLES_PER_READING;
    
    prevReading = lastReading;
    //  set photocell threshholds
    if (photocellReading < LIGHT_THRESHOLD) {
      lastReading = false;
      Serial.println("Photocell Reading Level: Dark");
    } else {
      lastReading = true;
      Serial.println("Photocell Reading Level: Light");
    }
    if(lastPhotocellReadingTime == 0 || immediate) {
      prevReading = lastReading;    
    }
    Serial.print("Photocell Analog Reading: ");
    Serial.println(photocellReading);
    Serial.print("Last Light Reading: ");
    Serial.println(lastReading);
    Serial.print("Prev Light Reading: ");
    Serial.println(prevReading);
    lastPhotocellReadingTime = currentTime;
  }
  byte ret = INCONSISTENT;
  if(!lastReading && !prevReading) {
    ret = DARK;
  } else if(lastReading && prevReading) {
    ret = LIGHT;
  }
  return ret;
}

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
        Serial.println(switchState);         
        if(switchState == 0) {
          Serial.println("switch is closed.");
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
  //Serial.println("Stopping motor");
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
    Serial.print("Set time closed: ");
    Serial.println(timeClosed);
  }
  if(!doorIsClosed) {
    doorWasClosed = false;
    digitalWrite (CLOSE_MOTOR_PIN, HIGH);     // turn on motor close direction
    digitalWrite (OPEN_MOTOR_PIN, LOW);       // turn off motor open direction
    digitalWrite (ENABLE_MOTOR_PIN, HIGH);    // enable motor
  }
  // if bottom reed switch is closed for enough time to let slack out of rope
  if (doorIsClosed && (currentTime - timeClosed) > RUN_AFTER_DOWN) {
    stopCoopDoorMotor();
    if(!doorWasClosed) { // we just now closed it
        Serial.println("Coop Door Closed");
        Serial.print("Time since closed: ");
        Serial.println(currentTime - timeClosed);
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
  if (doorIsOpen) {                           // if top reed switch is closed
    stopCoopDoorMotor();
    if (!doorWasOpen) {
      Serial.println("Coop Door open");
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
      Serial.print("Override switch value changed. override=");    
      Serial.println(overrideSwitchPinVal);
      if (prevOverrideDirSwitchPinVal != overrideDirSwitchPinVal) {
        Serial.print("Override switch direction changed. direction=");
        Serial.println(overrideDirSwitchPinVal);
      }
      
    }
    if(overrideDirSwitchPinVal == 1) {
      openCoopDoor();   
    } else {
      closeCoopDoor();  
    }
  } else if(remoteClose) {
    closeCoopDoor();  
  } else if(remoteOpen) {
    openCoopDoor();   
  } else {
    // act according to the light
    // if we just changed to non-override mode, then force an immediate photecell read
    boolean immediateReading = (overrideSwitchPinVal == 0) && (prevOverrideSwitchPinVal == 1);
    // act according to photo cell reading 
    byte lightSensor = isLight(immediateReading);
    if (lightSensor == DARK) {
       closeCoopDoor();    // close the door
    } else if (lightSensor == LIGHT) {
       openCoopDoor();     // Open the door
    } // else do nothing because we dont have consistent consecutive readings
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

const int RESPONSE_SIZE = 2;
byte response[RESPONSE_SIZE]; 

void handleEchoCommand(int bytesToRead) {
  if(bytesToRead > RESPONSE_SIZE) {
    Serial.println("Error requested too many bytes");
  } else {
    for(int i = 0; i < bytesToRead; i++) {
      response[i] = Wire.read();
    }
  }
}

void handleReadTempCommand() {
  int reading = analogRead(TEMP_PIN);
  float voltage = reading * AREF_VOLTAGE / 1024.0;
  float celsius = (voltage - 0.48) * 100;
  float fahrenheit = celsius * 1.8 + 32.0; 
  Serial.print("Read ");
  Serial.println(reading);
  Serial.print("Volts: ");
  Serial.print(voltage);
  Serial.print(", Celsius: ");
  Serial.print(celsius);
  Serial.print(", Fahrenheit: ");
  Serial.println(fahrenheit);
  Serial.print("Sending ");
  response[0] = reading / 256;
  response[1] = reading % 256;
  Serial.print(response[0]);
  Serial.print(", ");
  Serial.println(response[1]);
}

void handleReadLightCommand() {
  int reading = analogRead(PHOTO_CELL_PIN);
  Serial.print("Read ");
  Serial.println(reading);
  Serial.print("Sending ");
  response[0] = reading / 256;
  response[1] = reading % 256;
  Serial.print(response[0]);
  Serial.print(", ");
  Serial.println(response[1]);
}

void handleReadDoorCommand() {
  Serial.print("Sending ");
  response[0] = 0;
  int doorState = 0;
  if(isDoorOpen()) {
    response[1] = 0;
  } else if(isDoorClosed()) {
    response[1] = 2;
  } else {
    // in transition
    response[1] = 1;
  }
  Serial.print(response[0]);
  Serial.print(", ");
  Serial.println(response[1]);
}

void handleCloseDoorCommand() {
  remoteClose = true;
  remoteOpen = false;
}

void handleOpenDoorCommand() {
  remoteClose = false;
  remoteOpen = true;
}

void handleAutoDoorCommand() {
  remoteClose = false;
  remoteOpen = false;
}

void handleCommand(int command, int bytesToRead) {
    // commands
    const int CMD_ECHO = 0;
    const int CMD_RESET = 1;
    const int CMD_READ_TEMP = 2;
    const int CMD_READ_LIGHT = 3;
    const int CMD_READ_DOOR = 4;
    const int CMD_SHUT_DOOR = 5;
    const int CMD_OPEN_DOOR = 6;
    const int CMD_AUTO_DOOR = 7;

    switch(command) {
      case CMD_ECHO:
         Serial.println("Received echo command");
         handleEchoCommand(bytesToRead);
         break;
      case CMD_RESET:
         Serial.println("Received reset command");
         Serial.println("Reset not implemented");
         break;
      case CMD_READ_TEMP:
         Serial.println("Received read temp command");
         handleReadTempCommand();
         break;
      case CMD_READ_LIGHT:
         Serial.println("Received read light command");
         handleReadLightCommand();
         break;
      case CMD_READ_DOOR:
         Serial.println("Received read door command");
         handleReadDoorCommand();
         break;  
      case CMD_SHUT_DOOR:
         Serial.println("Received close door command");
         handleCloseDoorCommand();
         break;  
      case CMD_OPEN_DOOR:
         Serial.println("Received open door command");
         handleOpenDoorCommand();
         break;
      case CMD_AUTO_DOOR:
         Serial.println("Received auto door command");
         handleAutoDoorCommand();
         break;
      default: 
        Serial.print("Unrecognized command: ");
        Serial.println(command);
    }
    
    while(Wire.available()) {
      Serial.print("Clearing wire data: ");
      Serial.println(Wire.read());
    }
}

void onReceive(int byteCount) {
  Serial.print("onReceive byteCount: ");
  Serial.println(byteCount);
  if(byteCount > 0 && Wire.available()) {
    handleCommand(Wire.read(), byteCount-1);
  }
}

void onRequest() {
  Serial.print("onRequest: sending ");
  Serial.print(response[0]);
  Serial.print(", ");
  Serial.println(response[1]);
  Wire.write(response, RESPONSE_SIZE);
}
