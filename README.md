# coop
This project includes the Arduino sketch and Fritzing schematic for a chicken coop controller.  The controller opens and shuts the door based on input from a photo resistor.  There is also a command handler to receive commands from a Raspberry Pi over the i2C pins.  The schematic does not include the i2c wiring yet.  Some of the ideas were taken from the implementation here http://davenaves.com/blog/interests-projects/chickens/arduino-chicken-coop/ .  

Modifications made to the Dave Naves sketch were:
* i2c communication
* Reorganization to remove most of global variables
* Averaging of the light sensor (photo resistor) over several consucutive samples to make readings more stable
* Change door action to wait until getting consecutive light readings over a period of 5 minutes to reduce chances of light flicker (head lights, flash lights etc) causing the door to open
* Improved closing to run slack out of rope to allow levers to lock
* Added watchdog timer to reset if hung


![Alt text](/src/coop_controller/coop_controller_schematic.jpg?raw=true "Coop Controller Schematic")
