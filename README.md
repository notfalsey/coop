# coop
This project includes the Arduino sketch and Fritzing schematic for a chicken coop controller.  The controller opens and shuts the door based on input from remote i2c commands or local override toggle switches.  The schematic does not include the i2c wiring yet.  

Some of the ideas were taken from the implementation here http://davenaves.com/blog/interests-projects/chickens/arduino-chicken-coop/ .  I originally started with a light controlled coop using input from the photo resistor like Dave did, but I went through two photo resistors that were giving bad readings.  I tried insulating them with different materials including silicone and glass food jars, but could not get them to work reliably.  Also, there is the minor problem of artificial light like car head lamps, which is usually temporary though.  So, I changed my design to have a raspberry pi master in my crawl space driving the arduino controller slave over i2c based on local sunrise/sunset retrieved from the weatherundergroud REST API.  i2c is designed for short distance board-to-board commmunication, but it still works nicely if you use low capacitance wires like cat 5.  And it is so nice to communicate using just three wires including the common ground wire.  I originally used water sprinkler wire but the capacitance was too high for i2c at a distance of 25 ft, so I switched to cat 5.  The really nice thing about using the raspberry pi master is I can control and monitor the coop remotely.  I have a web server running on the raspberry pi which allows me to monitor and control the coop while I am away.  The source for the web application that acts as the i2c master is at https://github.com/notfalsey/cooperator

This sketch also utilizes a watchdog timer to reset the arduino under the following conditions:
- there is somethint causing the main loop to hang
- there is a remote reset command sent from the master i2c device
- there has not been any command within the timeout (currently set at 5 minutes)

![Alt text](/src/coop_controller/coop_controller_schematic.jpg?raw=true "Coop Controller Schematic")
