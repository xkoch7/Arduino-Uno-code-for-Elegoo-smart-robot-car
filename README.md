# Overview

This is a Github repositroy that contains code dealing with the sensors, motors and setup of the Elegoo smart car robot V4.0. This repository contains code for detecting lines, following lines and controlling motors. Along with using the gyro, servo and ultra sonic sensors to imporve controls and turning along with detecting obstacles.
## How to run

#### First you need these key resources
[Smart car robot link](https://www.elegoo.com/blogs/arduino-projects/elegoo-smart-robot-car-kit-v4-0-tutorial?srsltid=AfmBOoqn-qBqnSIaAcDy842MfXGA_dFm8mD5bX5J9NB-plZlTgztCLJc) and any Compatable Ide that supports arduino Uno we prefered to use and recommend ([Arduino IDE](https://docs.arduino.cc/software/ide/))

Then in order to actually run code in the ide and upload it into your robot. You will need to charge your robot and then take any existing code or your own writen code that works with your robots pins. Then with the code in your ide set up to upload port to your desired usb port. Plug the robot in to your laptop and click upload if there are no errors in your code the robot should run the code when turned on after upload.

## Known issues
The known issues we ran into outside of code errors was making sure that your ports are working fine. Along with caables being plugged in correctly when building the robot. When setting up your robot be sure to switch on the upload port on the actual board. Also in our code we have out set loght values for our line detection however your robot may have different values depending onyour sensor
