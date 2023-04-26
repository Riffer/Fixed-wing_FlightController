# credits for the initial version go to  [s-kim-mcg](https://github.com/s-kim-mcg)

# Why this fork?
* I had Arduino Nano (diecimilaatmega328) in stock and needed some changes for the pinouts.
* VScode and PlatformIO instead of Ardunio IDE => I am lazy and also have a bad memory, so method completion and so on just catched me. 
* To get a better understanding of the stabilizer functions I refactored the code but you will find most of it intact or at least similar.
* I wanted CPPM instead of PWM as input to use only one cable instead of 4 with smaller receivers having only CPPM output.
* libraries for MPU and Servo instead of direct coding => easier to migrate to other platforms

# How to use this fork?
* Install VSCode and PlatformIO (as a plugin)
* Clone the repository in PlatformIO
* Wait a bit until the plaform is automatically installed and configured
* Compile and upload to your Arduino Nano (most likely you find diecimilaatmega328 version, that is a clone of the initial Arduino Nano)

# Pinout of this fork
* PIN D4-D7 are the PWM out for standard servos
* PIN D8 for CPPM input
* PIN A4 (SDA) and A5 (SCL) to MPU6050
* Connect the Arduino Nano to 5 Volt of your ESC (make sure about the voltage!)


# below you find the original README.md
---

# Fixed-wing_FlightController
This file is a fixed-wing aircraft horizontal posture control program using Arduino.

It is based on Arduino Uno, GY-86 (MPU 6050).

## What I use
Arduino Uno: For Main Controller

GY-86(MPU6050): To collect posture information

[Freewing] 690mm F-22 Raptor high Performance EDF PNP: Main frame of plane

DXe DSMX transmitter: Controller

AR620 Receiver: Receiver

Breadboard and connector

## System Configuration
![system](https://user-images.githubusercontent.com/74999377/132136380-a29437c9-d93b-475f-a03a-04bd8a06dd22.png)

## To use
Convert .c file to .ino and apply it.




## Test video
https://user-images.githubusercontent.com/74999377/193177427-28df1e31-e5e1-4556-9eca-17c9d2bf1875.mp4

## License
MIT
