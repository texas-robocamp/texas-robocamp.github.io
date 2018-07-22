---
title: "Robot Programming Introduction"
tags: [robot programming]
keywords:
sidebar: tutorials
permalink: robot_programming_introduction.html
---



## Getting Started
Now that you've learned the basics of C++, we can move on to programming on the robot!

- Open up the Arduino IDE.

Before we start writing code, let's look at a simple Arduino code snippet. We'll start out with some code that just moves the robot around.

```
#include <BnrOneA.h>
#include <EEPROM.h>
#include <SPI.h>
BnrOneA one;

#define SSPIN 2

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  one.spiConnect(SSPIN);
  one.stop();
}

void loop() {
  // put your main code here, to run repeatedly:
  one.move(50,50);
  Serial.print("Moving");
}
```

### The Two Functions We'll Be Working With
Arduino programs are C++ programs.

After practicing C++ the first thing that you are likely to notice is the lack of a `main`.

Instead, the code that we will be using is divided into two functions, `setup` and `loop`.

`setup` is always called first. It is sort of like a constructor for setting up the robot. Things in `setup` happen once, when the robot is started.

`loop`, on the other hand, is called over and over. You can think of there being a `while` loop running on the robot which runs the `loop` command repeatedly as long as the power switch on the side of the robot is flipped on.


### The Header Files

```
#include <BnrOneA.h>
#include <EEPROM.h>
#include <SPI.h>
```

`BnrOneA.h` contains code specific to the Bot'n Roll One A robot which we are programming. There is a whole API which has been defined to access the functionality of this robot, and we will go step-by-step through this API 

`EEPROM.h` allows you to interact with the EEPROM on the Atmega328 microcontroller. This is sort of like saving files to a hard drive. Information on the EEPROM persists when the robot is shut off. This could be useful, for instance, if you solved a maze and wanted to store the solution to it, or if you programmed your robot to draw a picture on the floor and wanted to store the picture.

`SPI.h` gives you access to the Serial Peripheral Interface, and lets you write data over the USB port.

### The BnrOneA Class - Your Primary Interface to the Bot'n Roll API

```
BnrOneA one;
```

The `BnrOneA` class is your primary interface to interacting with the robot, through which you will read data from and control the robot.

### The rest of the code

Here, we'll quickly tell you what's going on with the rest of the code.

```
Serial.begin(57600);
```

This sets serial connection and specifies its speed (baud).


{{ site.data.alerts.tip }}
<ul>
<li>When using the Serial Monitor, make sure that the baud setting on the monitor is the same value as the number passed into Serial.begin(), or your output will not be understandable.</li>
<li>Similarly, if you decide to write C++ code which communicates over the USB port, you will need the baud rates to match between the robot and computer sides of the connection, or it will not work properly.</li>
</ul>
{{ site.data.alerts.end }}

```
one.spiConnect(SSPIN);
```

The Bot'n Roll uses SPI to communicate between the two microcontrollers on the robot: the Atmega328 - which we will be programming using Arduino, and the PIC18F45K22 - which we will not directly be programming.

The BnrOneA class formats messages to the PIC18F45K22, which are communicated using SPI. The PIC18F45K22 then directly handles some of the low-level control of the robot.

This line of code configures the connection between the two microcontrollers so the robot can operate properly.

```
one.stop();
```

This stops the motors on the Bot'n'Roll One. This line of code is just a precaution to ensure that the robot doesn't start moving for no reason.

```
one.move(speedL,speedR);
```

The `move` function takes in two parameters, `speedL` and `speedR`, and moves the left and right motors at the speeds specified. This function takes in values ranging from -100 to 100, where -100 is the maximum speed in reverse, and 100 corresponds to the maximum speed in the forward direction, and 0 stops the motors.

{{ site.data.alerts.tip }}
You can turn the robot by moving the motors at different speeds. 
<ul>
<li>-100, 100 would turn the robot as hard to the left as possible</li>
<li>100, 50, would turn the robot less sharply to the right, navigating the robot in a circle..</li>
</ul>
{{ site.data.alerts.end }}

```
Serial.print("Moving");
```

The `Serial.print` function for the Serial class similarly to `cout` in C++. It allows you to send data across the USB cable back to the computer.

## Hello World

Now that we're developing software on the actual robot, let's go ahead and write a new "Hello World" program. However, instead of just printing "Hello World" on the computer screen, we're going to print it on the robot's LCD screen! To do this, we'll need to use a new function in the Bot'n'Roll API, `lcdX()`.

### The `lcd` Function

```
void lcdX()
```

This function is actually one of many different lcdX functions, and each one takes in different arguments. The Bot'n'Roll has two lines of LCD output. Calling `lcd1` or `lcd2` specifies which line you would like for your text to appear on. There is no concept of a "carriage return," "\n," or `endl` that is useful here.

When calling this functions accept the following combinations of parameters:

- lcdX(string[])
- lcdX(num)
- lcdX(string[],num)
- lcdX(num1, num2)
- lcdX(num1, num2, num3)
- lcdX(num1, num2, num3, num4)

Here are some examples of how you could use these functions:

```
one.lcd1("The number is", 12); // lcdX(string[],num)
one.lcd2(1, 2); // lcdX(num1,num2)
```

### Exercise 4.1

- Write your first Hello World program on the robot!

- Create a new Arduino program, go to "File -> New".

- Copy the short program above into your Arduino IDE to set up your program, and save your program in an appropriate place on your computer (along with the other exercises you've done is a good idea).

- Edit this program so the robot does not start to move when it is running.

- Make this program say "Hello World!" on the first line in the LCD, and "Texas RoboCamp!" on the second line.

{% include callout_red_cup.html task="[Exercise 4.1]" %}

Before we start moving the car around, there's one more important function you'll need to learn how to use:

```
void delay(milliseconds)
```

This function pauses the program for the amount of time specified in milliseconds. You'll find this function to be useful for many things, such as ensuring that the car moves for a specified period of time, or that the lights stay on/off for a specific amount of time.

{{ site.data.alerts.tip }}
There are 1000 milliseconds in 1 second!"
{{ site.data.alerts.end }}

### Task 8.2

Let's try out using the `delay()` function. Write a program that alternates between showing "Hello World" and "Texas RoboCamp" on the LCD. Have it switch between the two every 5 seconds.

{% include callout_red_cup.html task="8.2" %}

## Moving the Car

The remaining functions from the obstacle_avoidance code that we haven't talked about all involve controlling the movement of the car.

```
one.move(50,50);
```

The `move()` function takes two arguments: speedL and speedR. These values define the speed of each motor, which ranges from -100 to 100. -100 corresponds to the max speed in reverse, and 100 corresponds to the max speed in the forward direction. An input of 0 stops the motor.

```
one.brake(50,50);
```

The `brake()` function also takes two arguments: torqueL and torqueR. These values define the braking power of each motor, which ranges between 0 and 100. Zero corresponds to stopping without braking, whereas 100 corresponds to stopping with the maximum braking torque.

### Task 8.2
Now that you know how to make the car start and stop, let's test out these functions! Write a program that has the car drive forwards for 2 seconds, brakes, and repeats.

Before you write this program, however, you'll need to know about one more function, `void delay(milliseconds)`. This function returns no value, and has the program wait for a specified amount of milliseconds before executing the next line of code. So, if you wanted to wait 5 seconds before running the next line of code, you would do:

```
//Do something
delay(5000);
//Do something else
```


### Additional Functions

Technically, you now have all of the functions that you need in order to get the robot to avoid obstacles. However, there are some additional functions for the BnrOneA class that you may find useful. Remember that to call any of these functions, you have to type `one.functionName()`.

```
void stop()
```

This function cuts energy to both motors on the Bot'n'Roll, causing them to rotate freely until they stop. This is the same as calling `move(0,0)`

```
void led(state)
```

This function can set the LED on or off, depending on the state passed in. Passing in 0 turns the LED off, and passing in 1 turns the LED on.

## Beeping

The default behavior of the obstacle avoidance program is to turn away when the robot senses obstacles on one side, or back up when the robot senses obstacles on both sides. This is bad driving! The robot should tell us when it's about to turn or back up. The objective of your first robot program is signalling. We will let you decide how to signal. We suggest a beep for backing up, and lights for turning.

In order to make your car beep, you'll need two more functions, `tone(pin,frequency,duration)` and `noTone(pin)`.

The `tone(pin,frequency,duration)` function generates a square wave of the specified frequency on the specified pin. The `duration` parameter is optional. If you do not put a value for duration, the wave will continue until `noTone(pin)` is called. The pin you'll be using for sound is pin 9.

### Task 8.3
Now that you know how to produce sound on your car, write a program that makes the car beep for 5 seconds.
