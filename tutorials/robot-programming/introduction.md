---
title: "Introduction"
tags: [robot programming]
keywords:
sidebar: tutorials
permalink: robot_programming_introduction.html
---



### Getting Started
Now that you've learned the basics of C++, we can move on to programming on the actual robot! Go ahead and open up the Arduino IDE again - remember, to do so, type `arduino` into either the terminal or the lens.

Before we start writing code, let's take apart a simple Arduino code snippet. We'll start out with some code that just moves the robot around.

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

Arduino prorgams are C++ programs, but the formatting for Arduino programs is special. You'll notice that the the code is divided into two functions, `setup()` and `loop()`. `setup()` is always called first. The code inside of this function is all neccessary, and is used to establish connections to the robot and ensure that it starts without any motors running. Almost all of the code you'll be writing will go inside `loop()`, which runs continually after `setup()` has ended.

Now that you've gone through the programming tutorials, most of this code should make sense to you. However, you'll notice that there are some lines that seem unfamiliar. These are functions included and needed for the Bot'n'Roll API, so we'll help you get started on figuring out what they do.

```
#include <BnrOneA.h>
#include <EEPROM.h>
#include <SPI.h>
```

These include statements are used to import the libraries you'll need to work with the robot.

```
BnrOneA one;
```

This line is initializing an BnrOneA object with the name of one. The BnrOneA object is how you'll send commands and receive information from the robot.

```
Serial.begin(57600);
```

This sets up the data rate in bits per second (baud) for serial transmission. Since you'll be communicating with the computer, the baud rate is set to 57600. Without this line of code, you wouldn't be able to use the Serial Monitor, which is a useful tool for debugging programs.

```
one.spiConnect(SSPIN);
```

This initializes the SPI communication bus. Basically, this is initializing the hardware on the robot to its proper initial state.

```
one.stop();
```

This stops the motors on the Bot'n'Roll One. This line of code is just a precaution to ensure that the robot doesn't start moving for no reason.

```
one.move(speedL,speedR);
```

The `move()` function takes in two parameters, `speedL` and `speedR`, and moves the left and right motors at the speeds specified. This function takes in values ranging from -100 to 100, where -100 is the maximum speed in reverse, and 100 corresponds to the maximum speed in the forward direction, and 0 stops the motors.

```
Serial.print("Moving");
```

The `print()` function for the Serial class works in a similar fashion to how you've used cout in C++. Use this to output debugging information when you're connected to the computer via USB. To view the output, you'll need to look at the Serial Monitor, which can be found at "Tools -> Serial Monitor".

{% include tip.html content="Make sure that the baud setting on the monitor is the same value as the number passed into Serial.begin(), or your output will not be understandable." %}

### Hello World

Now that we're developing software on the actual robot, let's go ahead and write a new "Hello World" program. However, instead of just printing "Hello World" on the computer screen, we're going to print it on the robot's LCD screen! To do this, we'll need to use a new function in the Bot'n'Roll API, `lcdX()`.

```
void lcdX()
```

This function is actually one of many different lcdX functions, and each one takes in different arguments. The Bot'n'Roll has two LCD screens, so it is valid to call lcd1 or lcd2 to produce output. You may find it useful to use these screens for debugging, since any calls to Serial.print() are only readable when you have the robot connected to your computer. When calling this function, you can pass in any of the following combinations of parameters:

- lcdX(string[])
- lcdX(num)
- lcdX(string[],num)
- lcdX(num1, num2)
- lcdX(num1, num2, num3)
- lcdX(num1, num2, num3, num4)

Here are some examples of how you could use these functions:

```
one.lcd1("The number is",12); // lcdX(string[],num)
one.lcd2(1,2); // lcdX(num1,num2)
```

### Task 8.1

Now that you know how to use the `lcdX()` functions, go ahead and write your first Hello World program on the robot! Remember, to create a new Arduino program, go to "File -> New". A new window should open up with an empty `setup()` and `loop()` function. Make sure you don't forget to `include` the libraries and write your `setup()` function correctly, or you won't be able to interface with the robot.

{% include callout_red_cup.html task="8.1" %}

Before we start moving the car around, there's one more important function you'll need to learn how to use:

```
void delay(milliseconds)
```

This function pauses the program for the amount of time specified in milliseconds. You'll find this function to be useful for many things, such as ensuring that the car moves for a specified period of time, or that the lights stay on/off for a specific amount of time.

{% include tip.html content="There are 1000 milliseconds in 1 second!"}

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
