---
title: "Firing Up the Robot"
tags: [arduino]
keywords: arduino
last_updated: July 17, 2018
sidebar: tutorials
permalink: arduino.html
---

<div style="background-color:rgba(255, 255, 0, 0.5)">
<b>SYNCHRONIZE</b>
<ul>
<li>There is reading material for you to enjoy while you wait for the tutorial to begin. The group section will start at the section marked GROUP SECTION.</li>
</ul>
</div>

## Microcontrollers

At the heart of your robot, and what you will spend much of the week programming, is an Atmega 328 microcontroller. A microcontroller is like a entire computer on a single microchip. It has storage, a processor, and memory, much like any other computer does.

Microcontrollers are designed primarily to be embedded in other devices. Many devices that you are familiar with have some type of microcontroller driving them.

There are actually two microcontrollers on the Bot'n Roll One. The second is a PIC18F45K22. On the Bot'n'Roll, this comes pre-loaded with all of the software that we want on it.

## Arduino

<div style="background-color:rgba(0, 255, 0, 0.5)">
<b>TIP</b>
<ul>
<li>Don't forget to open external links in new tabs if you go to them for extra reading. You'll want to get back to the tutorial quickly.</li>
</ul>
</div>

Arduino is an open-source electronics platform designed around Atmega microcontrollers. You can find more information on Arduinos ["Here"](https://www.arduino.cc/en/Guide/Introduction)

Arduinos makes the previously very difficult-to-enter world of microcontrollers accessible to programmers who already knew how to program in C++.

Part of this is by standardizing how pieces interact with the Arduino. Whereas there are many ways to connect devices to an Atmega microcontroller, there are a few *very well documented* ways to various devices it to an Arduino.

Another part of this is by offering an extensive API (Application Program Interface) for developers who want to use parts of the microcontroller. The API handles the small parts of interacting with the Atmega microcontroller so developers can concentrate on the problem that they want to solve.

Arduino microcontrollers also are at the heart of a number of projects that enthusiasts at all levels can take on. For instance, ["Adafruit"](https://www.adafruit.com/category/17?gclid=EAIaIQobChMI2LD6vtWl3AIVjYbACh0IvwY0EAAYASAAEgJKDPD_BwE) offers a number of kits based on Arduinos for enthusiasts to assemble and program.

One of my (Justin's) favorite projects for the Arduino microcontroller is building 3D printers. The RAMPS (RepRap Arduino Mega Pololu Shield) board is designed to attach to an Arduino in order to control a 3D printer.

Here are a couple of pictures of mine. Mine is a HyperCube CoreXY.

![HyperCube CoreXY](images/hypercube.jpg)

![HyperCube Owl](images/hypercube_owl.jpg)

## Loading Your First Program

<div style="background-color:rgba(255, 255, 0, 0.5)">
<b>GROUP SECTION</b>
<ul>
<li>People will have the same questions while we go through the basics for the first time. Let's do it together!</li>
</ul>
</div>

The first program we're going to load is a simple program that blinks the LEDs on the robot. Here, you'll learn how to navigate through the Arduino IDE so that you can build and upload your own programs.

First, let's go ahead and open up the Arduino IDE. In case you forgot, this can be done by typing "arduino" in either the terminal or the lens. Now, navigate to "File-> Examples -> BnrOneA -> Basic" and select "LED". You should see a new window pop up, with a program already written.

Now, let's go ahead and upload this code to the robot. To do so, you'll need to understand the menu bar at the top of the IDE. It should look just like this:

![IDE Toolbar](images/ide_toolbar.png)

The names of the buttons are as follows: Verify, Upload, New, Open, and Save. Let's go through what each of these does.

- The first button is the **verify** button. This is used to compile your code and check for errors. Go ahead and click that button, and you'll see the terminal at the bottom output some build information. You don't need to worry about what it's saying for now.

- The next button is the **upload** button. Once you've verified that your code works, selecting this button will upload the code to the robot. 

- The next button is the **new** button. You'll use this to create new scratch programs.

- The next button is the **open** button. You can use this to open up any scratch files that are located on your computer.

- The final button is the **save** button. Use this to save any programs you've written.


## Git

Git is a version control system. We could spend the whole week discussing the ins and outs of version control.. so we won't. We're just going to tell you what to type here.

<div style="background-color:rgba(127, 255, 255, 0.5)">
<b>OPEN A TERMINAL AND DO THIS</b>
<ul>
<li>cd</li>
<li>git clone git@github.com:UTCS-Robotics-Camp/exercises.git</li>
</ul>
</div>


## Next Step

Proceed to ["Welcome!"](texas-robocamp.github.io/welcome.html)
