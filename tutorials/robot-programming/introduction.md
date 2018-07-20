---
title: "Introduction"
tags: [robot programming]
keywords:
last_updated: July 5, 2018
summary:
sidebar: tutorials
permalink: robot_programming_introduction.html
---

## Turn and Push
Now that you understand the basics, let's start programming the robot! Your first task will be to implement obstacle detection and avoidance. This will involve monitoring the robot's sensors, and adjusting your movement accordingly to ensure that the robot does not get trapped. We'll be opting to do a navigation method called turn-and-push. You've probably seen this method before - it's what Roombas do to avoid collisions as well! The method goes as follows:

- If the robot sees an obstacle on one side, back up, turn towards the clear side, and continue forwards
- If the robot sees obstacles on both sides, back up

You'll probably find it helpful to use the obstacle_avoidance code as a starting point, so go ahead and open that up again. You'll notice that the bulk of the code is divided into two functions, `setup()` and `loop()`. The setup function is called first, and initalizes the robot. The code inside of this function is all neccessary, and is used to establish connections to the robot and ensure that it starts without any motors running. The actual obstacle detection will be done inside of `loop()`.

```
void loop() {
  int sensor = one.obstacleSensors();
  if(sensor == 0){
    one.move(50,50);
    Serial.print("No objects detected");
    one.lcd1("No objects detected");

  }
  else {
    one.brake(50,50);
    Serial.print("Object detected");
    one.lcd1("Object detected");
  }
}
```

Now that you've gone through the programming tutorials, most of this code should make sense to you. However, you'll notice that there are some function calls that seem unfamiliar. These are functions included in the Bot'n'Roll API, so we'll help you get started on figuring out what they do.

```
BnrOneA one;
```

This line is initializing an BnrOneA object with the name of one. The BnrOneA object is how you'll send commands and receive information from the robot.

```
int sensor = one.obstacleSensors();
```

The `obstacleSensors()` function returns an `int` based on what the robot's front sensors are detecting. If you receive 0, no obstacles are being detected. If you receive 1, then an obstacle is being detected on the left sensor. Receiving 2 indicates that an obstacle has been detected on the right sensor. Finally, receiving a 3 means that obstacles were detected on both the right and left sensors.

```
one.move(50,50);
```

The `move()` function takes two arguments: speedL and speedR. These values define the speed of each motor, which ranges from -100 to 100. -100 corresponds to the max speed in reverse, and 100 corresponds to the max speed in the forward direction. An input of 0 stops the motor.


```
Serial.print("No objects detected");
```

The `print()` function for the Serial class works in a similar fashion to the `printf()` function in C. Use this to output debugging information. To view the output, you'll need to look at the Serial Monitor, which can be found at "Tools -> Serial Monitor". 

<div style="background-color:rgba(0,255,0,0.5)">
<b>TIP</b>
<p></p>
<p>Make sure that the baums setting on the monitor is the same value as the number passed into Serial.begin(), or your output will not be understandable.</p>
</div>


```
one.lcd1("No objects detected");
```

This function is actually one of many different lcdX functions, and each one takes in different arguments. The Bot'n'Roll has two LCD screens, so it is valid to call lcd1 or lcd2 to produce output. You may find it useful to use these screens for debugging, since any calls to Serial.print() are only readable when you have the robot connected to your computer. When calling this function, you can pass in any of the following combinations of parameters:

- lcdX(string[])
- lcdX(num)
- lcdX(string[],num)
- lcdX(num1, num2)
- lcdX(num1, num2, num3)
- lcdX(num1, num2, num3, num4)

```
one.brake(50,50);
```

The `brake()` function also takes two arguments: torqueL and torqueR. These values define the braking power of each motor, which ranges between 0 and 100. Zero corresponds to stopping without braking, whereas 100 corresponds to stopping with the maximum braking torque.

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



## Task X

Now that you understand these functions, we can start coding! Your first programming task on the robot is to implement obstacle avoidance. 

The default behavior of the obstacle avoidance program is to turn away when the robot senses obstacles on one side, or back up when the robot senses obstacles on both sides. This is bad driving! The robot should tell us when it's about to turn or back up. The objective of your first robot program is signalling.

We will let you decide how to signal. We suggest a beep for backing up, and lights for turning.

### Helpful Functions
For beeping, you may find these functions useful:
- tone(pin, frequency, duration)
- notone(pin)

Note: The buzzer that emits sound is located on digital pin 9. 

<div style="background-color:rgba(255,0,0,0.5)">
<b>RED CUP: Track Progress [Task X]</b>
<ul>
<li>Please flip your cup to red.</li><li>A camp staff member will bring you to an open space to drive your robot into.</li>
<li>Feel free to run your robot at your desk while you wait, but try not to create chaos in doing so.</li>
<li>Camp Staff: **TODO: LOCATION UNKNOWN**Bring the group to a wall or out to the bridge to test that the robot both drives forward and stops when encountering the wall, using its obstacle detection sensors.</li>
</ul>
</div>

