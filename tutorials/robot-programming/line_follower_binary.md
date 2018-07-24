---
title: "The Line Follower"
tags: [Binary]
keywords:
sidebar: tutorials
permalink: line_follower.html
---

## Back to the Bot'n Roll Library

The `BnrOneA` class still has a significant amount of functionality that we haven't yet touched, one of those is the use of the analog-to-digital converter

### `int readAdc(byte), int readAdc#()`

The PIC microcontroller has an 8-channel analog to digital converter on it. This allows us to hook electronics components up to the robot and read information off of them. The data comes in as analog data; that is to say, electrical voltages. An <b>analog-to-digital (ADC)</b> converter converts these voltages to digital information, allowing us to use it in our computer programs.

{{ site.data.alerts.tip }}
<ul>
<li>You can think of the analog data sort of like a dimmer switch on a light. More electricity makes the light brighter. The problem for the microcontroller is that it doesn't work with voltages as analog circuits do. It works with binary, discrete representations. The <b>analog-to-digital (ADC)</b> will convert the voltage into a number that can be understood by the computer.</li>
</ul>
{{ site.data.alerts.end }}

The PIC has 8 ADC channels, which can be accessed by calling either the corresponding `readAdc#` function, or by providing 0-7 to `readAdc`. 

## The Line Follower Device
Look under your robot at the line follower. It has 8 little black blocks on it. These blocks are used to detect how much light us coming up off the ground to the robot.

Each of those blocks is hooked up to a separate channel of the ADC. Consequently, the ADC will read each of them and return a value between 0-1023.

## Exercise 4.5

### The Line Follower Test Target

- Print the test target out on Printer 303.

{{ site.data.alerts.terminal_commands }}
cd ~/forms
wget https://github.com/texas-robocamp/texas-robocamp.github.io/raw/master/forms/line.PDF
lpr -Plw303 line.PDF
{{ site.data.alerts.terminal_commands_end }}

- Go get the line target off of the printer.

### Tips on Serial.print

{{ site.data.alerts.tip }}
For this exercise we're going to use a bit of the Arduino IDE that we haven't used in a while, which is the Serial Monitor. If you don't recall how to open this up, look it up in the <a href="/robot_programming_introduction.html">Robot Programming Introduction</a>.
{{ site.data.alerts.end }}

- `Serial.print` is used to send text data on the USB port.
- You can see what has been printed using the Serial Monitor in the Arduino IDE.
- `Serial.print(number)` will print a number.
- `Serial.print(" ")` will print a space.
- `Serial.println() will give you a new line.

### Program the Exercise

For this exercise we want you to read the values off of the line follower, while running it over the line target. You're going to need to write a short program to do this.

Here's what your program should do.
- Read each of the values from the line follower using the `readAdc` function(s).
- Print each value onto the USB port using the `Serial.print` functions.
- Print a space following each value from the line follower.
- Print a new line at the end of the 8 values.

When you hook this up, the numbers are going to scream past on the screen really quickly, so we suggest adding a `delay` after the for loop to make everything much easier to read.

{{ site.data.alerts.tip }}
<ul>
<li>
The easiest way to implement this is with a for loop and the `int readAdc(byte)` function.
</li>
<li>Count from 0 to 7 using the for loop.</li>
<li>Inside the loop, read the value in the corresponding ADC channel.</li>
<li>`Serial.print` that value and a space.</li>
<li>After the for loop, use `Serial.println`.</li>
</ul>
{{ site.data.alerts.end }}


### Try the Line Follower

- Move the line target around under the robot.
- You should notice that the numbers where the black line is present are different from the numbers where there is no black.
  - How are they different?

{% include callout_red_cup.html task="[Exercise 4.5]" %}

## Next Step

If you have gotten to this point this early in the camp, then you are way ahead of where we expected you to be. Try some of the challenges under [Are You Ahead of the Group?/Optional Challenges and Tutorials](/bonus_tutorial_intro.html)
