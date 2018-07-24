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

### Binary Code

Oddly enough, now is a good time to discuss binary.

We all know that computers represent data as 1s and 0s, but what does that really mean? Well, it's about to become immediately relevant to your ability to program this robot.

Binary is called a numerical base.

Decimal is the base that we are used to, where there are 10 digits (0-9), and we add additional digits to make bigger numbers.

In binary, there are two digits (0 & 1), and we add additional digits to make bigger numbers.

Binary was chosen as the numerical system for computers because computers use transistors, which can be thought of as switches which only go on or off, to represent basically everything. Since a switch can only be on or off, this naturally maps to there being two values for each digit in the computer's number system. Hence binary.

Each switch, a 1 or 0 is called a bit.

8 bits make a byte.

{{ site.data.alerts.tip }}
<ul>
<li>If you have heard of 8-bit, 16-bit, 32-bit, or 64-bit computer architectures, this refers to the number of bits used to represent an integer and other basic things in the computer.</li>
<li>Notice that the lengths are all powers of two. This is no mistake. Using powers of two is typically more efficient when designing things for computers.</li>
</ul>
{{ site.data.alerts.end }}

We need to know a couple more things before we can understand binary. We need to quickly understand exponents.

Left-Hand-Side    | Equals
--------|-------------
2	| 2
2 * 2	    |4
2 * 2 * 2	| 8
2 * 2 * 2 * 2	| 16

We can also state these as exponents, which just means multiplying the number by itself as many times as is in the exponent.

Left-Hand-Side    | Exponent    | Equals
--------|-------------|-------------
2	| 2<sup>1</sup>	| 2
2 * 2	    | 2<sup>2</sup>	    |4
2 * 2 * 2	| 2<sup>3</sup>	| 8
2 * 2 * 2 * 2	| 2<sup>4</sup>	| 16

Okay, but here's one which will really blow your mind if you haven't seen it before.

 Exponent    | Equals
-------------|-------------
2<sup>0</sup>	| 1

{{ site.data.alerts.tip }}
Explaining this would really get the camp off-course, but if you are curious, you can learn more <a href="http://scienceline.ucsb.edu/getkey.php?key=2626">here</a>.
{{ site.data.alerts.end }}

