---
title: "LED"
tags: [c++ programming]
keywords: LED
summary:
sidebar: tutorials
permalink: led.html
---

## Attaching the LED strip to the robot
Connect the white wire to the ground header (labeled as GND) on the Bot'n Roll and the red wire to the 5V header next to the GND header. Next, take the middle green wire and attatch it to pin number 7 on the board.

## Programming the LEDs
In the top of the file, add `#include <FastLED.h>` and define the pin number and number of LEDs you have on your strip. Create an array of datatype CRGB with the size of the number of your LEDs. This array is what you will modify to alter the color of each LED. 
Within the setup function add the following: `FastLED.addLeds<WS2912, DATA_PIN, GRB>(leds, NUM_LEDS);`

In order to manipulate the LEDs, you simply modify the contents of the CRGB array. For example, to change the color of the first LED to green, we do `led[0].setRGB(0, 255, 0);`. For the RGB values, you can choose a value from 0 to 255. Click [here](https://www.w3schools.com/colors/colors_rgb.asp) for an RGB color picker.

Another way to choose colors is by using one of the many predefined colors, which you can find listed at this [link](https://docs.google.com/document/d/1QY85vLz8qK-xxumCuDploXaDOspO0OfxWeie8CQJsLI/pub). To set the color using this method, you do the following `led[0] = CRGB::AntiqueWhite;`, where you can replace the words after CRGB:: with the color you want.

Although these changes are made to the LED, the only way you can visualize the changes you made to the LED, you must do `FastLED.show()`. Once you upload the code to your robot, you should then be able to see your code work.

With all of this information, you can use the delay function as well as other control structures you've learned to make some really cool things!
