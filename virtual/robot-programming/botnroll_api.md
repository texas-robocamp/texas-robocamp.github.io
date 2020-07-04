## Documentation 

We've gone ahead and included a separate page with all the documentation for using the robot [here](docs.html), but we'll be introducing you to the various functions our robot as they become useful. Feel free to come back to this page or the documentation to check up on what these functions do.

## `LEDs`

```cpp
void ledLeft(boolean state)
```

```cpp
void ledRight(boolean state)
```

These functions turn on and off the left and right LEDs respectively, based on whether it is passed `true` or `false`. 

## `lcd#`

You already saw the `lcd#` function in the "Hello World" exercise on the robot. One other important feature of this function we should mention is that it can take any number of arguments - for example,

```cpp
bot.lcd1("Bob is ", 10, " years old!");
```

will print out "Bob is 10 years old!"


## Obstacle Sensors

```cpp
double leftObstacleSensor()
```

```cpp
double RightObstacleSensor()
```

These functions will return the distance (in meters) from the left or right obstacle sensor to the nearest obstacle. Each sensor is on the front of the robot, pointing out 45 degrees. The range for these values is [0.0,10.0]

### Exercise 4.2.1

**TODO** *Are we doing thresholding then? I'm thinkning for here we just tell them to print if sensor > 5.0 but then in the next exercise we have them figure out good thresholding later.*

- Start by copying the "empty" program from ["Robot Programming Introduction"](/robot_programming_introduction.html) into your ~~Arduino IDE~~, and saving it in a sensible place.
- Write a short program that will print "Left Sensor Activated" when the left sensor is activated, "Right Sensor Activated" when the right sensor is activated, and "Both Sensors Activated" when both sensors are activated on the LCD on the robot.
  - For this program, you'll be launching a new world. This launch file uses the same robocamp package, but the file name is now **TODO** *filename*

{% include callout_red_cup.html task="[Exercises 4.2.1 - 4.2.5]" %}


### Exercise 4.2.2

- Instead of just printing which sensor is activated, turn on the corresponding LED to indicate which obstacle sensor has been triggered.

## Next Step

Proceed to ["Move The Robot"](move_the_robot.html)
