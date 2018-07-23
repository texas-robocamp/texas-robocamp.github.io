

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
