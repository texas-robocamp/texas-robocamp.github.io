This page contains a list of all the functions you'll be using when working with the robot this week. As programmers, we call these kinds of pages **Documentation**, because they document what everything means and how it is used. Feel free to refer back to this page to check up on what functions might be useful for your programs as you perform the exercies.

## Opening VSCode via the CLI

To open a file in VSCode through your command line interface, do

```
code <file>
```

where *file* is the name of the file you would like to open.

You can also do this with directories!


## C++ Documentation

### Compiling C++ Code

```
g++ <file_name> -o <executable_name>
```

For example, if you have a file called `HelloWorld.cpp`, and want to compile it into an executable called `Hello`, you would do:

```
g++ HelloWorld.cpp -o Hello
```

### Logical Operators

| Logical Operator | Meaning             | Example of True Statement |
|---|---|:---:|
|`>`   | greater than                 | `5 > 4`                 |
|`>=`   | greater than or equal to     | `5 >= 5`                |
|`<`   | less than	                   | `2 < 5`                 |
|`<=`   | less than or equal to        | `2 <= 3`                |
|`==`   | is exactly the same as       | `3 == 3`                |
|`!=`   | is not the same as           | `3 != 5`                |

### Conditional Operators

| Conditional Operator | Meaning | Example of True Statement |
| :--- | :--- | :---: |
| `&&` | Both conditions must be true | `(2 > 1) && (5 != 0)` |
| `||` | Either condition must be true | `(1 > 2) || (5 != 0)` |
| `!` | True if the condition is false | `!(1 > 2)` |


### int usleep(useconds_t useconds)

This function pauses the program for `useconds` milliseconds. 


## Robot Documentation
These are the functions available to you for working with the robot:

### void move(float speedL, float speedR)

This function takes in speeds for the left and right wheels to drive the robot. Speeds are limited to [-100, 100]. Passing in zero for both speeds will result in stopping the robot.

### double leftObstacleSensor()

Returns the distance between the left obstacle sensor and the nearest obstacle. Output is from [0.1,10.0]

### double rightObstacleSensor()

Returns the distance between the right obstacle sensor and the nearest obstacle. Output is from [0.1,10.0]

### int readLineSensor(int sensorNum)

Returns the sensor value currently being read at the `sensorNum` [0-7] position of the line sensor. The value returned is from [0,255].

### int readButton()

Returns the value of which button was most recently pressed.
Possible return values:

---| ---
0 | no button pressed
1 | Button 1 pressed
2 | Button 2 pressed
3 | Button 3 pressed

### void ledLeft(bool on)

Sets the state of the left LED based on the value of `on`. True turns on the LED, false turns it off.

### void ledRight(bool on)

Sets the state of the right LED based on the value of `on`. True turns on the LED, false turns it off.

### void lcd1(const std::string &string)

Prints `string` to the top text on the GUI

### void lcd2(const std::string &string)

Prints `string` to the bottom text on the GUI

## ROS Documentation

### Roslaunch

Roslaunch commands look like this:

```
roslaunch <package_name> <launch_file>
```

Here's one example, which launches the basic world:

```
roslaunch robocamp empty_camp.launch
```
**TODO** *placeholder*

### Rosrun

Rosrun commands look like this:

```
rosrun <package_name> <node_name>
```

One common rosrun you'll probably be using is

```
rosrun robocamp teleop_robot
```

Our generator script `create-package` will always make the name of your node `node`, so when you run your code, you'll do something like

```
rosrun ex_6_1_2 node
```


