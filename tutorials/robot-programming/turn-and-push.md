---
title: "Turn and Push"
tags: [robot programming]
keywords:
sidebar: tutorials
permalink: turn_and_push.html
---


```
#define SSPIN  2    //Slave Select (SS) pin for SPI communication
#define M1  1       //Motor1
#define M2  2       //Motor2

#define Vtrans 300 //Line follower limit between white and black

float batmin=10.5; // safety voltage for discharging the battery
int vel=40;


void setup()
{  
  Serial.begin(57600);     // sets baud rate to 57600bps for printing values at serial monitor.
  one.spiConnect(SSPIN);   // starts the SPI communication module   
  one.stop();              // stop motors
  one.minBat(batmin);      // safety voltage for discharging the battery
  delay(1000);
}

```

None of the setup stuff is special for this program. `vel` is used later, and was a hardcoded value provided by Bnr. I suspect it could be made to go faster.


```
void loop()
{
  int line=readLine();

```

From a high-level perspective, all readLine does is return a value from -100 to 100 that represents where the line is, according to the sensor. I'll explain it more when I get to that part in the code.

```  
  double k=0.45;//Linear gain to control motor speed <> Ganho linear para controlar a velocidade dos motores

```

This value came directly from the Bnr code, and is the scaling factor for line. I assume they fine-tuned this parameter. It's worth noting that there might be a better number than this. The only constraint on the value for `k` is that `vel + line*k =< 100` and `vel - line*k >= -100`. Technically, none of the variables have to have any other contraints, but it seems intuitive to keep `-100 <= line <= 100`

```
  double velM1=(double)vel+((double)line*k);
  double velM2=(double)vel-((double)line*k);
  one.move((int)velM1,(int)velM2);

}
```

Here's where the actual proportional control is made. The only part about these lines that I haven't already mentioned that I think is important is that one of the values adds `line*k` whereas the other subtracts. Since the car is turning, one value has to be subtracted while the other needs to be added to ensure that the car turns. Technically it doesn't matter which is added and which is subtracted, but in the opposite case than what's written, the values being added to `lineValue` in `readLine()` will all need to have their signs flipped.

```
int readLine()
{
    int lineValue=0;
    int sensorCount=0;

```

`lineValue` is set to zero, since the default assumption is that the car is following the line.

```
    if(one.readAdc(0)>Vtrans) //Test Sensor1  <>  Testa o sensor1
      {                       
        lineValue-=100;
        sensorCount++;
      }
    if(one.readAdc(1)>Vtrans) //Test Sensor2  <>  Testa o sensor2
      {                       
        lineValue-=75;
        sensorCount++;
      }
    if(one.readAdc(2)>Vtrans)
      {
        lineValue-=50;
        sensorCount++;
      }
    if(one.readAdc(3)>Vtrans)
      {
        lineValue-=25;
        sensorCount++;
      }
    if(one.readAdc(4)>Vtrans)
      {
        lineValue+=25;
        sensorCount++;
      }
    if(one.readAdc(5)>Vtrans)
      {
        lineValue+=50;
        sensorCount++;
      }
    if(one.readAdc(6)>Vtrans)
      {
        lineValue+=75;
        sensorCount++;
      }
    if(one.readAdc(7)>Vtrans) //Test Sensor8  <>  Testa o sensor8
      {                       
        lineValue+=100;
        sensorCount++;
      }

```

Here's where all the sensors are evaluated. Due to how `velM1` and `velM2` are set, this function uses negative values to represent the line being towards the left, and positive values to represent the line being towards the right.

What the code is doing is checking to see which sensors are registering a dark line below them. The threshold value, `Vtrans`, was provided by Bnr, but other values may likely work.

The values being added to `lineValue` have the following idea in mind: the larger the value returned by readLine, the farther the robot needs to turn in order to get back on the line. I assume that the extreme values (-100, 100) were conjured up due to the limits on the `move()` function. From there, The number of sensors makes it such that incrementing by 25 makes the most sense. Since there is no middle sensor, there is no case that adds zero. Since the values being added to `lineValue` are opposites for opposite sensors, however, a line (no matter how thick) that is in the middle (or that is covering all of the sensors) will have `readLine()` return zero.

```
    if(sensorCount>0)
        lineValue=lineValue/sensorCount;
    return lineValue;
}
```

Finally, `lineValue` is divided by the number of sensors that registered above the threshold before being returned. This makes sense because if more than one sensor registers above the theshold, then the "center" of that line is technically between the two sensors. That is to say that if the two far left sensors registered above the threshold, we would not want to return -175, but rather 87.5



`lineValue` is originally set to zero, since the base assumption is that the car is driving straight on the line. From there, `readLine` goes through each sensor and checks to see if its above some threshold.

`readAdc(byte)` returns values from 0-1023. In their code, they specify 300 as the threshold for saying something is black vs white





## Turn and Push
Now that you understand the basics, let's start programming the robot! Your first task will be to implement obstacle detection and avoidance. This will involve monitoring the robot's sensors, and adjusting your movement accordingly to ensure that the robot does not get trapped. We'll be opting to do a navigation method called turn and push. You've probably seen this method before - it's what Roombas do to avoid collisions as well! The method goes as follows:

- If the robot sees an obstacle on one side, back up, turn towards the clear side, and continue forwards
- If the robot sees obstacles on both sides, back up

### History of Turn and Push

Grey Walter was a neuroscientist, who in the late 1940s created the first autonomous robots, Elsie and Elmer. These robots were phototropic, meaning they followed light, and were also sensitive to touch. These two sensory systems combined together with a motor helped him create "behavior" for these robots, to the point where they could move across a room autonomously. Because of the slow movement of the robots, Walter called them tortoises and believed they taught us the secrets to the organization of life. This movement behavior is now known as the turn and push behavior which we want to create to avoid obstacles. Below is an image of the path of one of Walter's tortoises. You can see the turn and push behavior in the path.

![Tortoises](images/turn_and_push.png)

```
int sensor = one.obstacleSensors();
```

The `obstacleSensors()` function returns an `int` based on what the robot's front sensors are detecting. If you receive 0, no obstacles are being detected. If you receive 1, then an obstacle is being detected on the left sensor. Receiving 2 indicates that an obstacle has been detected on the right sensor. Finally, receiving a 3 means that obstacles were detected on both the right and left sensors.

## Task 8.4

You should now have all of the pieces to write the turn and push behavior! Your first major programming task is now to implement that behavior. You'll be testing out your code on an L-shaped barrier, and you may to assume that you'll be turning left.

{% include callout_red_cup.html task="X" comment="Please flip your cup to red to indicate that you're ready to test your robot."%}

{% include note.html content="Camp Staff: Bring the group to a wall or out to the bridge to test that the robot both drives forward and stops when encountering the wall, using its obstacle detection sensors." %}


Now, let's get the robot to [follow a line](line_following.html).
