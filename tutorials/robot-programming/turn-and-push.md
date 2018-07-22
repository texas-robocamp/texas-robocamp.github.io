---
title: "Turn and Push"
tags: [robot programming]
keywords:
sidebar: tutorials
permalink: turn_and_push.html
---

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
