---
title: "Maze Solving"
tags: [c++ programming]
keywords: maze algorithm
last_updated: July 5, 2018
summary:
sidebar: tutorials
permalink: maze_solving.html
---
## Maze Solving
Your next task is to create a maze-solving algorithm. The maze in which the robots will be tested is *simply connected*, which means that all its walls will be connected together. In order to traverse the maze, the robot will be traveling along the line, so you will need to expand on the line follower program you created in the previous step.

## Develop a maze solving rule
You want your robot to be able to solve any maze we give it, so before you start programming, you need to develop a general rule for our robot to follow that will always lead it to the end of the maze. This will give you a good sense of what you are trying to develop.
Look at the example maze below, and use it to help you develop the rule you want your robot to follow. Whatever rule you do create, it probably will not be the fastest way through the maze, but it should eventually get through the maze.

![Sample Maze](images/sample_maze1.png)

## Design of the Maze
The maze is built with black lines so you can build off of your line follower code. The maze will have both T-intesections and four way intersections. In these cases, which way should the robot go first? Try to make the robot turn in the same direction, whichever you choose, every time it is faced with this choice. The end of the maze will be marked by a black rectangle, at which point the robot must stop.

## Task X
Once you have developed a general rule, it's time to start programming and then testing!

<div style="background-color:rgba(255,0,0,0.5)">
<b>RED CUP: Track Progress [Task X]</b>
<ul>
<li>Please flip your cup to red.</li><li>A camp staff member will bring you to a maze to test your robot with.</li>
<li>Feel free to run your robot at your desk while you wait, but try not to create chaos in doing so.</li>
  <li>Camp Staff: Bring the group to one of the mazes and ensure that the robot is able to complete the maze.</li>
</ul>
</div>
