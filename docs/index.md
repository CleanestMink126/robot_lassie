---
layout: default
---
# Overview
The Lassie-bot is a multi-stage environment mapping and person finding feature for the [Neato robot vacuum](https://www.neatorobotics.com/) with connected raspberry pi and camera for Olin ENGR3590. The main purpose of the lassie-bot is given odometry, laser scan, and camera data, explore the environment with the purpose of finding humans in order to lead them back to the starting position. This is based on the canonical behavior where Lassie runs to go find help after seeing someone trapped and returns to their rescue.

For the purposes of implementation, I broke this behavior down into 3 stages: explore, approach, and return.

Check out [blog 1](./story1.html) to see a first pass inplementation of each of these, or read on to see the final implementation.

## Explore

<center>
<img src="./assets/images/explore.jpg" alt="drawing" width="500"/>
</center>

The entire purpose of explore is to find a human. Unfortunately there is no universal metric as to how to achieve this. For the purposes of this project, I decided to define exploration as going to the farthest away places that we have confirmed do not contain an obstacle.

This analysis can still be further broken down into 2 more parts: creation of the environment map and traversal of this map to find good coordinates.

Checkout [blog 2](./story2.html) to see how I went about creating the map.

Here's a look at the mapping in action:
<center>
<img src="./assets/gifs/1.gif" alt="drawing" width="500"/>
</center>

Once I have a (theoretically) sufficient map of the environment, I can do helpful inference on this map with breadth first search algorithms such as [Dijkstra's Algorithm applied to maps ](http://www.roguebasin.com/index.php?title=Dijkstra_Maps_Visualized). This allows me to calculate distances from anything to anything in a varying amount of conditions. For this application, I calculate distance from any viable point to the nearest point where the Neato has been and filter these paths via a minimal threshold distance from the nearest object. The next goal is selected as the furthest away point (within reason, so not too far away) that is not close to an obstacle.

Here's an example of me controlling the Neato while it weights the environment (shown in yellow) and sets a waypoint (shown in white).
<center>
<img src="./assets/gifs/2.gif" alt="drawing" width="500"/>
</center>

Once a goal is decided, the robot can then use another graph traversal algorithm to find the best and safest route to the goal according to the maps the we defined above. The gif belows shows an example of the Neato maneuvering itself to the next goal in order to more explore the map.

<center>
<img src="./assets/gifs/3.gif" alt="drawing" width="500"/>
</center>

## Approach

<center>
<img src="./assets/images/approach.jpg" alt="drawing" width="500"/>
</center>

## Return

<center>
<img src="./assets/images/return.jpg" alt="drawing" width="500"/>
</center>
