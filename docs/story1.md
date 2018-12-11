---
layout: default
---
[Main Page](./index.html).
## Story 1: First Pass and Overall Architecture
Before jumping into the full project and attempting to make a full lassie-bot, I wanted to build out the basic behavior and overall architecture. My first pass involved getting the Neato to do a cursory sweep of for a person, approach that person, and navigate back to its starting position.

The code for each of these three behaviors is executed in the `robot_navigation.py`, support code for ROS is implemented in `interface.py`, and person detection code is implemented in `person_tracking.py`.

### Explore
This is the first step in robo-lassie looking for help. Lassie need to first find a person. At this stage this action is simple; Lassie will spin around until she sees a person.

For implementation, the spinning is easily accomplished with previously written velocity publisher that interfaces with  while the person detection I used a Tensorflow zoo mobilnet model for person detection, which will give bounding boxes and confidences. Once a confidence score registers above a set threshold the exploration phase will stop and the Neato will move onto approaching the person.

### Approach

Once lassie has located a potential savior she must get his attention. Like with the exploration phase, this is accomplished via velocity control and mobilenet model detection. In this case, a naive process using simple image logic works well.

In order to navigate towards the target the Neato must first be facing the right direction. Since the camera is placed in the direction the lassie neato is facing, we can use the information from the placement of the bounding box to inform how to orient our pseudo-canine. Using proportional controls, a further left center of the person's bounding box result in quicker left turning for the neato.

With orientation figured out, we may also result to image logic to determine when we are close enough to the target. By determining how large the bounding box is compared to frame, we can approximate how close the individual is to the neato. When that point is reached the Neato turns around and returns to it's starting place.

### Return

Finally the lassie must return to where she started. In order to make the program robust for future complex environments, the Neato won't beeline for her starting coordinates. Instead, in all the previous steps the robot saved its position in order to retrace its steps on the way back.Again a simple controller works in this scenario to check the Neato's progress towards each point along its journey and direct it to the next one when it gets there.
