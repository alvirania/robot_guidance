# robot_guidance
Group Project

Objective:
The purpose of this project was to develop a robust navigation system that allowed the robot to autonomously traverse the maze, fulfilling objectives such as following a guidance line, navigating S turns, making informed decisions at junctions, identifying and re-routing at dead ends using the front bumper, and reaching the final destination.

Description:
The project involved programming the eebot mobile robot to navigate a maze autonomously. The given maze set the stage for a series of tasks. Starting at the entry point, the robot followed a guidance line and negotiated S turns to validate the guidance algorithm. At junctions, it made informed decisions, and upon encountering dead ends, it executed a 180-degree turn once the front bumper was activated, and chose the alternate branch eventually reaching the maze's forward destination.

Our project code implemented a state machine approach to navigate the maze. Based on the sensor readings of the robot, the robot transitions into one of the following states: start, forward, left, right, reverse, backtrack, and standby. The behaviour of the most common states have been described below.

When certain condition(s) are met, the corresponding state is initialized and the current state (CRNT_STATE) is updated to reflect the state. For example, when the eebot is in the start state (START_ST), it checks for the ‘NOT FWD_BUMP’ condition and once it is satisfied it initializes the forward state (FWD_ST) and the CRNT_STATE is updated to FWD_ST. While in FWD_ST, it checks for several conditions including forwards/backward movement and left/right turns. To navigate the ‘S’ curve, the difference between E and F sensor readings are compared to the E and F threshold values to adjust the position on the line by either shifting left or right.

Similarly, when the eebot enters either the left turn state (LT_TRN_ST) or the right turn state (RT_TRN_ST), it verifies that the distance traveled exceeds the predefined threshold (STR_DIS) for straight movement. Once this condition is met, it turns in the specified direction until the turn distance threshold (TRN_DIS) is met. In the case of encountering an obstacle that occurs when a forward bump is detected, the eebot initiates a u-turn and it does so by entering the reverse state (REV_ST) first. The eebot turns until the distance traveled exceeds the threshold value for the u-turn distance (UTRN_DIS).

Despite challenges in tracking the line and fine-tuning turn delays, the robot learned from incorrect choices, adjusted its path, and successfully reached the final destination. Thus, this project enhanced our programming skills and understanding of navigation algorithms for autonomous systems.

Problems:
While working on this project, our group encountered many problems. Listed below are a few issues that we encountered and how we overcame them.

Robot Unable to Track Line:
After completing our first code version, and testing the robot, it didn’t track the line very well.  We thought this was due to the speed of the robot as it was rolling pretty fast and it seemed like the robot was rolling so fast, the sensors couldn’t keep up.  Whenever the robot went through the ‘S’ curve at the beginning of the maze, it would execute a right turn and then left, instead of making small adjustments to follow the line.  We eventually had to re-measure the threshold values for the sensors and make the robot roll forward incrementally.  The robot tracked the line better but still not to the degree we wanted and we eventually realized that our sensor E and F threshold values were flipped.  After flipping them back to where they should have been, the robot tracked the line much better.

Robot Rear Bumper not Stopping Robot:
We wanted to implement a subroutine similar to the ALL_STOP subroutine in the previous labs.  We had already programmed a backtrack subroutine and ended up re-purposing the method to the all-stop method.  Since the backtrack methods were already implemented all over the code, we just changed the backtrack subroutine and kept the name the same so it wouldn’t tamper with the rest of the code.  

Delay Values:
We had to take lots of time to experiment with the delay values for the U-turn and the 90-degree turns.  At first, the robot overturned for the U-turn and under-turned for the 90-degree turns.  The over-turn from the U-turn didn’t mess up the robot's line-tracking ability, however, the under-turn in the 90-degree turn caused it to execute 2 of the same turn.  We ended up reducing the U-turn delay and increasing the 90-degree turn delay incrementally until the turns were executed for the correct time. 
