Ardrone_VISION
==============

Dependencies: ROS, Drone_teleop, openCV, Ardrone_Autonomy
please have these dependencies installed when you are running this program.

you will also need to have a good knowledge of ROS in order to set this program up correctly.

There will eventually be a SETUP.txt file that will help users who are unfamiliar with ROS setup
up the program.

How to use:

This program has two nodes that it runs for controls.

1. controller node:
  This node gives the user the controls to the copter, you can alternate between manual and 
  autonomous flight. The gui will display the navdata of the copter as well as what mode it is 
  currently in, and what commands you are sending to the copter.
  
2. Vison node:
  This node gives you the video feed from the copter, and the color filtration window and configuration
  window. You will have to ability to toggle on and off the circle tracking in real time. The vision node
  is required for autonomous flight mode.
