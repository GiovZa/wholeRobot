# motor_control_gen2bot

This package has all the motor code.

To control the robot with an Xbox controller:

Terminal 1: roscore

Terminal 2: rosrun motor_control_gen2bot listenerMotor

Terminal 3: rosrun joy joy_node

Terminal 4: rosrun motor_control_gen2bot motorTalker.py 

