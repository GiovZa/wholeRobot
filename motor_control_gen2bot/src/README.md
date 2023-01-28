This src folder has all the C++ scripting code.

Currently has:

falcon_tests.cpp:
  A simple file that just spins motors by their ID for a 
  specified amount of time with no ROS implementation.

listenerMotor.cpp:
  A subscriber node that communicates with motorTalker.py to 
  give motor control commands manually.

listenerMotorAuto.cpp:
  A subscriber node that communicates with the move_base node to
  give motor control commands autonomously.


