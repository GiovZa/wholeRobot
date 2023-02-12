This src folder has all the C++ scripting code.

To learn how the code works, start with falcon_tests.cpp, then listenerMotor.cpp, 
listenerMotorAuto.cpp, and finally gen2bot1.cpp and mine1.cpp simultaneously with gen2bot1.h

Currently has:

falcon_tests.cpp:
  A simple file that just spins motors by their ID for a specified amount of time with 
  no ROS implementation. Use this to test if motors can move without worrying about ROS.

listenerMotor.cpp:
  A subscriber node that communicates with motorTalker.py to 
  give wheel motor control commands manually.

listenerMotorAuto.cpp:
  A subscriber node that communicates with the move_base node to
  give wheel motor control commands autonomously. 

gen2bot1.cpp and mine1.cpp:
  Both are linked with gen2bot1.h with mine1.cpp being a node subscriber and carrying out 
  functions that uses the auger, ballscrew, and linear actuator simultaneously

  gen2bot1.cpp holds the class functions from gen2bot1.h and initializes the motors
