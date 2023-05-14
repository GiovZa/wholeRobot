This src folder has all the C++ scripting code.

Currently has:

falcon_tests.cpp:
  A simple file that just spins motors by their ID for a specified amount of time with 
  no ROS implementation. Use this to test if motors can move without worrying about ROS.

manualDrive.cpp:
  A subscriber node that communicates with motorTalker.py to 
  give wheel motor control commands manually.

autoDrive.cpp:
  A subscriber node that communicates with the move_base node to
  give wheel motor control commands autonomously. 

augerOperationsClass.cpp and miningOperationsAuger.cpp and processManagerClass.cpp:
  All work with augerOperationsClass.h and processManagerClass.h to control and update the mining and depositing
  motors while also checking if functions ever need to be killed mid script

motorTest.cpp:
  Simple file that gives menu options to check if user can move motors, get desired sensor values, and configure motor PID