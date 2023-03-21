This src folder has all the C++ scripting code.

To learn how the code works, start with falcon_tests.cpp, then listenerMotor.cpp, listenerMotorAuto.cpp, 
and finally miningOperationsAuger.cpp and augerOperationsClass.cpp simultaneously with processManagerClass.cpp

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

trencherOperationsClass.cpp and miningOperationsTrencherAuto.cpp and processManagerClass.cpp:
  Still being tested on but will eventually make the new robot fully autonomous with its motor functions working
