This src folder has all the C++ scripting code.

To learn how the code works, start with falcon_tests.cpp, then motorTest.cpp, manualDrive.cpp, autoDrive.cpp
and finally miningOperationsTrencherPOL.cpp.cpp and trencherOperationsClass.cpp simultaneously with processManagerClass.cpp

IGNORE ALL OTHER .cpp FILES FOR NOW!!!!! THEY ARE HALF-BAKED GARBAGE THAT WILL PROBABLY BE DELETED

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

wheelDrive.cpp:
  A subscriber node that communicates with the move_base node and robot_mux_control.py to
  give wheel motor control commands autonomously and manually. 

base_trencher_class.cpp:
  Parent class for motors, declares motors and has overarching functions as well as defining variables for each

manual_trencher_class.cpp:
  Child class for motors with sole purpose of manual operations

semi_auto_trencher_class.cpp:
  Child class for motors with sole purpose of operating motor scripts

TODO:
  Make a child class that takes in semi and manual classes so motors have only one class (currently copies same motor for both classes)
  Make abstract code for motors in base class that works for all control modes and checks for all positions reached
  Swap bScrew motor so it doesn't match with a wheel