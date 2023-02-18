This src folder has all the C++ scripting code.

To learn how the code works, start with falcon_tests.cpp, then listenerMotor.cpp, 
listenerMotorAuto.cpp, and finally notDT.cpp and NotDTClass.cpp simultaneously with ProcessManager.cpp

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

notDT.cpp and notDTClass.cpp and ProcessManager.cpp:
  All work with NotDTClass.h and Processmanager.h to control and update the mining and depositing
  motors while also checking if functions ever need to be killed mid script
