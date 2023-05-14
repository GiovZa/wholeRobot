This folder contains the header files for augerOperationsClass.cpp, base_trencher_class.cpp and processManagerClass.cpp

processManagerClass.h checks to see if a process should be killed mid function or not

base_trencher_class.h contains all functions and variables needed for motors

semi_auto_trencher_class.h contains all motor scripts

manual_trencher_class.h contains all manual controls

wheel_trencher_class.h contains a ros class that inherits base_trencher_class to get wheel motors and then run a callback function
to receive a twist topic from robot_mux_control.py that controls drivetrain movement