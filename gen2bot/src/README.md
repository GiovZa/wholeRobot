This src folder has all the C++ scripting code.

Currently has:

base_trencher_class.cpp:
  Parent class for motors, declares motors and has overarching functions as well as defining variables for each

manual_trencher_class.cpp:
  Child class for motors with sole purpose of manual operations

semi_auto_trencher_class.cpp:
  Child class for motors with sole purpose of operating motor scripts

processManagerClass.cpp:
  Updates state of robot with a pointer and callback function to allow user to end a function mid script

robot_mux.cpp:
  Has the main() function that runs all classes and motor subscribers

motorTest.cpp:
  Simple file that gives menu options to check if user can move motors, get desired sensor values, and configure motor PID

TODO:
  Make a child class that takes in semi and manual classes so motors have only one class (currently copies same motor for both classes)
  Make abstract code for motors in base class that works for all control modes and checks for all positions reached
  Swap bScrew motor so it doesn't match with a wheel