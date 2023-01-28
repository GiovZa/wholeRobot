This folder has all the C++ code from VEX Motors.

These are all header files which is object orientated programming files that contain the objects.

Each object has their own functions and variables that I call upon in my motorControlGen2Bot/motor_control_gen2bot/src folder. 

Though CTRE has like 1000s of functions, so I am only using a very small amount of their code.

The most relavent files for you are: 

Phoenix.h 
  which provides a lot of namespaces and headers including these two lines:
  #include "ctre/phoenix/motorcontrol/can/TalonFX.h"
  #include "ctre/phoenix/motorcontrol/can/TalonSRX.h"

  Which includes the motor classes we need for programming the motors to move.
  The namespace stuff just makes it so you don't have to provide the path to 
  the function you are using for every line.

ctre/phoenix/unmanaged/Unmanaged.h 
  Which provides the feedEnable() function that just makes it so the motors run 
  for a specified amount of time in miliseconds.

ctre/phoenix/platform/Platform.h 
  Which I think just registers the code to the motors' specific ID and links them to their objects.
  It basically is a middleman from raw motor commands to your C++ code

ctre/phoenix/cci/Unmanaged_CCI.h
  I think its the same with this file, it's a middleman that allows communication 
  between code and motors.
