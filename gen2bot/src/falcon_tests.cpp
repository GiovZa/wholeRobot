// Simple C++ file that doesn't incorporate ROS. Spins motors at a certain hard-coded 
// percent for a specified amount of time. 

#define Phoenix_No_WPI // remove WPI dependencies, we don't use it so just always have it off

// CTRE header files found in /include/gen2bot/ctre/
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"

// Basic includes from C++, string allows string type messages 
// iostream allows cout, which is just a print statement
// chrono and thread is for the sleepApp function, dunno what unistd is
#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <unistd.h>

// CTRE namespaces, these make it so you don't need to put these in each line with functions
// found in ctre/phoenix/...
using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

// Canable labelling
std::string interface = "can0";

// Motor labelling 
TalonFX talLeft(22, interface); 
TalonFX talRght(21);

// Function that runs for specified amount of time in ms to leave a time gap
// between line above and below function call
void sleepApp(int ms)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

// Function that runs on default
int main() 
{	
	// SetInverted(true) sets selected motor to flip its percent ex. 0.7 -> -0.7
	talRght.SetInverted(true);
	
	// Sets specified motor to follow given motor ID. So, talRght now follows
	// talLeft but with inverted inputs due to line 46, needs to be done cuz this motor
	// is oriented in the opposite direction
	talRght.Set(ControlMode::Follower, 22);

	// FeedEnable(setTime) tells motors to run for setTime amount of miliseconds
	ctre::phoenix::unmanaged::Unmanaged::FeedEnable(10000);

	// std::cout prints information and std::endl skips to new line in terminal
	// talLeft.GetSelectedSensorPosition() retrieves position of default sensor
	std::cout << talLeft.GetSelectedSensorPosition() << std::endl;

	// Sets the left motor to spin for a velocity input of 5, thus making right spin at -5
	// from line 46's invert. Dunno units I think it's in ticks
	talLeft.Set(ControlMode::Velocity, 5);
	
	// Print statement with just text
	std::cout << "Running motor for 5 seconds" << std::endl;

	// Wait 5 seconds
	sleepApp(5000);

	std::cout << talLeft.GetSelectedSensorPosition() << std::endl;

	ctre::phoenix::unmanaged::Unmanaged::FeedEnable(10000);

	// Sets left motor to spin at 30% of maximum output
	talLeft.Set(ControlMode::PercentOutput, .3);

	std::cout << talLeft.GetSelectedSensorPosition() << std::endl;

	std::cout << "Running motor 2x for 10 seconds" << std::endl;
	sleepApp(5000);
	talLeft.Set(ControlMode::PercentOutput, 0);

	std::cout << talLeft.GetSelectedSensorPosition() << std::endl;
	std::cout << "Motor off" << std::endl;

	// Return 0 to close program
	return 0;
}
