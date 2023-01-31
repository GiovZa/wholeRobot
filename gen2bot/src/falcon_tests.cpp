// Simple C++ file that doesn't incorporate ROS. Spins motors at a certain percent for a 
// specified amount of time. 

#define Phoenix_No_WPI // remove WPI dependencies

// CTRE header files 
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"

// Basic includes from C++, string allows string type messages 
// iostream allows cout, which is just a print statement
#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <unistd.h>

//CTRE namespaces
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
// between above and below line
void sleepApp(int ms)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}


int main() 
{	
	// SetInverted(true) sets selected motor to flip its percent ex. 0.7 -> -0.7
	talRght.SetInverted(true);
	
	// Sets specified motor to follow given motor ID. 
	talRght.Set(ControlMode::Follower, 22);

	// FeedEnable(setTime) tells motors to run for setTime amount of miliseconds
	ctre::phoenix::unmanaged::Unmanaged::FeedEnable(10000);

	std::cout << talLeft.GetSelectedSensorPosition() << std::endl;

	// Sets the left motor to spin for a velocity input of 5, thus making right spin at -5
	talLeft.Set(ControlMode::Velocity, 5);
	
	// Print statement
	std::cout << "Running motor for 5 seconds" << std::endl;

	// Wait 10 seconds
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
