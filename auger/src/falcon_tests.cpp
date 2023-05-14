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
TalonFX gio(11, interface); 
TalonSRX talRght(32);
TalonSRX talLeft(31);
TalonSRX talRght1(51);
TalonSRX talLeft1(52);

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
	talLeft.SetInverted(true);
	talRght.SetInverted(true);
	talLeft.SetSensorPhase(true);
	talRght.SetSensorPhase(true);

	talLeft1.SetInverted(true);
	talRght1.SetInverted(true);
	talLeft1.SetSensorPhase(true);
	talRght1.SetSensorPhase(true);
	
	// Sets specified motor to follow given motor ID. So, talRght now follows
	// talLeft but with inverted inputs due to line 46, needs to be done cuz this motor
	// is oriented in the opposite direction
	talRght.Set(ControlMode::Follower, 31);
	talRght1.Set(ControlMode::Follower, 31);
	talLeft1.Set(ControlMode::Follower, 31);

	sleepApp(3000);


	// FeedEnable(setTime) tells motors to run for setTime amount of miliseconds
	ctre::phoenix::unmanaged::Unmanaged::FeedEnable(10000);

	sleepApp(3000);


	// std::cout prints information and std::endl skips to new line in terminal
	// talLeft.GetSelectedSensorPosition() retrieves position of default sensor
	std::cout << talLeft.GetSelectedSensorPosition() << std::endl;
	std::cout << talRght.GetSelectedSensorPosition() << std::endl;
	std::cout << talLeft1.GetSelectedSensorPosition() << std::endl;
	std::cout << talRght1.GetSelectedSensorPosition() << std::endl;


	// Sets the left motor to spin for a velocity input of 5000, thus making right spin at -5000
	// from line 46's invert. Dunno units I think it's in ticks
	talLeft.Set(ControlMode::PercentOutput, -.5);	
	//talRght.Set(ControlMode::PercentOutput, .5);
	
	// Print statement with just text
	std::cout << "Running motor for 5 seconds" << std::endl;

	// Wait 5 seconds
	sleepApp(10000);

	talLeft.SetSelectedSensorPosition(0);
	talRght.SetSelectedSensorPosition(0);
	talLeft1.SetSelectedSensorPosition(0);
	talRght1.SetSelectedSensorPosition(0);

	sleepApp(1000);


	std::cout << talLeft.GetSelectedSensorPosition() << std::endl;
	std::cout << talRght.GetSelectedSensorPosition() << std::endl;
	std::cout << talLeft1.GetSelectedSensorPosition() << std::endl;
	std::cout << talRght1.GetSelectedSensorPosition() << std::endl;

	ctre::phoenix::unmanaged::Unmanaged::FeedEnable(10000);

	// Sets left motor to spin at 30% of maximum output
	talLeft.Set(ControlMode::PercentOutput, .5);
	//talRght.Set(ControlMode::PercentOutput, -.4);

	std::cout << talLeft.GetSelectedSensorPosition() << std::endl;
	std::cout << talRght.GetSelectedSensorPosition() << std::endl;
	std::cout << talLeft1.GetSelectedSensorPosition() << std::endl;
	std::cout << talRght1.GetSelectedSensorPosition() << std::endl;

	std::cout << "Running motor 2x for 10 seconds" << std::endl;
	sleepApp(15000);
	talLeft.Set(ControlMode::PercentOutput, 0);
	//talRght.Set(ControlMode::PercentOutput, 0);

	std::cout << talLeft.GetSelectedSensorPosition() << std::endl;
	std::cout << talRght.GetSelectedSensorPosition() << std::endl;
	std::cout << talLeft1.GetSelectedSensorPosition() << std::endl;
	std::cout << talRght1.GetSelectedSensorPosition() << std::endl;

	std::cout << "Motor off" << std::endl;

	// Sets the left motor to spin until it reaches position 50000, thus making right spin until -5000
	// from line 46's invert. Dunno units I think it's in ticks
	//talLeft.Set(ControlMode::Position, 50000);

	// time included so previous script has time to move motors to position
	/*sleepApp(5000);

	// When setting one ControlMode right after another, the newest one replaces the old one
	// This doesn't spin at 30% output until position 50000 is reached, it just goes to position 50000
	// based on configuration of PID
	ctre::phoenix::unmanaged::Unmanaged::FeedEnable(10000);


	talLeft.Set(ControlMode::PercentOutput, -.3);

	sleepApp(3000);

	std::cout << talLeft.GetSelectedSensorPosition() << std::endl;
	std::cout << talRght.GetSelectedSensorPosition() << std::endl;

	talLeft.Set(ControlMode::PercentOutput, 0);

	sleepApp(1000);

	talLeft.Set(ControlMode::PercentOutput, .3);

	sleepApp(3000);

	std::cout << talLeft.GetSelectedSensorPosition() << std::endl;
	std::cout << talRght.GetSelectedSensorPosition() << std::endl;

	//talLeft.Set(ControlMode::Position, 50000);

	// Return 0 to close program */
	return 0;
}
