#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"
#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <unistd.h>

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;


std::string interface = "can0";

TalonFX talLeft(22, interface); 
TalonFX talRght(21);

void sleepApp(int ms)
{
	std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

int main() 
{	
	talRght.SetInverted(true);
	talRght.Set(ControlMode::Follower, 22);
	// ctre::phoenix::unmanaged::Unmanaged::FeedEnable(10000);
	talLeft.Set(ControlMode::Velocity, 5);
	// talRght.Set(ControlMode::PercentOutput, .2);
	std::cout << "Running motor for 10 seconds" << std::endl;
	sleepApp(1000);
	ctre::phoenix::unmanaged::Unmanaged::FeedEnable(10000);
	talLeft.Set(ControlMode::PercentOutput, .1);
	// talRght.Set(ControlMode::PercentOutput, .4);
	std::cout << "Running motor 2x for 10 seconds" << std::endl;
	sleepApp(100000);
	talLeft.Set(ControlMode::PercentOutput, 0);
	std::cout << "Motor off" << std::endl;

	return 0;
}
