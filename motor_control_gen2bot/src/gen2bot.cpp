#include <motor_control_gen2bot/gen2bot.h>

#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"

using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;


std::string interface = "can0";

TalonSRX linAct(31, interface);
TalonFX ballScrew(32);
TalonFX auger(41);

gen2bot::gen2bot()
	:	ms(1000),
		percOut(.2)
{}

	void gen2bot::sleepApp(int ms)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(ms));
	}

	void gen2bot::spin(int ms, double percOut)
	{
		auger.Set(ControlMode::PercentOutput, percOut);
		sleepApp(ms);
	}

	void gen2bot::unspin(int ms, double percOut)
	{
		auger.Set(ControlMode::PercentOutput, -percOut);
		sleepApp(ms);
	}

	void gen2bot::extend(int ms, double percOut)
	{
		ballScrew.Set(ControlMode::PercentOutput, percOut);
		spin(ms, percOut);
	}

	void gen2bot::deextend(int ms, double percOut)
	{
		ballScrew.Set(ControlMode::PercentOutput, -percOut);
		sleepApp(ms);
	}

	void gen2bot::goDown(int ms, double percOut) 
	{
		linAct.Set(ControlMode::PercentOutput, percOut);
		sleepApp(ms);
	}

	void gen2bot::goUp(int ms, double percOut) 
	{
		linAct.Set(ControlMode::PercentOutput, -percOut);
		sleepApp(ms);
	}
