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

class gen2bot
{

public:
	gen2bot();

	void sleepApp(int ms)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(ms));
	}

	void spin(int ms, double percOut)
	{
		auger.Set(ControlMode::PercentOutput, percOut);
		sleepApp(ms);
	}

	void unspin(int ms, double percOut)
	{
		auger.Set(ControlMode::PercentOutput, -percOut);
		sleepApp(ms);
	}

	void extend(int ms, double percOut)
	{
		ballScrew.Set(ControlMode::PercentOutput, percOut);
		spin(ms, percOut);
	}

	void deextend(int ms, double percOut)
	{
		ballScrew.Set(ControlMode::PercentOutput, -percOut);
		sleepApp(ms);
	}

	void goDown(int ms, double percOut) 
	{
		linAct.Set(ControlMode::PercentOutput, percOut);
		sleepApp(ms);
	}

	void goUp(int ms, double percOut) 
	{
		linAct.Set(ControlMode::PercentOutput, -percOut);
		sleepApp(ms);
	}
	
	int ms = 1000;
	double percOut = .2;

};
