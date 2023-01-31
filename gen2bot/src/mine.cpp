// Script that runs mining operation
#include <gen2bot/gen2bot.h>
#include <iostream>

void mine(gen2bot robot, int time, double output)
{
	robot.goDown(time, output);
	robot.extend(time, output);
	robot.spin(time, output);
	robot.deextend(time, output);
	robot.goUp(time, output);
	std::cout << "Mining autonomy complete" << std::endl;
	robot.sleepApp(time * 5);
}

void deposit(gen2bot robot, int time, double output)
{
	robot.goUp(time, output);
	robot.extend(time, output);
	robot.unspin(time, output);
	robot.deextend(time, output);
	robot.goDown(time, output);
	std::cout << "Depositing autonomy complete" << std::endl;
}

int main() 
{	
	int time = 1000;
 	double output = 0.2;
	gen2bot robot;

	mine(robot, time, output);

	deposit(robot, time, output);

	return 0;
}
