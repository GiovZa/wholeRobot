#pragma once

class gen2bot
{

public:
	gen2bot();

	void sleepApp(int ms);

	void spin(int ms, double percOut);

	void unspin(int ms, double percOut);

	void extend(int ms, double percOut);
	void deextend(int ms, double percOut);

	void goDown(int ms, double percOut);
	void goUp(int ms, double percOut);
	
	int ms = 1000;
	double percOut = .2;

};
