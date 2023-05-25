#pragma once

// This class is the base class for all essential motor declarations, functions, and variables
#include <gen2bot/base_trencher_class.h>

// This class runs all motor scripts
class semi_auto_trencher_class : public base_trencher_class
{

public:
    semi_auto_trencher_class(ros::NodeHandle nh);

    void ConfigMotionMagic(TalonFX* talon1, int vel, int accel, int pos);
    void ConfigMotionMagic(TalonSRX* talon1, TalonSRX* talon2, int vel, int accel, int pos);

    void Jitter(TalonSRX* talon1, TalonSRX* talon2, int num, std::string name, int& p_cmd, ros::NodeHandle  nh);

    void displayData();
    void displayData(TalonFX* talon1, std::string name);
    void displayData(TalonSRX* talon1, TalonSRX* talon2, std::string name);

    bool ReverseLimitSwitchTriggered(TalonFX* talon1, std::string name);
    bool ReverseLimitSwitchTriggered(TalonSRX* talon1, TalonSRX* talon2, std::string name);

    bool isNear(int a, int b, int tolerance);

    bool TargetPositionReached(TalonFX* talon1, int pos, int buffer, std::string name);
    bool TargetPositionReached(TalonSRX* talon1, TalonSRX* talon2, int pos, std::string name);

    bool CheckMode(int laPos, int buPos, int bsPos);

    void zeroStart(int& p_cmd, ros::NodeHandle  nh);
    void zero(int& p_cmd, ros::NodeHandle  nh);

    void driveMode(int& p_cmd, ros::NodeHandle  nh);
    void deposit(int& p_cmd, ros::NodeHandle  nh);
    void dig(int& p_cmd, ros::NodeHandle  nh);
	void spinAround(int& p_cmd);

    void isSafe(int& p_cmd);
    void speedUpdate(ros::NodeHandle  nh);

    void turnTrencher(int& p_cmd, ros::NodeHandle  nh);

    int mBuffer;
    int wheelBuffer;
    int bScrewBuffer;
    int trencherBuffer;

    double linActSpeed;
    double bucketSpeed;
    double bScrewSpeed;
    double bScrewSpeedDown;
    double scoopsSpeed;
    double bScrewPercent;
};
