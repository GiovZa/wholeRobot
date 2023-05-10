// Class that runs manual mining operation

// all other includes are in this file:
#include <gen2bot/manual_trencher_class.h>

// This line uses the constructor from base_trencher_class to do all configurations for us
manual_trencher_class::manual_trencher_class(ros::NodeHandle nh) : base_trencher_class(nh) {}

	// end of recursion
	void spinMotors(){}

	// this template takes any type and number of motors and sets a percent output
	// use case: spinMotors(linAct1, .5, linAct2, .5);, which spins both motors at .5 percent
	template<typename Motor, typename Speed, typename... Args>
	void spinMotors(Motor& motor, Speed speed, Args&&... args)
	{
		// Makes sure all motors have a speed variable attached
    	static_assert(sizeof...(args)%2 == 0, "Error: number of arguments is odd");

		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		motor.Set(ControlMode::PercentOutput, speed);

		// Recursion call with all arguments minus the first 2 that are already used, which was just used in line above
		spinMotors(args...);
	}

	// For each function from manual_trencher_class, we must pass in p_cmd so that we can constantly
	// check if it changes inside the function and if so, kill the function mid-run
	// After function ends, put p_cmd in default value so process knows function has ended		
	void manual_trencher_class::keepSpinningMotors(int& p_cmd, ros::NodeHandle  nh)
	{
		while (p_cmd == sentinel)
		{
			ROS_INFO("Inside While loop");
			ROS_INFO("p_cmd: %d", p_cmd);
			ROS_INFO("sentinel: %d", sentinel);
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}
	}

	// output results
	void manual_trencher_class::motorsStoppedSpinning(int& p_cmd, ros::NodeHandle  nh)
	{
		ROS_INFO("Motor(s) shut off");
		ROS_INFO("p_cmd out of loop: %d", p_cmd);
		ROS_INFO("sentinel out of loop: %d", sentinel);
	}

	void manual_trencher_class::rightLinActBack(int& p_cmd, ros::NodeHandle  nh) 
	{
		sentinel = p_cmd;
		ROS_INFO("rightLinActBackClass");

		spinMotors(linAct1, -0.5);
		keepSpinningMotors(p_cmd, nh);
		spinMotors(linAct1, 0);
		motorsStoppedSpinning(p_cmd, nh);
	}

	void manual_trencher_class::rightLinActForward(int& p_cmd, ros::NodeHandle  nh) 
	{
		sentinel = p_cmd;
		ROS_INFO("rightLinActForwardClass");

		spinMotors(linAct1, 0.5);
		keepSpinningMotors(p_cmd, nh);
		spinMotors(linAct1, 0);
		motorsStoppedSpinning(p_cmd, nh);
	}

	void manual_trencher_class::rightBucketForward(int& p_cmd, ros::NodeHandle  nh)
	{
		sentinel = p_cmd;
		
		ROS_INFO("rightBucketForwardClass");

		spinMotors(bucket1, 0.6);
		keepSpinningMotors(p_cmd, nh);
		spinMotors(bucket1, 0);
		motorsStoppedSpinning(p_cmd, nh);
	}

	void manual_trencher_class::rightBucketBack(int& p_cmd, ros::NodeHandle  nh)
	{
		sentinel = p_cmd;
		ROS_INFO("rightBucketBackClass");

		spinMotors(bucket1, -0.6);
		keepSpinningMotors(p_cmd, nh);
		spinMotors(bucket1, 0);
		motorsStoppedSpinning(p_cmd, nh);
	}

	void manual_trencher_class::spinScoops(int& p_cmd, ros::NodeHandle nh)
	{
		sentinel = p_cmd;
		ROS_INFO("spinScoopsClass");

		spinMotors(trencher, 0.4);
		keepSpinningMotors(p_cmd, nh);
		spinMotors(trencher, 0);
		motorsStoppedSpinning(p_cmd, nh);
	}

	void manual_trencher_class::leftLinActBack(int& p_cmd, ros::NodeHandle nh)
	{
		sentinel = p_cmd;
		ROS_INFO("leftLinActBackClass");

		spinMotors(linAct2, -0.5);
		keepSpinningMotors(p_cmd, nh);
		spinMotors(linAct2, 0);
		motorsStoppedSpinning(p_cmd, nh);
	}

	void manual_trencher_class::leftLinActForward(int& p_cmd, ros::NodeHandle nh)
	{
		sentinel = p_cmd;
		ROS_INFO("leftLinActForwardClass");

		spinMotors(linAct2, 0.5);
		keepSpinningMotors(p_cmd, nh);
		spinMotors(linAct2, 0);
		motorsStoppedSpinning(p_cmd, nh);
	}

	void manual_trencher_class::leftBucketForward(int& p_cmd, ros::NodeHandle nh)
	{
		sentinel = p_cmd;
		ROS_INFO("leftBucketForwardClass");

		spinMotors(bucket2, 0.6);
		keepSpinningMotors(p_cmd, nh);
		spinMotors(bucket2, 0);
		motorsStoppedSpinning(p_cmd, nh);
	}

	void manual_trencher_class::leftBucketBack(int& p_cmd, ros::NodeHandle nh)
	{
		sentinel = p_cmd;
		ROS_INFO("leftBucketBackClass");
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

		spinMotors(bucket2, -0.6);
		keepSpinningMotors(p_cmd, nh);
		spinMotors(bucket2, 0);
		motorsStoppedSpinning(p_cmd, nh);
	}

	void manual_trencher_class::ballScrewIn(int& p_cmd, ros::NodeHandle nh)
	{
		sentinel = p_cmd;
		ROS_INFO("ballScrewInClass");
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

		spinMotors(bScrew, 0.8);
		keepSpinningMotors(p_cmd, nh);
		spinMotors(bScrew, 0);
		motorsStoppedSpinning(p_cmd, nh);
	}

	void manual_trencher_class::ballScrewOut(int& p_cmd, ros::NodeHandle nh)
	{
		sentinel = p_cmd;
		ROS_INFO("ballScrewOutClass");

		spinMotors(bScrew, -0.8);
		keepSpinningMotors(p_cmd, nh);
		spinMotors(bScrew, 0);
		motorsStoppedSpinning(p_cmd, nh);
	}

	void manual_trencher_class::scoopsBScrew(int& p_cmd, ros::NodeHandle nh)
	{
		sentinel = p_cmd;
		ROS_INFO("scoopsBScrewClass");

		spinMotors(bScrew, 0.8, trencher, 0.6);
		keepSpinningMotors(p_cmd, nh);
		spinMotors(bScrew, 0, trencher, 0);
		motorsStoppedSpinning(p_cmd, nh);
	}

	void manual_trencher_class::linActsForward(int& p_cmd, ros::NodeHandle nh)
	{
		sentinel = p_cmd;
		ROS_INFO("linActsForwardClass");

		spinMotors(linAct1, 0.5, linAct2, 0.5);
		keepSpinningMotors(p_cmd, nh);
		spinMotors(linAct1, 0, linAct2, 0);
		motorsStoppedSpinning(p_cmd, nh);
	}

	void manual_trencher_class::linActsBack(int& p_cmd, ros::NodeHandle nh)
	{
		sentinel = p_cmd;
		ROS_INFO("linActsBackClass");

		spinMotors(linAct1, -0.5, linAct2, -0.5);
		keepSpinningMotors(p_cmd, nh);
		spinMotors(linAct1, 0, linAct2, 0);
		motorsStoppedSpinning(p_cmd, nh);
	}

	void manual_trencher_class::bucketsForward(int& p_cmd, ros::NodeHandle nh)
	{
		sentinel = p_cmd;
		ROS_INFO("bucketsForwardClass");

		spinMotors(bucket1, 0.4, bucket2, 0.4);
		keepSpinningMotors(p_cmd, nh);
		spinMotors(bucket1, 0, bucket2, 0);
		motorsStoppedSpinning(p_cmd, nh);
	}

	void manual_trencher_class::bucketsBack(int& p_cmd, ros::NodeHandle nh)
	{
		sentinel = p_cmd;
		ROS_INFO("bucketsBackClass");

		spinMotors(bucket1, -0.4, bucket2, -0.4);
		keepSpinningMotors(p_cmd, nh);
		spinMotors(bucket1, 0, bucket2, 0);
		motorsStoppedSpinning(p_cmd, nh);
	}
