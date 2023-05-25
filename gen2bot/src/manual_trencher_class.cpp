// Class that runs manual mining operation

// all other includes are in this file:
#include <gen2bot/manual_trencher_class.h>

// This line uses the constructor from base_trencher_class to do all configurations for us
manual_trencher_class::manual_trencher_class(ros::NodeHandle nh) 
	: 
		base_trencher_class(nh),
	  	linActSpeed(.5),
		bucketSpeed(.5),
		bScrewSpeed(.5),
		scoopsSpeed(.5)
	   {speedUpdate(nh);}

	// Updates speeds of motors
	void manual_trencher_class::speedUpdate(ros::NodeHandle nh)
	{	
		nh.getParam("/linact_cfg/percentOutput", linActSpeed);
		nh.getParam("/bucket_cfg/percentOutput", bucketSpeed);
		nh.getParam("/bscrew_cfg/percentOutput", bScrewSpeed);
		nh.getParam("/trencher_cfg/percentOutput", scoopsSpeed);
	}

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

		speedUpdate(nh);
		spinMotors(linAct1, (-1 * linActSpeed));
		keepSpinningMotors(p_cmd, nh);
		spinMotors(linAct1, 0);
		motorsStoppedSpinning(p_cmd, nh);
	}

	void manual_trencher_class::rightLinActForward(int& p_cmd, ros::NodeHandle  nh) 
	{
		sentinel = p_cmd;
		ROS_INFO("rightLinActForwardClass");

		speedUpdate(nh);
		spinMotors(linAct1, linActSpeed);
		keepSpinningMotors(p_cmd, nh);
		spinMotors(linAct1, 0);
		motorsStoppedSpinning(p_cmd, nh);
	}

	void manual_trencher_class::rightBucketForward(int& p_cmd, ros::NodeHandle  nh)
	{
		sentinel = p_cmd;
		
		ROS_INFO("rightBucketForwardClass");

		speedUpdate(nh);
		spinMotors(bucket1, bucketSpeed);
		keepSpinningMotors(p_cmd, nh);
		spinMotors(bucket1, 0);
		motorsStoppedSpinning(p_cmd, nh);
	}

	void manual_trencher_class::rightBucketBack(int& p_cmd, ros::NodeHandle  nh)
	{
		sentinel = p_cmd;
		ROS_INFO("rightBucketBackClass");

		speedUpdate(nh);
		spinMotors(bucket1, (-1 * bucketSpeed));
		keepSpinningMotors(p_cmd, nh);
		spinMotors(bucket1, 0);
		motorsStoppedSpinning(p_cmd, nh);
	}

	void manual_trencher_class::scoopsForward(int& p_cmd, ros::NodeHandle nh)
	{
		sentinel = p_cmd;
		ROS_INFO("spinScoopsForwardClass");

		speedUpdate(nh);
		spinMotors(trencher, scoopsSpeed);
		keepSpinningMotors(p_cmd, nh);
		spinMotors(trencher, 0);
		motorsStoppedSpinning(p_cmd, nh);
	}

	void manual_trencher_class::scoopsBack(int& p_cmd, ros::NodeHandle nh)
	{
		sentinel = p_cmd;
		ROS_INFO("spinScoopsBackClass");
		
		speedUpdate(nh);
		spinMotors(trencher, (-1 * scoopsSpeed));
		keepSpinningMotors(p_cmd, nh);
		spinMotors(trencher, 0);
		motorsStoppedSpinning(p_cmd, nh);
	}

	void manual_trencher_class::leftLinActBack(int& p_cmd, ros::NodeHandle nh)
	{
		sentinel = p_cmd;
		ROS_INFO("leftLinActBackClass");

		speedUpdate(nh);
		spinMotors(linAct2, (-1 * linActSpeed));
		keepSpinningMotors(p_cmd, nh);
		spinMotors(linAct2, 0);
		motorsStoppedSpinning(p_cmd, nh);
	}

	void manual_trencher_class::leftLinActForward(int& p_cmd, ros::NodeHandle nh)
	{
		sentinel = p_cmd;
		ROS_INFO("leftLinActForwardClass");

		speedUpdate(nh);
		spinMotors(linAct2, linActSpeed);
		keepSpinningMotors(p_cmd, nh);
		spinMotors(linAct2, 0);
		motorsStoppedSpinning(p_cmd, nh);
	}

	void manual_trencher_class::leftBucketForward(int& p_cmd, ros::NodeHandle nh)
	{
		sentinel = p_cmd;
		ROS_INFO("leftBucketForwardClass");

		speedUpdate(nh);
		spinMotors(bucket2, bucketSpeed);
		keepSpinningMotors(p_cmd, nh);
		spinMotors(bucket2, 0);
		motorsStoppedSpinning(p_cmd, nh);
	}

	void manual_trencher_class::leftBucketBack(int& p_cmd, ros::NodeHandle nh)
	{
		sentinel = p_cmd;
		ROS_INFO("leftBucketBackClass");
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

		speedUpdate(nh);
		spinMotors(bucket2, (-1 * bucketSpeed));
		keepSpinningMotors(p_cmd, nh);
		spinMotors(bucket2, 0);
		motorsStoppedSpinning(p_cmd, nh);
	}

	void manual_trencher_class::ballScrewIn(int& p_cmd, ros::NodeHandle nh)
	{
		sentinel = p_cmd;
		ROS_INFO("ballScrewInClass");
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

		speedUpdate(nh);
		spinMotors(bScrew, bScrewSpeed);
		keepSpinningMotors(p_cmd, nh);
		spinMotors(bScrew, 0);
		motorsStoppedSpinning(p_cmd, nh);
	}

	void manual_trencher_class::ballScrewOut(int& p_cmd, ros::NodeHandle nh)
	{
		sentinel = p_cmd;
		ROS_INFO("ballScrewOutClass");

		speedUpdate(nh);
		spinMotors(bScrew, (-1 * bScrewSpeed));
		keepSpinningMotors(p_cmd, nh);
		spinMotors(bScrew, 0);
		motorsStoppedSpinning(p_cmd, nh);
	}

	void manual_trencher_class::scoopsBScrew(int& p_cmd, ros::NodeHandle nh)
	{
		sentinel = p_cmd;
		ROS_INFO("scoopsBScrewClass");

		speedUpdate(nh);
		spinMotors(bScrew, (-1 * bScrewSpeed), trencher, scoopsSpeed);
		keepSpinningMotors(p_cmd, nh);
		spinMotors(bScrew, 0, trencher, 0);
		motorsStoppedSpinning(p_cmd, nh);
	}

	void manual_trencher_class::linActsForward(int& p_cmd, ros::NodeHandle nh)
	{
		sentinel = p_cmd;
		ROS_INFO("linActsForwardClass");

		ROS_INFO("linActSpeed: %d", linActSpeed);

		speedUpdate(nh);

		ROS_INFO("linActSpeed: %d", linActSpeed);

		spinMotors(linAct1, linActSpeed, linAct2, linActSpeed);
		keepSpinningMotors(p_cmd, nh);
		spinMotors(linAct1, 0, linAct2, 0);
		motorsStoppedSpinning(p_cmd, nh);
	}

	void manual_trencher_class::linActsBack(int& p_cmd, ros::NodeHandle nh)
	{
		sentinel = p_cmd;
		ROS_INFO("linActsBackClass");

		speedUpdate(nh);
		spinMotors(linAct1, (-1 * linActSpeed), linAct2, (-1 * linActSpeed));
		keepSpinningMotors(p_cmd, nh);
		spinMotors(linAct1, 0, linAct2, 0);
		motorsStoppedSpinning(p_cmd, nh);
	}

	void manual_trencher_class::bucketsForward(int& p_cmd, ros::NodeHandle nh)
	{
		sentinel = p_cmd;
		ROS_INFO("bucketsForwardClass");

		speedUpdate(nh);
		spinMotors(bucket1, bucketSpeed, bucket2, bucketSpeed);
		keepSpinningMotors(p_cmd, nh);
		spinMotors(bucket1, 0, bucket2, 0);
		motorsStoppedSpinning(p_cmd, nh);
	}

	void manual_trencher_class::bucketsBack(int& p_cmd, ros::NodeHandle nh)
	{
		sentinel = p_cmd;
		ROS_INFO("bucketsBackClass");

		speedUpdate(nh);
		spinMotors(bucket1, (-1 * bucketSpeed), bucket2, (-1 * bucketSpeed));
		keepSpinningMotors(p_cmd, nh);
		spinMotors(bucket1, 0, bucket2, 0);
		motorsStoppedSpinning(p_cmd, nh);
	}

	void manual_trencher_class::bucketsTrencher(int& p_cmd, ros::NodeHandle nh)
	{
		sentinel = p_cmd;
		ROS_INFO("bucketsBackClass");

		speedUpdate(nh);
		spinMotors(bucket1, bucketSpeed, bucket2, bucketSpeed, trencher, scoopsSpeed);
		keepSpinningMotors(p_cmd, nh);
		spinMotors(bucket1, 0, bucket2, 0, trencher, 0);
		motorsStoppedSpinning(p_cmd, nh);
	}