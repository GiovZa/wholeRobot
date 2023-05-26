// Classes that runs motor scripts

// There is a ton of checks to see if sentinel = p_cmd because we must be able to kill the motors at any point in any function

#include <gen2bot/semi_auto_trencher_class.h>

semi_auto_trencher_class::semi_auto_trencher_class(ros::NodeHandle nh) 
	: 
		base_trencher_class(nh),
		mBuffer(10),
		wheelBuffer(100),
		bScrewBuffer(10000),
		trencherBuffer(15000),
		linActSpeed(10),
		bucketSpeed(10),
		bScrewSpeed(-40000),
		bScrewSpeedDown(-10000),
		scoopsSpeed(100000),
		scoopsPercent(.35),
		bScrewPercent(-0.15)
		{speedUpdate(nh);}

	// Makes sure the linear actuators of linAct and bucket aren't moving when they are misaligned.
	void semi_auto_trencher_class::isSafe(int& p_cmd)
	{	
		sentinel = p_cmd;

		if (!(isNear(linAct1.GetSelectedSensorPosition(), linAct2.GetSelectedSensorPosition(), 5)) 
			|| !(isNear(bucket1.GetSelectedSensorPosition(), bucket2.GetSelectedSensorPosition(), 5)))
		base_trencher_class::stopMotors();
	}

	// Updates speeds of motors
	void semi_auto_trencher_class::speedUpdate(ros::NodeHandle nh)
	{	
		nh.getParam("/linact_cfg/motionCruiseVelocity", linActSpeed);
		nh.getParam("/bucket_cfg/motionCruiseVelocity", bucketSpeed);
		nh.getParam("/bscrew_cfg/motionCruiseVelocity", bScrewSpeed);
		nh.getParam("/bscrew_cfg/motionCruiseVelocityDown", bScrewSpeedDown);
		nh.getParam("/bscrew_cfg/percentOutput", bScrewPercent);
		nh.getParam("/trencher_cfg/motionCruiseVelocity", scoopsSpeed);
		nh.getParam("/trencher_cfg/percentOutput", scoopsPercent);
		nh.getParam("/trencherBuffer", trencherBuffer);
		nh.getParam("/bScrewBuffer", bScrewBuffer);
		nh.getParam("/wheelBuffer", wheelBuffer);
		nh.getParam("/mBuffer", mBuffer);
	}

	// Velocity and acceleration should always be positive inputs. Position can be positive or negative.
	void semi_auto_trencher_class::ConfigMotionMagic(TalonFX* talon1, int vel, int accel, int pos)
	{
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

		talon1->ConfigMotionAcceleration(abs(accel));
		talon1->ConfigMotionCruiseVelocity(abs(vel));
		
		talon1->Set(ControlMode::MotionMagic, pos);

		std::cout << "kP: " <<bScrewMM.slot0.kP << std::endl;
		std::cout << "kI: " << bScrewMM.slot0.kI << std::endl;
		std::cout << "kD: " << bScrewMM.slot0.kD << std::endl;
		std::cout << "A TalonFX motor is moving" << std::endl;
	    std::cout << "Desired Velocity: " << vel << std::endl;
	    std::cout << "Desired Accleration: " << accel << std::endl;
	    std::cout << "Desired Position: " << pos << std::endl;

	}

	void semi_auto_trencher_class::ConfigMotionMagic(TalonSRX* talon1, TalonSRX* talon2, int vel, int accel, int pos)
	{
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

		talon1->ConfigMotionAcceleration(abs(accel));
		talon2->ConfigMotionAcceleration(abs(accel));

		talon1->ConfigMotionCruiseVelocity(abs(vel));
		talon2->ConfigMotionCruiseVelocity(abs(vel));

		talon1->Set(ControlMode::MotionMagic, pos);
		talon2->Set(ControlMode::MotionMagic, pos);

	    std::cout << "A pair of TalonSRX motors are moving" << std::endl;

	}

	void semi_auto_trencher_class::Jitter(TalonSRX* talon1, TalonSRX* talon2, int num, std::string name, int& p_cmd, ros::NodeHandle  nh)
	{
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

		std::cout << name << " are JITTERING " << num << " times" << std::endl;

		std::this_thread::sleep_for(std::chrono::milliseconds(500));

		for (int i = 0; i < num; i++)
		{
			talon1->Set(ControlMode::PercentOutput, 1); 
			talon2->Set(ControlMode::PercentOutput, 1); 

			std::this_thread::sleep_for(std::chrono::milliseconds(50));

			talon2->Set(ControlMode::PercentOutput, -1); 
			talon1->Set(ControlMode::PercentOutput, -1); 

			std::this_thread::sleep_for(std::chrono::milliseconds(25));

		}

		talon2->Set(ControlMode::PercentOutput, 0); 
		talon1->Set(ControlMode::PercentOutput, 0); 

	}

	void semi_auto_trencher_class::displayData()
	{
		displayData(&bucket1, &bucket2, "bucket");
		displayData(&linAct1, &linAct2, "linAct");
		displayData(&bScrew, "bScrew");
	}

	void semi_auto_trencher_class::displayData(TalonFX* talon1, std::string name)
	{
		std::cout << name << " Position: " << talon1->GetSelectedSensorPosition() << std::endl;
		std::cout << name << " Velocity: " << talon1->GetSelectedSensorVelocity(0) << std::endl;
	}

	void semi_auto_trencher_class::displayData(TalonSRX* talon1, TalonSRX* talon2, std::string name)
	{
		std::cout << name << "1 Position: " << talon1->GetSelectedSensorPosition() << std::endl;
		std::cout << name << "2 Position: " << talon2->GetSelectedSensorPosition() << std::endl;
		std::cout << name << "1 Velocity: " << talon1->GetSelectedSensorVelocity(0) << std::endl;
		std::cout << name << "2 Velocity: " << talon2->GetSelectedSensorVelocity(0) << std::endl;
	}

	bool semi_auto_trencher_class::ReverseLimitSwitchTriggered(TalonFX* talon1, std::string name)
	{
		
		if (talon1->GetSelectedSensorVelocity(0) == 0) 
		{
			std::cout << "Reverse limit switch for "<< name << " triggered." << std::endl;
			
			ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
			talon1->Set(ControlMode::Velocity, 0);
			talon1->SetSelectedSensorPosition(0);

			return true;
		}
		return false;
	}

	bool semi_auto_trencher_class::ReverseLimitSwitchTriggered(TalonSRX* talon1, TalonSRX* talon2, std::string name)
	{
		if (talon1->GetSelectedSensorVelocity(0) == 0 && talon2->GetSelectedSensorVelocity(0) == 0) 
		{
			std::cout << "Reverse limit switch for "<< name << "1 triggered." << std::endl;
			std::cout << "Reverse limit switch for "<< name << "2 triggered." << std::endl;

			talon1->Set(ControlMode::Velocity, 0);
			talon2->Set(ControlMode::Velocity, 0);
			talon1->SetSelectedSensorPosition(0);
			talon2->SetSelectedSensorPosition(0);
			
			return true;
		}
		return false;
	}

	bool semi_auto_trencher_class::isNear(int a, int b, int tolerance) 
	{
		if (abs(a - b) <= tolerance)
			std::cout << "true" << std::endl;
		else
		{
			std::cout << "false" << std::endl;
			std::cout << "Current Position: " << a << std::endl;
			std::cout << "Wanted Position: " << b << std::endl;
			std::cout << "Tolerance: " << tolerance << std::endl;
		}
		return abs(a - b) <= tolerance;
	}

	bool semi_auto_trencher_class::TargetPositionReached(TalonFX* talon1, int pos, int buffer, std::string name)
	{
		if (isNear(talon1->GetSelectedSensorPosition(), pos, buffer))
		{
			std::cout << name << " is in desired position." << std::endl;
			return true;
		}
		return false;
	}

	// Alex said why 2 couts with same input?
	bool semi_auto_trencher_class::TargetPositionReached(TalonSRX* talon1, TalonSRX* talon2, int pos, std::string name)
	{
		if (isNear(talon1->GetSelectedSensorPosition(), pos, mBuffer) && isNear(talon2->GetSelectedSensorPosition(), pos, mBuffer))
		{
			std::cout << name << " is in desired position." << std::endl;
			std::cout << name << " is in desired position." << std::endl;
			return true;
		}
		return false;
	}

	bool semi_auto_trencher_class::CheckMode(int laPos, int buPos, int bsPos)
	{
		if (isNear(linAct1.GetSelectedSensorPosition(), laPos, mBuffer) &&  isNear(linAct2.GetSelectedSensorPosition(), laPos, mBuffer)
			&& (isNear(bucket1.GetSelectedSensorPosition(), buPos, mBuffer) ||  isNear(bucket2.GetSelectedSensorPosition(), buPos, mBuffer))
			&& isNear(bScrew.GetSelectedSensorPosition(), bsPos, bScrewBuffer))
			return true;
		return false;
	}

	void semi_auto_trencher_class::zeroStart(int& p_cmd, ros::NodeHandle  nh)
	{
		std::cout << "Running zeroStart" << std::endl;
		displayData();

		sentinel = p_cmd;

		base_trencher_class::config(nh);

		linAct1.SetSelectedSensorPosition(0);
		linAct2.SetSelectedSensorPosition(0);
		bucket1.SetSelectedSensorPosition(0);
		bucket2.SetSelectedSensorPosition(0);
		bScrew.SetSelectedSensorPosition(0);
		trencher.SetSelectedSensorPosition(0);

		std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		std::cout << "POSITIONS ARE ZEROED:" << std::endl;
		displayData();

		base_trencher_class::stopMotors();
	}

	// Reassigns absolute position so motors know where they are
	void semi_auto_trencher_class::zero(int& p_cmd, ros::NodeHandle  nh) 
	{
		sentinel = p_cmd;
		std::cout << "\nRunning zero\n" << std::endl;
		zeroStart(p_cmd, nh);
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		
		// ZERO THE LINEAR ACTUATOR
		std::cout << "Zeroing the Linear Actuator" << std::endl;

		ConfigMotionMagic(&linAct1, &linAct2, linActSpeed, 5, -800);
		std::this_thread::sleep_for(std::chrono::milliseconds(2000));

		do {
			displayData(&linAct1, &linAct2, "linAct");

			if(base_trencher_class::exitFunction(p_cmd)) return;

			std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		} while(!ReverseLimitSwitchTriggered(&linAct1, &linAct2, "linAct"));

		std::cout << "Finished zeroing the linAct" << std::endl;
		// Move the linAct to dig position so that the bScrew and bucket can be zeroed
		while (!TargetPositionReached(&linAct1, &linAct2, laDigPosition, "linAct")){
			std::cout << "Moving to dig position: " << laDigPosition << std::endl;

			ConfigMotionMagic(&linAct1, &linAct2, linActSpeed, 5, laDigPosition);

			displayData(&linAct1, &linAct2, "linAct");

			if(base_trencher_class::exitFunction(p_cmd)) return;

			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}
		
		std::cout << "LINACT FINAL: " << std::endl;
		displayData(&linAct1, &linAct2, "linAct");
		
		// ZERO THE BUCKET
		std::cout << "Zeroing the bucket" << std::endl;

		ConfigMotionMagic(&bucket1, &bucket2, bucketSpeed, 5, -800);

		do 
		{
			displayData(&bucket1, &bucket2, "bucket");

			if(base_trencher_class::exitFunction(p_cmd)) return;

			std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		} while(!ReverseLimitSwitchTriggered(&bucket1, &bucket2, "bucket"));

		// Move the bucket to drive position
		while (!TargetPositionReached(&bucket1, &bucket2, buDrivePosition, "bucket")){
			std::cout << "Moving bucket to position: " << buDrivePosition << std::endl;

			ConfigMotionMagic(&bucket1, &bucket2, bucketSpeed, 5, buDrivePosition);

			displayData(&bucket1, &bucket2, "bucket");

			if(base_trencher_class::exitFunction(p_cmd)) return;

			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}

		std::cout << "BUCKET FINAL: " << std::endl;
		displayData(&bucket1, &bucket2, "bucket");

		// ZERO THE BALL SCREW

		std::cout << "Zeroing the bScrew" << std::endl;

		ConfigMotionMagic(&bScrew, bScrewSpeed, 5, -9720000);

		do 
		{	
			displayData(&bScrew, "bScrew");

			if(base_trencher_class::exitFunction(p_cmd)) return;

			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		
		} while(!ReverseLimitSwitchTriggered(&bScrew, "bScrew"));


		std::cout << "BSCREW FINAL: " << std::endl;
		displayData(&bScrew, "bScrew");

		// Move the Linear Actuator back to drive position
		while(!TargetPositionReached(&linAct1, &linAct2, laDrivePosition, "linAct")){
			
			ConfigMotionMagic(&linAct1, &linAct2, linActSpeed, 5, laDrivePosition);

			displayData(&linAct1, &linAct2, "linAct");

			if(base_trencher_class::exitFunction(p_cmd)) return;

			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}

		std::cout << "LINACT FINAL: " << std::endl;
		displayData(&linAct1, &linAct2, "linAct");

		base_trencher_class::stopMotors();
		std::cout << std::endl << std::endl << std::endl << std::endl;

	}

	// trencher is all the way tucked in and parallel to ground
	void semi_auto_trencher_class::driveMode(int& p_cmd, ros::NodeHandle  nh) 
	{
		sentinel = p_cmd;
		
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		std::cout << "Moving to driveMode: " << std::endl;

		displayData();
	
		trencher.Set(ControlMode::PercentOutput, 0);

		if(base_trencher_class::exitFunction(p_cmd)) return;

		std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		// While not in drive mode,
		while(!CheckMode(laDrivePosition, buDrivePosition, bsDrivePosition))
		{
		
			ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

			trencher.Set(ControlMode::PercentOutput, 0);

			// bucket moves first
			ConfigMotionMagic(&bucket1, &bucket2, bucketSpeed, 5, buDrivePosition);
			if(base_trencher_class::exitFunction(p_cmd)) return;

			std::this_thread::sleep_for(std::chrono::milliseconds(1000));

			if (!(TargetPositionReached(&bScrew, bsDrivePosition, bScrewBuffer, "bScrew")))
			{
				std::cout << "Moving bScrew up to bsDrivePosition." << std::endl;
				bScrew.Set(ControlMode::PercentOutput, 0.5);
				std::cout << "po speed: " << 0.5 << std::endl;


				if(base_trencher_class::exitFunction(p_cmd)) return;
			}	
			
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));


			// If the ball screw retracted and bucket in drive position, start moving the linAct
			if (TargetPositionReached(&bScrew, bsDrivePosition, bScrewBuffer, "bScrew") 
				&& (isNear(bucket1.GetSelectedSensorPosition(), buDrivePosition, mBuffer) ||  isNear(bucket2.GetSelectedSensorPosition(), buDrivePosition, mBuffer)))
			{
				ConfigMotionMagic(&linAct1, &linAct2, linActSpeed, 5, laDrivePosition);
				std::this_thread::sleep_for(std::chrono::milliseconds(3000));

			}

			if(base_trencher_class::exitFunction(p_cmd)) return;

			displayData();

			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}
		
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		base_trencher_class::stopMotors();

		std::cout<< "\n\ndriveMode reached.\n" << std::endl;
		
	}

	/* void semi_auto_trencher_class::deposit(int& p_cmd, ros::NodeHandle  nh)
	{
		sentinel = p_cmd;
		speedUpdate(nh);
		base_trencher_class::config(nh);

		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
			
		std::cout << "Moving to depositMode: " << std::endl;

		displayData();

		
		// while not in deposit position.
		while(!CheckMode(laDepositPosition, buDepositPosition, bsDepositPosition))
		{
			// Move the linAct first
			ConfigMotionMagic(&linAct1, &linAct2, linActSpeed, 5, laDepositPosition);
			if(base_trencher_class::exitFunction(p_cmd)) return;

			// Move the bucket when the linAct has reached deposit position
			if (TargetPositionReached(&linAct1, &linAct2, laDepositPosition, "linAct"))
			{
				trencher.Set(ControlMode::PercentOutput, scoopsSpeed);
				
				ConfigMotionMagic(&bucket1, &bucket2, bucketSpeed, 5, buDepositPosition);
			}

			if(base_trencher_class::exitFunction(p_cmd)) return;

			std::this_thread::sleep_for(std::chrono::milliseconds(1000));

			displayData();
		}

		trencher.Set(ControlMode::PercentOutput, 0);
		// For now, no timer to let gravel fall into sieve, if mech says otherwise, uncomment
		std::this_thread::sleep_for(std::chrono::milliseconds(3000));

		// Move back to drive position
		driveMode(p_cmd, nh);

		if(base_trencher_class::exitFunction(p_cmd)) return;

		displayData();

		std::this_thread::sleep_for(std::chrono::milliseconds(3000));
		
		base_trencher_class::stopMotors();

		std::cout<< "depositMode reached." << std::endl;

		std::cout << std::endl << std::endl << std::endl << std::endl;

	} */

	void semi_auto_trencher_class::dig(int& p_cmd, ros::NodeHandle  nh)
	{
		sentinel = p_cmd;
		speedUpdate(nh);
		base_trencher_class::config(nh);
		std::cout << "Moving to digMode: " << std::endl;

		int initialDigPosition = laDigPosition - 150;

		// Move the bucket
		ConfigMotionMagic(&bucket1, &bucket2, bucketSpeed, 5, buDigPosition);
		if(base_trencher_class::exitFunction(p_cmd)) return;
		
		while(!CheckMode(initialDigPosition, buDigPosition, bsDrivePosition))
		{
			// Move the linAct first
			std::cout << "linAct moving to initial dig position (SLANTED)" << std::endl;

			ConfigMotionMagic(&linAct1, &linAct2, linActSpeed, 5, initialDigPosition);
			if(base_trencher_class::exitFunction(p_cmd)) return;
			
			displayData();
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		}

		std::cout << "linAct has reached initialDigPosition (slanted)" << std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));


		// When linAct has reached initialDigPosition (slanted), turn on trencher
		if (TargetPositionReached(&linAct1, &linAct2, initialDigPosition, "linAct"))
		{
			std::cout << "Turning on trencher" << std::endl;
			ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

			bool checkerPO;

			nh.getParam("/trencherPO", checkerPO);

			std::cout << "checkerPO: " << checkerPO << std::endl;
			if (checkerPO)
			{
				trencher.Set(ControlMode::PercentOutput, (-1 * scoopsPercent));
				std::cout << "po trencher speed: " << scoopsPercent << std::endl;
			}
			else
			{
				trencher.Set(ControlMode::Velocity, scoopsSpeed);
				std::cout << "vel trencher speed: " << scoopsSpeed << std::endl;
			}

			displayData(&bScrew, "bScrew");
			displayData(&trencher, "trencher");

			if(base_trencher_class::exitFunction(p_cmd)) return;
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}

		// While not in digMode, and slowly go to laDigPosition
		while(!CheckMode(laDigPosition, buDigPosition, bsDigPosition))
		{
			std::cout << "linAct moving to final dig position (VERTICAL)" << std::endl;
			ConfigMotionMagic(&linAct1, &linAct2, linActSpeed - 8, 2, laDigPosition);
			if(base_trencher_class::exitFunction(p_cmd)) return;
			
			// When linAct has reached laDigPosition (vertical), extend the ball screw into the ground.
			// The trencher will still be moving.
			if (TargetPositionReached(&linAct1, &linAct2, laDigPosition, "linAct"))
			{
				std::cout << "Extending bScrew" << std::endl;
				ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

				bool checkerPO;
				nh.getParam("/bscrewPO", checkerPO);

				std::cout << "checkerPO: " << checkerPO << std::endl;
				if (checkerPO)
				{
					bScrew.Set(ControlMode::PercentOutput, bScrewPercent);
					std::cout << "po speed: " << bScrewPercent << std::endl;
				}
				else
				{
					bScrew.Set(ControlMode::Velocity, bScrewSpeedDown);
					std::cout << "vel bs speed: " << bScrewSpeedDown << std::endl;
				}
				
				// bScrew.Set(ControlMode::PercentOutput, .3);
				//ConfigMotionMagic(&bScrew, bScrewSpeed, 100000, bsDigPosition);
				
				if(base_trencher_class::exitFunction(p_cmd)) return;

			}

			displayData();
			displayData(&bScrew, "bScrew");
			displayData(&trencher, "trencher");
			if(base_trencher_class::exitFunction(p_cmd)) return;
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}

		std::cout<< "digMode reached." << std::endl;

		std::this_thread::sleep_for(std::chrono::milliseconds(3000));

		std::cout << "Cycling the trencher while bScrew goes up" << std::endl;
		while(!(TargetPositionReached(&bScrew, bsDrivePosition, bScrewBuffer, "bScrew")))
		{
			std::cout << "bScrew going up. Trencher cycling." << std::endl;
			
			bScrew.Set(ControlMode::Velocity, bScrewSpeed);

			bool checkerPO;

			nh.getParam("/trencherPO", checkerPO);

			std::cout << "checkerPO: " << checkerPO << std::endl;
			if (checkerPO)
			{
				trencher.Set(ControlMode::PercentOutput, (-1 * scoopsPercent/2));
				std::cout << "po trencher speed: " << scoopsPercent/2 << std::endl;
			}
			else
			{
				trencher.Set(ControlMode::Velocity, (scoopsSpeed/2));
				std::cout << "vel trencher speed: " << scoopsSpeed/2 << std::endl;
			}

			displayData(&bScrew, "bScrew");
			displayData(&trencher, "trencher");

			if(base_trencher_class::exitFunction(p_cmd)) return;

			std::this_thread::sleep_for(std::chrono::milliseconds(500));

		}

		std:: cout << "Trencher cycling stopped" << std::endl;

		std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		// Move back to drive mode
		driveMode(p_cmd, nh);

		if(base_trencher_class::exitFunction(p_cmd)) return;

		displayData(); 
		std::this_thread::sleep_for(std::chrono::milliseconds(3000));
		
		base_trencher_class::stopMotors();

		std::cout<< "\n\ndigMode COMPLETED.\n" << std::endl;

		

	}

	void semi_auto_trencher_class::spinAround(int& p_cmd)
	{
		rightWheel.SetInverted(true);
		std::chrono::steady_clock::time_point startTime = std::chrono::steady_clock::now();
		std::chrono::milliseconds duration(3000);
		int x = 0;
		while (x <= 5)
		{
			if(base_trencher_class::exitFunction(p_cmd)) return;
			while (std::chrono::steady_clock::now() - startTime < duration)
			{
				leftWheel.Set(ControlMode::PercentOutput, .20);
				rightWheel.Set(ControlMode::PercentOutput, -.15);
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(5000));
			x += 1;
		}
	}

	void semi_auto_trencher_class::deposit(int& p_cmd, ros::NodeHandle  nh)
	{
		sentinel = p_cmd;
		speedUpdate(nh);
		base_trencher_class::config(nh);

		std::cout << "\nStarting depositMode with PercentOutput.\n" << std::endl;

		std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		
		linAct1.Set(ControlMode::PercentOutput, 1);
		linAct2.Set(ControlMode::PercentOutput, 1);

		for(int i = 0; i < 10; i++)
		{
			displayData();
			if(base_trencher_class::exitFunction(p_cmd))
			{
				return;
			} 
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}

		linAct1.Set(ControlMode::PercentOutput, 0);
		linAct2.Set(ControlMode::PercentOutput, 0);

		// Extend the bScrew to jack up the robot.
		bScrew.Set(ControlMode::PercentOutput, -1);

		for(int i = 0; i < 10; i++)
		{
			displayData();

			if(base_trencher_class::exitFunction(p_cmd)) return;

			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}


		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

		bScrew.Set(ControlMode::PercentOutput, 0);

		std::this_thread::sleep_for(std::chrono::milliseconds(2000));

		bucket1.Set(ControlMode::PercentOutput, 1);
		bucket2.Set(ControlMode::PercentOutput, 1);		

		for(int i = 0; i < 10; i++)
		{
			displayData();

			if(base_trencher_class::exitFunction(p_cmd)) return;
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}

		Jitter(&bucket1, &bucket2, 60, "buckets", p_cmd, nh);

		std::this_thread::sleep_for(std::chrono::milliseconds(1000));


		bucket1.Set(ControlMode::PercentOutput, -1);
		bucket2.Set(ControlMode::PercentOutput, -1);	

		for(int i = 0; i < 10; i++)
		{
			displayData();
			if(base_trencher_class::exitFunction(p_cmd)) return;
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}

		bucket1.Set(ControlMode::PercentOutput, 0);
		bucket2.Set(ControlMode::PercentOutput, 0);	

		bScrew.Set(ControlMode::PercentOutput, 1);

		for(int i = 0; i < 10; i++)
		{
			displayData();
			if(base_trencher_class::exitFunction(p_cmd)) return;
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}

		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

		bScrew.Set(ControlMode::PercentOutput, 0);

		linAct1.Set(ControlMode::PercentOutput, -1);
		linAct2.Set(ControlMode::PercentOutput, -1);

		for(int i = 0; i < 10; i++)
		{
			displayData();
			if(base_trencher_class::exitFunction(p_cmd)) return;
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}

		linAct1.Set(ControlMode::PercentOutput, 0);
		linAct2.Set(ControlMode::PercentOutput, 0);

		displayData();

		base_trencher_class::stopMotors();

		std::cout << "depositMode Completed." << std::endl;

	}
