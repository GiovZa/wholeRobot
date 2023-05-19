// Classes that runs motor scripts

// There is a ton of checks to see if sentinel = p_cmd because we must be able to kill the motors at any point in any function

#include <gen2bot/semi_auto_trencher_class.h>

semi_auto_trencher_class::semi_auto_trencher_class(ros::NodeHandle nh) : base_trencher_class(nh), mBuffer(10){}

	// Makes sure the linear actuators of linAct and bucket aren't moving when they are misaligned.
	void semi_auto_trencher_class::isSafe(int& p_cmd)
	{	
		sentinel = p_cmd;

		if (!(isNear(linAct1.GetSelectedSensorPosition(), linAct2.GetSelectedSensorPosition(), 5)) 
			|| !(isNear(bucket1.GetSelectedSensorPosition(), bucket2.GetSelectedSensorPosition(), 5)))
		stopMotors();
	}

	// Velocity and acceleration should always be positive inputs. Position can be positive or negative.
	void semi_auto_trencher_class::ConfigMotionMagic(TalonFX* talon1, int vel, int accel, int pos)
	{
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

		talon1->ConfigMotionAcceleration(abs(accel));
		talon1->ConfigMotionCruiseVelocity(abs(vel));
		
		talon1->Set(ControlMode::MotionMagic, pos);
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

	bool semi_auto_trencher_class::TargetPositionReached(TalonFX* talon1, int pos, std::string name)
	{
		if (isNear(talon1->GetSelectedSensorPosition(), pos, mBuffer))
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
			&& isNear(bucket1.GetSelectedSensorPosition(), buPos, mBuffer) &&  isNear(bucket2.GetSelectedSensorPosition(), buPos, mBuffer)
			&& isNear(bScrew.GetSelectedSensorPosition(), bsPos, mBuffer))
			return true;
		return false;
	}

	void semi_auto_trencher_class::zeroStart(int& p_cmd, ros::NodeHandle  nh)
	{
		std::cout << "Running zeroStart" << std::endl;

		sentinel = p_cmd;

		config(nh);

		linAct1.SetSelectedSensorPosition(0);
		linAct2.SetSelectedSensorPosition(0);
		bucket1.SetSelectedSensorPosition(0);
		bucket2.SetSelectedSensorPosition(0);
		bScrew.SetSelectedSensorPosition(0);
		trencher.SetSelectedSensorPosition(0);

		std::cout << "START" << std::endl;
		displayData();

		stopMotors();
	}

	// Reassigns absolute position so motors know where they are
	void semi_auto_trencher_class::zero(int& p_cmd, ros::NodeHandle  nh) 
	{
		sentinel = p_cmd;
		std::cout << "Running zero" << std::endl;
		zeroStart(p_cmd, nh);
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		
		// ZERO THE LINEAR ACTUATOR
		std::cout << "Zeroing the Linear Actuator" << std::endl;

		ConfigMotionMagic(&linAct1, &linAct2, 10, 5, -800);
		std::this_thread::sleep_for(std::chrono::milliseconds(2000));

		do {
			displayData(&linAct1, &linAct2, "linAct");

			if(exitFunction(p_cmd)) return;

			std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		} while(!ReverseLimitSwitchTriggered(&linAct1, &linAct2, "linAct"));

		std::cout << "Finished zeroing the linAct" << std::endl;
		
		// Move the linAct to dig position so that the bScrew and bucket can be zeroed
		while (!TargetPositionReached(&linAct1, &linAct2, laDigPosition, "linAct")){
			std::cout << "Moving to dig position: " << laDigPosition << std::endl;

			ConfigMotionMagic(&linAct1, &linAct2, 10, 5, laDigPosition);

			displayData(&linAct1, &linAct2, "linAct");

			if(exitFunction(p_cmd)) return;

			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}
		
		std::cout << "LINACT FINAL: " << std::endl;
		displayData(&linAct1, &linAct2, "linAct");
		
		// ZERO THE BUCKET
		std::cout << "Zeroing the bucket" << std::endl;

		ConfigMotionMagic(&bucket1, &bucket2, 10, 5, -800);

		do 
		{
			displayData(&bucket1, &bucket2, "bucket");

			if(exitFunction(p_cmd)) return;

			std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		} while(!ReverseLimitSwitchTriggered(&bucket1, &bucket2, "bucket"));

		// Move the bucket to drive position
		while (!TargetPositionReached(&bucket1, &bucket2, buDrivePosition, "bucket")){
			std::cout << "Moving bucket to position: " << buDrivePosition << std::endl;

			ConfigMotionMagic(&bucket1, &bucket2, 10, 5, buDrivePosition);

			displayData(&bucket1, &bucket2, "bucket");

			if(exitFunction(p_cmd)) return;

			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}

		std::cout << "BUCKET FINAL: " << std::endl;
		displayData(&bucket1, &bucket2, "bucket");

		// ZERO THE BALL SCREW

		std::cout << "Zeroing the bScrew" << std::endl;

		ConfigMotionMagic(&bScrew, 10, 5, -9720000);

		do 
		{	
			displayData(&bScrew, "bScrew");

			if(exitFunction(p_cmd)) return;

			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		
		} while(!ReverseLimitSwitchTriggered(&bScrew, "bScrew"));


		std::cout << "BSCREW FINAL: " << std::endl;
		displayData(&bScrew, "bScrew");

		// Move the Linear Actuator back to drive position
		while(!TargetPositionReached(&linAct1, &linAct2, laDrivePosition, "linAct")){
			
			ConfigMotionMagic(&linAct1, &linAct2, 10, 5, laDrivePosition);

			displayData(&linAct1, &linAct2, "linAct");

			if(exitFunction(p_cmd)) return;

			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}

		std::cout << "LINACT FINAL: " << std::endl;
		displayData(&linAct1, &linAct2, "linAct");

		stopMotors();
	}

	// trencher is all the way tucked in and parallel to ground
	void semi_auto_trencher_class::driveMode(int& p_cmd, ros::NodeHandle  nh) 
		{
			sentinel = p_cmd;
			
			ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
			std::cout << "Moving to driveMode: " << std::endl;
		
			trencher.Set(ControlMode::Velocity, 0);

			if(exitFunction(p_cmd)) return;

			// While not in drive mode,
			while(!CheckMode(laDrivePosition, buDrivePosition, bsDrivePosition))
			{
			
				ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

				trencher.Set(ControlMode::Velocity, 0);

				// If in dig position and bScrew retracted, cycle the trencher once.
				// if (CheckMode(laDigPosition, buDigPosition, bsDrivePosition))
				// {	
				// 	trencher.Set(ControlMode::Position, 10000);
				// 	std::this_thread::sleep_for(std::chrono::milliseconds(5000));
				// }
				
				// bucket moves first
				ConfigMotionMagic(&bucket1, &bucket2, 10, 5, buDrivePosition);

				// bScrew moves
				ConfigMotionMagic(&bScrew, 10, 5, bsDrivePosition);

				// If the ball screw retracted and bucket in drive position, start moving the linAct
				if (TargetPositionReached(&bScrew, bsDrivePosition, "bScrew") && TargetPositionReached(&bucket1, &bucket2, buDrivePosition, "bucket"))
					ConfigMotionMagic(&linAct1, &linAct2, 10, 5, laDrivePosition);

				if(exitFunction(p_cmd)) return;

				displayData();

				std::this_thread::sleep_for(std::chrono::milliseconds(3000));
			}
			stopMotors();
		}

	void semi_auto_trencher_class::deposit(int& p_cmd, ros::NodeHandle  nh)
	{
		sentinel = p_cmd;

		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
			
		std::cout << "Moving to depositMode: " << std::endl;

		displayData(&bucket1, &bucket2, "bucket");

		// If robot is in drive position, go to deposit position.
		if(CheckMode(laDrivePosition, buDrivePosition, bsDrivePosition))
		{
			// while not in deposit position.
			while(!CheckMode(laDepositPosition, buDepositPosition, bsDepositPosition))
			{
				// Move the linAct first
				ConfigMotionMagic(&linAct1, &linAct2, 10, 5, laDepositPosition);

				// Move the bucket when the linAct has reached deposit position
				if (TargetPositionReached(&linAct1, &linAct2, laDepositPosition, "linAct"))
					ConfigMotionMagic(&bucket1, &bucket2, 10, 5, buDepositPosition);

				if(exitFunction(p_cmd)) return;

				std::this_thread::sleep_for(std::chrono::milliseconds(1000));
			}

			// For now, no timer to let gravel fall into sieve, if mech says otherwise, uncomment
			std::this_thread::sleep_for(std::chrono::milliseconds(5000));

			// Move back to drive position
			driveMode(p_cmd, nh);

			if(exitFunction(p_cmd)) return;

			displayData();

			std::this_thread::sleep_for(std::chrono::milliseconds(3000));
		}
		stopMotors();

		// a parameter that lets robot_mux_control.py know to tell move_base_client.py to send robot to dig
		nh.setParam("ongoingDepositPhase", true);
	}

	void semi_auto_trencher_class::dig(int& p_cmd, ros::NodeHandle  nh)
	{
		sentinel = p_cmd;
		
		std::cout << "Moving to digMode: " << std::endl;

		// If robot is in driveMode, go to digMode.
		if(CheckMode(laDrivePosition, buDrivePosition, bsDrivePosition))
			{
				// While not in digMode,
				while(!CheckMode(laDigPosition, buDigPosition, bsDigPosition))
				{
					// Move the linAct first
					ConfigMotionMagic(&linAct1, &linAct2, 10, 5, laDigPosition);
					
					// Move the bucket
					ConfigMotionMagic(&bucket1, &bucket2, 10, 5, buDigPosition);
					
					// Move the ball screw after the linAct has rotated past a certain point
					if (TargetPositionReached(&linAct1, &linAct2, laDigPosition, "linAct"))
					{
						ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);

						ConfigMotionMagic(&bScrew, 10, 5, bsDigPosition);
						// Turn the trencher on
						trencher.Set(ControlMode::Velocity, 10);
					}

					displayData();
					if(exitFunction(p_cmd)) return;
					std::this_thread::sleep_for(std::chrono::milliseconds(1000));
				}

				// Move back to drive position
				driveMode(p_cmd, nh);

				if(exitFunction(p_cmd)) return;

				displayData(); 
				std::this_thread::sleep_for(std::chrono::milliseconds(3000));
			}	
		stopMotors();

		// a parameter that lets robot_mux_control.py know to tell move_base_client.py to send robot to deposit
		nh.setParam("ongoingDigPhase", true);
	}

	double semi_auto_trencher_class::calculateDistanceWheels()
	{
		wheelPositionDifference = abs(desiredWheelPosition - initialWheelPosition);
		return wheelPositionDifference;
	}

	void semi_auto_trencher_class::returnInitialWheelPosition()
	{initialWheelPosition = leftWheel.GetSelectedSensorPosition();}

	void semi_auto_trencher_class::returnDesiredWheelPosition()
	{desiredWheelPosition = leftWheel.GetSelectedSensorPosition();}

	void semi_auto_trencher_class::moveWheelsToSieve()
	{
		double newPos = calculateDistanceWheels();
		ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
		leftWheel.Set(ControlMode::Position, (newPos + leftWheel.GetSelectedSensorPosition()));
		rightWheel.Set(ControlMode::Position, (newPos + leftWheel.GetSelectedSensorPosition()));
	}