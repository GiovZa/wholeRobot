// The base class for all motors. Declares motors and assigns values based on PID.yaml.
// You must know how classes work in cpp before proceeding

#include <cmath>
#include <string>

// This file we include contains all other necessary includes too
#include <gen2bot/base_trencher_class.h>

base_trencher_class::base_trencher_class(ros::NodeHandle nh)
	: 
	  // can network ID
	  interface("can0"),

	  // dig/deposit motors
	  linAct1(51, interface),
      linAct2(52),
      bScrew(11),
	  trencher(41),
	  bucket1(31),
	  bucket2(32),
	  linActMM(),
	  bScrewMM(),
	  trencherMM(),
	  bucketMM(),
	  
	  // wheel motors
	  leftWheel(22),
	  rightWheel(21),
	  wheelMM(),

	  // cancellation token 
	  sentinel(),

	  // position values
	  laDrivePosition(0),
	  laDepositPosition(650),
	  laDigPosition(550),
	  bsDrivePosition(0),
	  bsDepositPosition(0),
	  bsDigPosition(9720000),
	  buDrivePosition(30),
	  buDepositPosition(840),
	  buDigPosition(0),
	  trencherZeroPosition(0),
	  initialWheelPosition(0),
	  desiredWheelPosition(500),
	  wheelPositionDifference(500),

	  trencherCycle(736000)
{	
	config(nh);
}

void base_trencher_class::config(ros::NodeHandle nh)
{

	// bScrew parameters
	// Gets parameter /bscrew_cfg/motionCruiseVelocity in ros and assigns its value to bScrewMM.motionCruiseVelocity variable
    nh.getParam("/bscrew_cfg/motionCruiseVelocity", bScrewMM.motionCruiseVelocity);
    nh.getParam("/bscrew_cfg/motionAcceleration", bScrewMM.motionAcceleration);
    nh.getParam("/bscrew_cfg/motionCurveStrength", bScrewMM.motionCurveStrength);

    nh.getParam("/bscrew_cfg/slot0/kP", bScrewMM.slot0.kP);
    nh.getParam("/bscrew_cfg/slot0/kI", bScrewMM.slot0.kI);
    nh.getParam("/bscrew_cfg/slot0/kD", bScrewMM.slot0.kD);

	nh.getParam("/bscrew_cfg/drivePosition", bsDrivePosition);
	nh.getParam("/bscrew_cfg/depositPosition", bsDepositPosition);
	nh.getParam("/bscrew_cfg/digPosition", bsDigPosition);

	// linAct parameters
	nh.getParam("/linact_cfg/motionCruiseVelocity", linActMM.motionCruiseVelocity);
    nh.getParam("/linact_cfg/motionAcceleration", linActMM.motionAcceleration);
    nh.getParam("/linact_cfg/motionCurveStrength", linActMM.motionCurveStrength);

    nh.getParam("/linact_cfg/slot0/kP", linActMM.slot0.kP);
    nh.getParam("/linact_cfg/slot0/kI", linActMM.slot0.kI);
    nh.getParam("/linact_cfg/slot0/kD", linActMM.slot0.kD);

	nh.getParam("/linact_cfg/drivePosition", laDrivePosition);
	nh.getParam("/linact_cfg/depositPosition", laDepositPosition);
	nh.getParam("/linact_cfg/digPosition", laDigPosition);

	// bucket parameters
	nh.getParam("/bucket_cfg/motionCruiseVelocity", bucketMM.motionCruiseVelocity);
    nh.getParam("/bucket_cfg/motionAcceleration", bucketMM.motionAcceleration);
    nh.getParam("/bucket_cfg/motionCurveStrength", bucketMM.motionCurveStrength);

    nh.getParam("/bucket_cfg/slot0/kP", bucketMM.slot0.kP);
    nh.getParam("/bucket_cfg/slot0/kI", bucketMM.slot0.kI);
    nh.getParam("/bucket_cfg/slot0/kD", bucketMM.slot0.kD);

	nh.getParam("/bucket_cfg/drivePosition", buDrivePosition);
	nh.getParam("/bucket_cfg/depositPosition", buDepositPosition);
	nh.getParam("/bucket_cfg/digPosition", buDigPosition);

	// trencher parameters
	nh.getParam("/trencher_cfg/motionCruiseVelocity", trencherMM.motionCruiseVelocity);
    nh.getParam("/trencher_cfg/motionAcceleration", trencherMM.motionAcceleration);
    nh.getParam("/trencher_cfg/motionCurveStrength", trencherMM.motionCurveStrength);

    nh.getParam("/trencher_cfg/slot0/kP", trencherMM.slot0.kP);
    nh.getParam("/trencher_cfg/slot0/kI", trencherMM.slot0.kI);
    nh.getParam("/trencher_cfg/slot0/kD", trencherMM.slot0.kD);

	nh.getParam("/trencher_cfg/drivePosition", trencherZeroPosition);
	nh.getParam("/trencher_cfg/cycle", trencherCycle);

	// wheel parameters
 	nh.getParam("/wheel_cfg/motionCruiseVelocity", wheelMM.motionCruiseVelocity);
    nh.getParam("/wheel_cfg/motionAcceleration", wheelMM.motionAcceleration);
    nh.getParam("/wheel_cfg/motionCurveStrength", wheelMM.motionCurveStrength);

    nh.getParam("/wheel_cfg/slot0/kP", wheelMM.slot0.kP);
    nh.getParam("/wheel_cfg/slot0/kI", wheelMM.slot0.kI);
    nh.getParam("/wheel_cfg/slot0/kD", wheelMM.slot0.kD);
    nh.getParam("/wheel_cfg/slot0/kF", wheelMM.slot0.kF);

	// Sensor that reads back encoder ticks and applies PID, can be set in Phoenix Tuner
	linActMM.primaryPID.selectedFeedbackSensor = (FeedbackDevice)TalonSRXFeedbackDevice::Analog;
	bucketMM.primaryPID.selectedFeedbackSensor = (FeedbackDevice)TalonSRXFeedbackDevice::Analog;
	trencherMM.primaryPID.selectedFeedbackSensor = (FeedbackDevice)TalonFXFeedbackDevice::IntegratedSensor;
	bScrewMM.primaryPID.selectedFeedbackSensor = (FeedbackDevice)TalonFXFeedbackDevice::IntegratedSensor;
	wheelMM.primaryPID.selectedFeedbackSensor = (FeedbackDevice)TalonFXFeedbackDevice::IntegratedSensor;

	// Configure the linear actuator motors
	linAct1.ConfigAllSettings(linActMM);
	linAct2.ConfigAllSettings(linActMM);

	// Configure the bucket motors
	bucket1.ConfigAllSettings(bucketMM);
	bucket2.ConfigAllSettings(bucketMM);

	// Configure the trencher motor
	trencher.ConfigAllSettings(trencherMM);

	// configures the ballscrew motor
	bScrew.ConfigAllSettings(bScrewMM);

	// configures the wheel motors
	leftWheel.ConfigAllSettings(wheelMM);
	rightWheel.ConfigAllSettings(wheelMM);

	// Configure the limit switches
	linAct1.ConfigForwardLimitSwitchSource(LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyOpen);
    linAct2.ConfigReverseLimitSwitchSource(LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyOpen);
	bucket1.ConfigForwardLimitSwitchSource(LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyOpen);
    bucket2.ConfigReverseLimitSwitchSource(LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyOpen);

	// Configure sensor phase of motors and inverts them
	// SetSensorPhase inverts motor tick readings (ex. if linAct1.GetSelectedSensorPosition() = 10, now it = -10)
	// SetInverted inverts direction motors move (ex. if linAct1.Set(ControlMode::PercentOutput, 10);, now it = -10)
    linAct1.SetSensorPhase(true);
    linAct2.SetSensorPhase(true);
    bucket1.SetSensorPhase(true);
    bucket2.SetSensorPhase(true);
    leftWheel.SetSensorPhase(true);
	rightWheel.SetSensorPhase(true);
	bScrew.SetSensorPhase(true);

    linAct1.SetInverted(true);
    linAct2.SetInverted(true);
    bucket1.SetInverted(true);
    bucket2.SetInverted(true);
	//bScrew.SetInverted(true);
	//leftWheel.SetInverted(true);
    //rightWheel.SetInverted(true);
	
	// sets acceleration and velocity for movement, but it is set in ROS parameters already
	/*linAct1.ConfigMotionAcceleration(10);
	linAct2.ConfigMotionAcceleration(10);
	linAct1.ConfigMotionCruiseVelocity(20);
	linAct2.ConfigMotionCruiseVelocity(20);*/
}

// kills motors
void base_trencher_class::stopMotors()
{
	ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100000);
	linAct1.Set(ControlMode::PercentOutput, 0);	
	linAct2.Set(ControlMode::PercentOutput, 0);

	bucket1.Set(ControlMode::PercentOutput, 0);
	bucket2.Set(ControlMode::PercentOutput, 0);	

	trencher.Set(ControlMode::PercentOutput, 0);
	bScrew.Set(ControlMode::PercentOutput, 0);

	leftWheel.Set(ControlMode::PercentOutput, 0);
	rightWheel.Set(ControlMode::PercentOutput, 0);

	std::cout << "Stopping all motors" << std::endl;
}

// a function that checks to see if ProcessManager has changed modes, and if so, kills motors and exit current function
// use case: if(exitFunction(p_cmd)) {stopMotors(); return; }
bool base_trencher_class::exitFunction(int& p_cmd)
{
	if(sentinel != p_cmd)
	{
		stopMotors();
		return true;

		std::cout << "ProcessorManager changed nodes. Exiting current function" << std::endl;
	}

	std::cout << "ProcessorManager is same. Continuing function" << std::endl;

	return false;
}

// Makes sure the trencher doesn't physically break itself
void base_trencher_class::isSafe(int& p_cmd)
{ return; } // might need to abandon

