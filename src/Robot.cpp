#include <Commands/AutoRoutineLeftStartLeftScaleLeftCube1SwitchCommandGroup.h>
#include <Commands/AutoRoutineLeftStartLeftScaleLeftCube6SwitchCommandGroup.h>
#include <Commands/AutoRoutineRightStartRightScaleRightCube1SwitchCommandGroup.h>
#include <Commands/AutoRoutineRightStartRightScaleRightCube6SwitchCommandGroup.h>
#include <Commands/AutoScaleCommandGroup.h>
#include <memory>

#include "WPILib.h"
#include <Commands/Command.h>
#include <Commands/DriveTrainFollowPath.h>
#include <Commands/DriveTrainWaitForFieldXorYCommandGroup.h>
#include <Commands/ReloadNewPathsCommand.h>
#include <Commands/Scheduler.h>
#include <TimedRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <Subsystems/DriveTrain.h>
#include "CommandBase.h"
#include "Commands/Diag/DriveTrainTestCommandGroup.h"
#include "Commands/DriveTrainEngagePtoCommand.h"
#include "Commands/DriveTrainOpenLoopCommand.h"
#include "Commands/DriveTrainShiftCommand.h"
#include "Commands/ArmBaseCommand.h"
#include "Commands/AutoCommand.h"
#include "Components/FieldConfiguration.h"
//#include "Commands/AutoSwitch2CommandGroup.h"
#include "Commands/TestDrivePathGeneratorCommand.h"
#include "Commands/AutoRoutineCommandGroup.h"
#include "Subsystems/Observer.h"
#include "Commands/ArmZeroCommandGroup.h"
#include "Commands/AutoSwitchCommandGroup.h"
#include "Commands/AutoCubeCommandGroup.h"
#include "Commands/DriveTrainZeroGyroCommand.h"
#include "Commands/ArmClearStickyFaults.h"
#include "Commands/TestArmDuringPathCommandGroup.h"
#include "Commands/LogObserverCommand.h"
#include "Commands/AutoCubeAndSwitchCommandGroup.h"
#include "Commands/AutoCubeAndScaleCommandGroup.h"
#include "Commands/AutoStartSwitchCommandGroup.h"

#include "Commands/Autos/AutoLLR.h"
#include "Commands/Autos/AutoLRR.h"
#include "Commands/AutoTest3CubeFromSwitchCommandGroup.h"

#include "Commands/Autos/AutoLRL.h"
#include "Commands/Autos/AutoLL0.h"
#include "Commands/DriveTrainAutoTestDrive.h"

#include "Commands/DriveTrainDriveLogCommand.h"

#include "Commands/Autos/Auto3LRScale.h"
#include "Commands/Autos/Auto3LRSwitch.h"
#include "Commands/Autos/Auto3LLScale.h"
#include "Commands/Autos/Auto3LLSwitch.h"


enum Autos {
	POS_LEFT = 1,
	POS_CENTER = 2,
	POS_RIGHT = 4,

	SCALE_LEFT = 8,
	SCALE_RIGHT = 16,

	SWITCH_LEFT = 32,
	SWITCH_RIGHT = 64,

	CAN_CROSS = 128,
	CANT_CROSS = 256,

	SWITCH_AUTO = 512,
	SCALE_AUTO = 1024
};

class Robot: public TimedRobot {
public:
	int m_intakePos;

private:
	CameraServer* m_server = CameraServer::GetInstance();
	cs::UsbCamera m_usbCam1;
	cs::UsbCamera m_usbCam2;

	Command* m_logger;

	SendableChooser<Autos>* m_posChooser;
	SendableChooser<Autos>* m_crossCenterChooser;
	SendableChooser<Autos>* m_objectiveChooser;

	FieldConfiguration m_fieldConfig;

	std::map<int, std::shared_ptr<Command>> *m_autoTasks;

	void RobotInit() {
		SetPeriod(.015); //100hz
		CommandBase::init();

		frc::SmartDashboard::PutData("Drive Train Test", new DriveTrainTestCommandGroup());

//		CommandBase::m_limeLight->ActivatePowerCubePipeline();

//		SmartDashboard::PutData("DriveTrainEngagePtoCommand", new DriveTrainEngagePtoCommand());
//		SmartDashboard::PutData("DriveTrainOpenLoopCommand", new DriveTrainOpenLoopCommand());

//		frc::SmartDashboard::PutData("Drive Train Test", new DriveTrainTestCommandGroup());

		AutoTasksFunction();

		m_posChooser = new SendableChooser<Autos>();
		m_posChooser->AddDefault("Left", POS_LEFT);
		m_posChooser->AddObject("Center", POS_CENTER);
		m_posChooser->AddObject("Right", POS_RIGHT);

		m_crossCenterChooser = new SendableChooser<Autos>();
		m_crossCenterChooser->AddDefault("Can cross center", CAN_CROSS);
		m_crossCenterChooser->AddObject("Can't cross center", CANT_CROSS);

		m_objectiveChooser = new SendableChooser<Autos>();
		m_objectiveChooser->AddDefault("Scale auto", SCALE_AUTO);
		m_objectiveChooser->AddObject("Switch auto", SWITCH_AUTO);

//		SmartDashboard::PutData("Start Pos", m_posChooser);
//		SmartDashboard::PutData("First Cube", m_firstCubeChooser);
//		SmartDashboard::PutData("Second Cube", m_secondCubeChooser);
//		SmartDashboard::PutData("Third Cube", m_thirdCubeChooser);

		SmartDashboard::PutData("Start Pos", m_posChooser);
		SmartDashboard::PutData("Ability to cross center", m_crossCenterChooser);

		CommandBase::m_driveTrain->GetObserver()->ResetPose(RigidTransform2D(Translation2D(46.4, 19.5),
																		  Rotation2D::fromDegrees(0)));

		SmartDashboard::PutData("Auto LLR", new AutoLLR());

		SmartDashboard::PutData("Auto LRR", new AutoLRR());

		SmartDashboard::PutData("Zero Pose Left Start", new ObserverResetPosCommand(RigidTransform2D(Translation2D(46.44, 19.5), Rotation2D::fromDegrees(0))));
//		SmartDashboard::PutData("Zero Pose Right Start", new ObserverResetPosCommand(RigidTransform2D(Translation2D(324 - 46.4, 19.5), Rotation2D::fromDegrees(0))));
		SmartDashboard::PutData(new ArmZeroCommandGroup());

		SmartDashboard::PutData("Zero Pose", new ObserverResetPosCommand(RigidTransform2D(Translation2D(0, 0), Rotation2D::fromDegrees(0))));
//		SmartDashboard::PutData("Zero Pose Left Scale", new ObserverResetPosCommand(RigidTransform2D(Translation2D(83, 20), Rotation2D::fromDegrees(10))));

		SmartDashboard::PutData(new DriveTrainZeroGyroCommand());

		SmartDashboard::PutData(new ArmClearStickyFaults());

		SmartDashboard::PutData(new ReloadNewPathsCommand());

//		SmartDashboard::PutData("reset observer left to right", new ObserverResetPosCommand(RigidTransform2D(Translation2D(78.6, 278.5), Rotation2D::fromDegrees(-10))));

//		SmartDashboard::PutData("reset observer left", new ObserverResetPosCommand(RigidTransform2D(Translation2D(46.4, 19.5), Rotation2D::fromDegrees(0))));

		SmartDashboard::PutData(new ArmExtensionEncoderZeroCommand());
		SmartDashboard::PutData(new ArmPivotEncoderZeroCommand());

		SmartDashboard::PutData("log observer", new LogObserverCommand());

		SmartDashboard::PutData("drive test auto", new DriveTrainAutoTestDrive());

		SmartDashboard::PutData("drive vel .6", new DriveTrainDriveLogCommand());

		SmartDashboard::PutData(Scheduler::GetInstance());
		SmartDashboard::PutData(CommandBase::m_arm.get());
		SmartDashboard::PutData(CommandBase::m_driveTrain.get());
		SmartDashboard::PutData(CommandBase::m_intake.get());

		m_usbCam1 = m_server->StartAutomaticCapture("cam1", 0);
		m_usbCam1.SetFPS(15);
		m_usbCam1.SetResolution(320, 180);

		m_usbCam2 = m_server->StartAutomaticCapture("cam2", 1);
		m_usbCam2.SetFPS(15);
		m_usbCam2.SetResolution(320, 180);

		m_logger = new LogObserverCommand();

		SmartDashboard::PutData("DriveTrainFollowPath", new DriveTrainFollowPath("/home/lvuser/robotPath.csv"));

		NetworkTable::SetUpdateRate(0.02);

		SmartDashboard::PutData("3 switch L", new Auto3LLSwitch());
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	void DisabledInit() override {

		CommandBase::m_intake->CloseClamp();
		CommandBase::m_intake->RollerOff();

		if (autonomousCommand != nullptr) {
			autonomousCommand->Cancel();
		}
	}

	void DisabledPeriodic() override {
		frc::Scheduler::GetInstance()->Run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString code to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the if-else structure below with additional strings & commands.
	 */
	void AutonomousInit() override {
//		std::string autoSelected = frc::SmartDashboard::GetString("Auto Selector", "Default");

		CommandBase::m_driveTrain->SetBrake(false);

		if (m_logger != nullptr) {
			m_logger->Start();
		}

		m_fieldConfig.Initialize();

		Autos autoMode = static_cast<Autos>(
								m_posChooser->GetSelected() |
								m_crossCenterChooser->GetSelected() |
								m_objectiveChooser->GetSelected() |
 							  ((m_fieldConfig.GetOurSwitchPlate() == FieldConfiguration::LEFT) ? SWITCH_LEFT : SWITCH_RIGHT) |
						      ((m_fieldConfig.GetScalePlate() == FieldConfiguration::LEFT) ? SCALE_LEFT : SCALE_RIGHT));

//		if(auto
				autonomousCommand = m_autoTasks->at(autoMode);

		if(autonomousCommand.get() != nullptr) {
			autonomousCommand->Start();
		}
		SmartDashboard::PutBoolean("ShouldResetPlot", true);
	}

	void AutonomousPeriodic() override {
		frc::Scheduler::GetInstance()->Run();
	}

	void TeleopInit() override {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != nullptr) {
			autonomousCommand->Cancel();
		}
	}

	void TeleopPeriodic() override {
		double startTime = RobotController::GetFPGATime();
		frc::Scheduler::GetInstance()->Run();
		double duration = RobotController::GetFPGATime() - startTime;
		SmartDashboard::PutNumber("cycle per sec", 1 / (duration / 1000000));
//		SmartDashboard::PutNumber("period", );

	}

	void TestPeriodic() override {
		frc::LiveWindow::GetInstance()->Run();
	}

	void AutoTasksFunction(){
		m_autoTasks = new std::map<int, std::shared_ptr<Command>>();

//		(*m_autoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_LEFT | CAN_CROSS] = std::make_shared<Auto3LLScale>();
//		(*m_autoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_RIGHT | CAN_CROSS] = std::make_shared<AutoLRL>();
//		(*m_autoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_LEFT | CAN_CROSS] = std::make_shared<AutoLLR>();
//		(*m_autoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_RIGHT | CAN_CROSS] = std::make_shared<AutoLRR>();
//
//		(*m_autoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_LEFT | CANT_CROSS] = std::make_shared<Auto3LLScale>();
//		(*m_autoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_RIGHT | CANT_CROSS] = std::make_shared<AutoLL0>();
//		(*m_autoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_LEFT | CANT_CROSS] = std::shared_ptr<Command>(nullptr);
//		(*m_autoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_RIGHT | CANT_CROSS] = std::shared_ptr<Command>(nullptr);
//
//		(*m_autoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_LEFT | CAN_CROSS] = std::shared_ptr<Command>(nullptr);
//		(*m_autoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_RIGHT | CAN_CROSS] = std::shared_ptr<Command>(nullptr);
//		(*m_autoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_LEFT | CAN_CROSS] = std::shared_ptr<Command>(nullptr);
//		(*m_autoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_RIGHT | CAN_CROSS] = std::shared_ptr<Command>(nullptr);
//
//		(*m_autoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_LEFT | CANT_CROSS] = std::shared_ptr<Command>(nullptr);
//		(*m_autoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_RIGHT | CANT_CROSS] = std::shared_ptr<Command>(nullptr);
//		(*m_autoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_LEFT | CANT_CROSS] = std::shared_ptr<Command>(nullptr);
//		(*m_autoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_RIGHT | CANT_CROSS] = std::shared_ptr<Command>(nullptr);

		//new autos taking into acount choice b/t switch & scale
		//start left
		(*m_autoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_LEFT | CAN_CROSS | SCALE_AUTO] = std::make_shared<Auto3LLScale>();
		(*m_autoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_RIGHT | CAN_CROSS | SCALE_AUTO] = std::make_shared<Auto3LLScale>();
		(*m_autoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_LEFT | CAN_CROSS | SCALE_AUTO] = std::make_shared<Auto3LRScale>();
		(*m_autoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_RIGHT | CAN_CROSS | SCALE_AUTO] = std::make_shared<Auto3LRScale>();

		(*m_autoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_LEFT | CAN_CROSS | SWITCH_AUTO] = std::make_shared<Auto3LLSwitch>();
		(*m_autoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_RIGHT | CAN_CROSS | SWITCH_AUTO] = std::make_shared<Auto3LRSwitch>();
		(*m_autoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_LEFT | CAN_CROSS | SWITCH_AUTO] = std::make_shared<Auto3LLSwitch>();
		(*m_autoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_RIGHT | CAN_CROSS | SWITCH_AUTO] = std::make_shared<Auto3LRSwitch>();

		(*m_autoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_LEFT | CANT_CROSS | SCALE_AUTO] = std::make_shared<Auto3LLScale>();
		(*m_autoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_RIGHT | CANT_CROSS | SCALE_AUTO] = std::make_shared<Auto3LLScale>();
		(*m_autoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_LEFT | CANT_CROSS | SCALE_AUTO] = std::make_shared<Auto3LLSwitch>(); //do switch instead
		(*m_autoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_RIGHT | CANT_CROSS | SCALE_AUTO] = std::shared_ptr<Command>(nullptr);  //drive forward

		(*m_autoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_LEFT | CANT_CROSS | SWITCH_AUTO] = std::make_shared<Auto3LLSwitch>();
		(*m_autoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_RIGHT | CANT_CROSS | SWITCH_AUTO] = std::make_shared<Auto3LRSwitch>();
		(*m_autoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_LEFT | CANT_CROSS | SWITCH_AUTO] = std::make_shared<Auto3LLSwitch>();
		(*m_autoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_RIGHT | CANT_CROSS | SWITCH_AUTO] = std::shared_ptr<Command>(nullptr);

		//start right
		(*m_autoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_LEFT | CAN_CROSS | SCALE_AUTO] = std::shared_ptr<Command>(nullptr);
		(*m_autoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_RIGHT | CAN_CROSS | SCALE_AUTO] = std::shared_ptr<Command>(nullptr);
		(*m_autoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_LEFT | CAN_CROSS | SCALE_AUTO] = std::shared_ptr<Command>(nullptr);
		(*m_autoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_RIGHT | CAN_CROSS | SCALE_AUTO] = std::shared_ptr<Command>(nullptr);

		(*m_autoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_LEFT | CAN_CROSS | SWITCH_AUTO] = std::shared_ptr<Command>(nullptr);
		(*m_autoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_RIGHT | CAN_CROSS | SWITCH_AUTO] = std::shared_ptr<Command>(nullptr);
		(*m_autoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_LEFT | CAN_CROSS | SWITCH_AUTO] = std::shared_ptr<Command>(nullptr);
		(*m_autoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_RIGHT | CAN_CROSS | SWITCH_AUTO] = std::shared_ptr<Command>(nullptr);

		(*m_autoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_LEFT | CANT_CROSS | SCALE_AUTO] = std::shared_ptr<Command>(nullptr);
		(*m_autoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_RIGHT | CANT_CROSS | SCALE_AUTO] = std::shared_ptr<Command>(nullptr);
		(*m_autoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_LEFT | CANT_CROSS | SCALE_AUTO] = std::shared_ptr<Command>(nullptr);
		(*m_autoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_RIGHT | CANT_CROSS | SCALE_AUTO] = std::shared_ptr<Command>(nullptr);

		(*m_autoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_LEFT | CANT_CROSS | SWITCH_AUTO] = std::shared_ptr<Command>(nullptr);
		(*m_autoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_RIGHT | CANT_CROSS | SWITCH_AUTO] = std::shared_ptr<Command>(nullptr);
		(*m_autoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_LEFT | CANT_CROSS | SWITCH_AUTO] = std::shared_ptr<Command>(nullptr);
		(*m_autoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_RIGHT | CANT_CROSS | SWITCH_AUTO] = std::shared_ptr<Command>(nullptr);
};

private:
	std::shared_ptr<frc::Command> autonomousCommand;
	frc::SendableChooser<frc::Command*> autoPos;
	frc::SendableChooser<frc::Command*> centerCross;
};

//LRR hit last 2 cubes
//LLR arm goes down too soon
//LLL cube to switch, not thrown away

START_ROBOT_CLASS(Robot)
