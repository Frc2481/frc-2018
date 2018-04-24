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
#include "Commands/Autos/Auto3LLScale.h"
#include "Commands/Autos/Auto3LLSwitch.h"
#include "Commands/autos/AutoTestArm.h"

#include "Commands/Autos/AutoLLSimpleScale.h"
#include "Commands/Autos/AutoDriveToCenter.h"

#include "Commands/ClimberReverseScaleGrabber.h"
#include "Commands/ClimberReverseSpringHooks.h"

enum Autos {
	SCALE_LEFT = 1,
	SCALE_RIGHT = 2,

	SWITCH_LEFT = 4,
	SWITCH_RIGHT = 8,

	DO_L_SWITCH = 16,
	NO_OVERRIDE = 32,

	TRUST_PARTNER_L_SCALE = 64,
	DONT_TRUST_PARTNER_L_SCALE = 128,

	TRUST_PARTNER_R_SCALE = 256,
	DONT_TRUST_PARTNER_R_SCALE = 512
};

class Robot: public TimedRobot {
public:
	int m_intakePos;

private:
	CameraServer* m_server = CameraServer::GetInstance();
	cs::UsbCamera m_usbCam1;
	cs::UsbCamera m_usbCam2;

	Command* m_logger;

	SendableChooser<Autos>* m_switchOverride;
	SendableChooser<Autos>* m_trustLScale;
	SendableChooser<Autos>* m_trustRScale;

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

		m_switchOverride = new SendableChooser<Autos>();
		m_switchOverride->AddDefault("Do L Switch", DO_L_SWITCH);
		m_switchOverride->AddObject("Don't override", NO_OVERRIDE);

		m_trustLScale = new SendableChooser<Autos>();
		m_trustLScale->AddDefault("Trust partner left", TRUST_PARTNER_L_SCALE);
		m_trustLScale->AddObject("Don't trust partner left", DONT_TRUST_PARTNER_L_SCALE);

		m_trustRScale = new SendableChooser<Autos>();
		m_trustRScale->AddDefault("Trust partner right", TRUST_PARTNER_R_SCALE);
		m_trustRScale->AddObject("Don't trust partner right", DONT_TRUST_PARTNER_R_SCALE);

//		SmartDashboard::PutData("Start Pos", m_posChooser);
//		SmartDashboard::PutData("First Cube", m_firstCubeChooser);
//		SmartDashboard::PutData("Second Cube", m_secondCubeChooser);
//		SmartDashboard::PutData("Third Cube", m_thirdCubeChooser);

		SmartDashboard::PutData("L switch override", m_switchOverride);
		SmartDashboard::PutData("Trust Partner L Scale", m_trustLScale);
		SmartDashboard::PutData("Trust Partner R Scale", m_trustRScale);

		CommandBase::m_driveTrain->GetObserver()->ResetPose(RigidTransform2D(Translation2D(46.4, 19.5),
																		  Rotation2D::fromDegrees(0)));

//		SmartDashboard::PutData("Auto LLR", new AutoLLR());
//
//		SmartDashboard::PutData("Auto LRR", new AutoLRR());

		SmartDashboard::PutData("Auto test arm", new AutoTestArm());

//		SmartDashboard::PutData("Auto3LLSwitch", new Auto3LLSwitch());
//		SmartDashboard::PutData("Auto3LRSwitch", new Auto3LRSwitch());

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

//		SmartDashboard::PutData("3 switch L", new Auto3LLSwitch());

//		SmartDashboard::PutData("Reverse spring hooks", new ClimberReleaseSpringHooks());
//		SmartDashboard::PutData("Reverse scale grabber", new ClimberReleaseScaleGrabber());
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
								m_switchOverride->GetSelected() |
								m_trustLScale->GetSelected() |
								m_trustRScale->GetSelected() |
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

		CommandBase::m_driveTrain->SetBrake(true);
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

		//new autos taking into acount choice b/t switch & scale
		//start left
		(*m_autoTasks)[SCALE_LEFT | SWITCH_LEFT | DO_L_SWITCH | TRUST_PARTNER_L_SCALE | TRUST_PARTNER_R_SCALE] = std::make_shared<AutoLLSimpleScale>();
		(*m_autoTasks)[SCALE_LEFT | SWITCH_LEFT | DO_L_SWITCH | TRUST_PARTNER_L_SCALE | DONT_TRUST_PARTNER_R_SCALE] = std::make_shared<AutoLLSimpleScale>();

		(*m_autoTasks)[SCALE_LEFT | SWITCH_LEFT | DO_L_SWITCH | DONT_TRUST_PARTNER_L_SCALE | TRUST_PARTNER_R_SCALE] = std::make_shared<AutoLLSimpleScale>();
		(*m_autoTasks)[SCALE_LEFT | SWITCH_LEFT | DO_L_SWITCH | DONT_TRUST_PARTNER_L_SCALE | DONT_TRUST_PARTNER_R_SCALE] = std::make_shared<AutoLLSimpleScale>();

		(*m_autoTasks)[SCALE_LEFT | SWITCH_LEFT | NO_OVERRIDE | TRUST_PARTNER_L_SCALE | TRUST_PARTNER_R_SCALE] = std::make_shared<AutoLLSimpleScale>();
		(*m_autoTasks)[SCALE_LEFT | SWITCH_LEFT | NO_OVERRIDE | TRUST_PARTNER_L_SCALE | DONT_TRUST_PARTNER_R_SCALE] = std::make_shared<AutoLLSimpleScale>();

		(*m_autoTasks)[SCALE_LEFT | SWITCH_LEFT | NO_OVERRIDE | DONT_TRUST_PARTNER_L_SCALE | TRUST_PARTNER_R_SCALE] = std::make_shared<Auto3LLScale>();
		(*m_autoTasks)[SCALE_LEFT | SWITCH_LEFT | NO_OVERRIDE | DONT_TRUST_PARTNER_L_SCALE | DONT_TRUST_PARTNER_R_SCALE] = std::make_shared<Auto3LLScale>();

		(*m_autoTasks)[SCALE_LEFT | SWITCH_RIGHT | DO_L_SWITCH | TRUST_PARTNER_L_SCALE | TRUST_PARTNER_R_SCALE] = std::make_shared<AutoLLSimpleScale>();
		(*m_autoTasks)[SCALE_LEFT | SWITCH_RIGHT | DO_L_SWITCH | TRUST_PARTNER_L_SCALE | DONT_TRUST_PARTNER_R_SCALE] = std::make_shared<AutoLLSimpleScale>();

		(*m_autoTasks)[SCALE_LEFT | SWITCH_RIGHT | DO_L_SWITCH | DONT_TRUST_PARTNER_L_SCALE | TRUST_PARTNER_R_SCALE] = std::make_shared<Auto3LLScale>();
		(*m_autoTasks)[SCALE_LEFT | SWITCH_RIGHT | DO_L_SWITCH | DONT_TRUST_PARTNER_L_SCALE | DONT_TRUST_PARTNER_R_SCALE] = std::make_shared<Auto3LLScale>();

		(*m_autoTasks)[SCALE_LEFT | SWITCH_RIGHT | NO_OVERRIDE | TRUST_PARTNER_L_SCALE | TRUST_PARTNER_R_SCALE] = std::make_shared<AutoLLSimpleScale>();
		(*m_autoTasks)[SCALE_LEFT | SWITCH_RIGHT | NO_OVERRIDE | TRUST_PARTNER_L_SCALE | DONT_TRUST_PARTNER_R_SCALE] = std::make_shared<AutoLLSimpleScale>();

		(*m_autoTasks)[SCALE_LEFT | SWITCH_RIGHT | NO_OVERRIDE | DONT_TRUST_PARTNER_L_SCALE | TRUST_PARTNER_R_SCALE] = std::make_shared<Auto3LLScale>();
		(*m_autoTasks)[SCALE_LEFT | SWITCH_RIGHT | NO_OVERRIDE | DONT_TRUST_PARTNER_L_SCALE | DONT_TRUST_PARTNER_R_SCALE] = std::make_shared<Auto3LLScale>();


		(*m_autoTasks)[SCALE_RIGHT | SWITCH_LEFT | DO_L_SWITCH | TRUST_PARTNER_L_SCALE | TRUST_PARTNER_R_SCALE] = std::make_shared<AutoLLSimpleScale>();
		(*m_autoTasks)[SCALE_RIGHT | SWITCH_LEFT | DO_L_SWITCH | TRUST_PARTNER_L_SCALE | DONT_TRUST_PARTNER_R_SCALE] = std::make_shared<AutoLLSimpleScale>();

		(*m_autoTasks)[SCALE_RIGHT | SWITCH_LEFT | DO_L_SWITCH | DONT_TRUST_PARTNER_L_SCALE | TRUST_PARTNER_R_SCALE] = std::make_shared<AutoLLSimpleScale>();
		(*m_autoTasks)[SCALE_RIGHT | SWITCH_LEFT | DO_L_SWITCH | DONT_TRUST_PARTNER_L_SCALE | DONT_TRUST_PARTNER_R_SCALE] = std::make_shared<AutoLLSimpleScale>();

		(*m_autoTasks)[SCALE_RIGHT | SWITCH_LEFT | NO_OVERRIDE | TRUST_PARTNER_L_SCALE | TRUST_PARTNER_R_SCALE] = std::make_shared<Auto3LLSwitch>();
		(*m_autoTasks)[SCALE_RIGHT | SWITCH_LEFT | NO_OVERRIDE | TRUST_PARTNER_L_SCALE | DONT_TRUST_PARTNER_R_SCALE] = std::make_shared<Auto3LRScale>();

		(*m_autoTasks)[SCALE_RIGHT | SWITCH_LEFT | NO_OVERRIDE | DONT_TRUST_PARTNER_L_SCALE | TRUST_PARTNER_R_SCALE] = std::make_shared<Auto3LLSwitch>();
		(*m_autoTasks)[SCALE_RIGHT | SWITCH_LEFT | NO_OVERRIDE | DONT_TRUST_PARTNER_L_SCALE | DONT_TRUST_PARTNER_R_SCALE] = std::make_shared<Auto3LRScale>();

		(*m_autoTasks)[SCALE_RIGHT | SWITCH_RIGHT | DO_L_SWITCH | TRUST_PARTNER_L_SCALE | TRUST_PARTNER_R_SCALE] = std::make_shared<AutoDriveToCenter>();
		(*m_autoTasks)[SCALE_RIGHT | SWITCH_RIGHT | DO_L_SWITCH | TRUST_PARTNER_L_SCALE | DONT_TRUST_PARTNER_R_SCALE] = std::make_shared<Auto3LRScale>();

		(*m_autoTasks)[SCALE_RIGHT | SWITCH_RIGHT | DO_L_SWITCH | DONT_TRUST_PARTNER_L_SCALE | TRUST_PARTNER_R_SCALE] = std::make_shared<AutoDriveToCenter>();
		(*m_autoTasks)[SCALE_RIGHT | SWITCH_RIGHT | DO_L_SWITCH | DONT_TRUST_PARTNER_L_SCALE | DONT_TRUST_PARTNER_R_SCALE] = std::make_shared<Auto3LRScale>();

		(*m_autoTasks)[SCALE_RIGHT | SWITCH_RIGHT | NO_OVERRIDE | TRUST_PARTNER_L_SCALE | TRUST_PARTNER_R_SCALE] = std::make_shared<AutoDriveToCenter>();
		(*m_autoTasks)[SCALE_RIGHT | SWITCH_RIGHT | NO_OVERRIDE | TRUST_PARTNER_L_SCALE | DONT_TRUST_PARTNER_R_SCALE] = std::make_shared<Auto3LRScale>();

		(*m_autoTasks)[SCALE_RIGHT | SWITCH_RIGHT | NO_OVERRIDE | DONT_TRUST_PARTNER_L_SCALE | TRUST_PARTNER_R_SCALE] = std::make_shared<AutoDriveToCenter>();
		(*m_autoTasks)[SCALE_RIGHT | SWITCH_RIGHT | NO_OVERRIDE | DONT_TRUST_PARTNER_L_SCALE | DONT_TRUST_PARTNER_R_SCALE] = std::make_shared<Auto3LRScale>();


//		(*m_autoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_RIGHT | CAN_CROSS | SCALE_AUTO] = std::make_shared<Auto3LLScale>();
//		(*m_autoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_LEFT | CAN_CROSS | SCALE_AUTO] = std::make_shared<Auto3LRScale>();
//		(*m_autoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_RIGHT | CAN_CROSS | SCALE_AUTO] = std::make_shared<Auto3LRScale>();
//
//		(*m_autoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_LEFT | CAN_CROSS | SWITCH_AUTO] = std::make_shared<Auto3LLSwitch>();
//		(*m_autoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_RIGHT | CAN_CROSS | SWITCH_AUTO] = std::make_shared<Auto3LRSwitch>();
//		(*m_autoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_LEFT | CAN_CROSS | SWITCH_AUTO] = std::make_shared<Auto3LLSwitch>();
//		(*m_autoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_RIGHT | CAN_CROSS | SWITCH_AUTO] = std::make_shared<Auto3LRSwitch>();
//
//		(*m_autoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_LEFT | CANT_CROSS | SCALE_AUTO] = std::make_shared<Auto3LLScale>();
//		(*m_autoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_RIGHT | CANT_CROSS | SCALE_AUTO] = std::make_shared<Auto3LLScale>();
//		(*m_autoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_LEFT | CANT_CROSS | SCALE_AUTO] = std::make_shared<Auto3LLSwitch>(); //do switch instead
//		(*m_autoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_RIGHT | CANT_CROSS | SCALE_AUTO] = std::shared_ptr<Command>(nullptr);  //drive forward
//
//		(*m_autoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_LEFT | CANT_CROSS | SWITCH_AUTO] = std::make_shared<Auto3LLSwitch>();
//		(*m_autoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_RIGHT | CANT_CROSS | SWITCH_AUTO] = std::make_shared<Auto3LRSwitch>();
//		(*m_autoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_LEFT | CANT_CROSS | SWITCH_AUTO] = std::make_shared<Auto3LLSwitch>();
//		(*m_autoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_RIGHT | CANT_CROSS | SWITCH_AUTO] = std::shared_ptr<Command>(nullptr);
//
//		//start right
//		(*m_autoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_LEFT | CAN_CROSS | SCALE_AUTO] = std::shared_ptr<Command>(nullptr);
//		(*m_autoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_RIGHT | CAN_CROSS | SCALE_AUTO] = std::shared_ptr<Command>(nullptr);
//		(*m_autoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_LEFT | CAN_CROSS | SCALE_AUTO] = std::shared_ptr<Command>(nullptr);
//		(*m_autoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_RIGHT | CAN_CROSS | SCALE_AUTO] = std::shared_ptr<Command>(nullptr);
//
//		(*m_autoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_LEFT | CAN_CROSS | SWITCH_AUTO] = std::shared_ptr<Command>(nullptr);
//		(*m_autoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_RIGHT | CAN_CROSS | SWITCH_AUTO] = std::shared_ptr<Command>(nullptr);
//		(*m_autoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_LEFT | CAN_CROSS | SWITCH_AUTO] = std::shared_ptr<Command>(nullptr);
//		(*m_autoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_RIGHT | CAN_CROSS | SWITCH_AUTO] = std::shared_ptr<Command>(nullptr);
//
//		(*m_autoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_LEFT | CANT_CROSS | SCALE_AUTO] = std::shared_ptr<Command>(nullptr);
//		(*m_autoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_RIGHT | CANT_CROSS | SCALE_AUTO] = std::shared_ptr<Command>(nullptr);
//		(*m_autoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_LEFT | CANT_CROSS | SCALE_AUTO] = std::shared_ptr<Command>(nullptr);
//		(*m_autoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_RIGHT | CANT_CROSS | SCALE_AUTO] = std::shared_ptr<Command>(nullptr);
//
//		(*m_autoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_LEFT | CANT_CROSS | SWITCH_AUTO] = std::shared_ptr<Command>(nullptr);
//		(*m_autoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_RIGHT | CANT_CROSS | SWITCH_AUTO] = std::shared_ptr<Command>(nullptr);
//		(*m_autoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_LEFT | CANT_CROSS | SWITCH_AUTO] = std::shared_ptr<Command>(nullptr);
//		(*m_autoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_RIGHT | CANT_CROSS | SWITCH_AUTO] = std::shared_ptr<Command>(nullptr);
};

private:
	std::shared_ptr<frc::Command> autonomousCommand;
	frc::SendableChooser<frc::Command*> autoPos;
	frc::SendableChooser<frc::Command*> centerCross;
	frc::SendableChooser<frc::Command*> objective;
};

//LRR hit last 2 cubes
//LLR arm goes down too soon
//LLL cube to switch, not thrown away

START_ROBOT_CLASS(Robot)
