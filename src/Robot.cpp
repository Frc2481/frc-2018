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
#include "Commands/DriveTrainDriveToPosition.h"
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

enum Autos {
	POS_LEFT = 1,
	POS_CENTER = 2,
	POS_RIGHT = 4,

	SCALE_LEFT = 8,
	SCALE_RIGHT = 16,

	SWITCH_LEFT = 32,
	SWITCH_RIGHT = 64,

	NOTHING1 = 128,
	SCALE1 = 256,
	SWITCH1 = 512,

	NOTHING2 = 1024,
	SCALE2 = 2048,
	SWITCH2 = 4096,

	NOTHING3 = 8192,
	SCALE3 = 16384,
	SWITCH3 = 32768,

	EXCHANGE1 = 65536,
	EXCHANGE2 = 131072
};

class Robot: public TimedRobot {
public:
	int m_intakePos;

private:
	CameraServer* m_server = CameraServer::GetInstance();
	cs::UsbCamera m_usbCam1;
	cs::UsbCamera m_usbCam2;

	SendableChooser<Autos>* m_posChooser;
	SendableChooser<Autos>* m_firstCubeChooser;
	SendableChooser<Autos>* m_secondCubeChooser;
	SendableChooser<Autos>* m_thirdCubeChooser;

	FieldConfiguration m_fieldConfig;

	std::map<int, Command*> *AutoTasks;

	void RobotInit() {
		SetPeriod(.015); //100hz
		CommandBase::init();
		frc::SmartDashboard::PutData("Auto Modes", &chooser);

		frc::SmartDashboard::PutData("Drive Train Test", new DriveTrainTestCommandGroup());

		SmartDashboard::PutData("Drive to Position", new DriveTrainDriveToPosition());
		SmartDashboard::PutData("Drive Path", new DriveTrainFollowPath("/home/lvuser/robotPath.csv"));

		SmartDashboard::PutData("Shift Up", new DriveTrainShiftCommand());

		CommandBase::m_limeLight->ActivatePowerCubePipeline();

		SmartDashboard::PutData("TestDrivePathGeneratorCommand", new TestDrivePathGeneratorCommand());

		SmartDashboard::PutData("DriveTrainEngagePtoCommand", new DriveTrainEngagePtoCommand());
		SmartDashboard::PutData("DriveTrainOpenLoopCommand", new DriveTrainOpenLoopCommand());

//		frc::SmartDashboard::PutData("Drive Train Test", new DriveTrainTestCommandGroup());

		AutoTasksFunction();

		m_posChooser = new SendableChooser<Autos>();
		m_posChooser->AddDefault("Left", POS_LEFT);
		m_posChooser->AddObject("Center", POS_CENTER);
		m_posChooser->AddObject("Right", POS_RIGHT);

		m_firstCubeChooser = new SendableChooser<Autos>();
		m_firstCubeChooser->AddDefault("Nothing1", NOTHING1); //drive forward
		m_firstCubeChooser->AddObject("Scale1", SCALE1);
		m_firstCubeChooser->AddObject("Switch1", SWITCH1);

		m_secondCubeChooser = new SendableChooser<Autos>();
		m_secondCubeChooser->AddDefault("Nothing2", NOTHING2);
		m_secondCubeChooser->AddObject("Scale2", SCALE2);
		m_secondCubeChooser->AddObject("Switch2", SWITCH2);

		m_thirdCubeChooser = new SendableChooser<Autos>();
		m_thirdCubeChooser->AddDefault("Nothing3", NOTHING3);
		m_thirdCubeChooser->AddObject("Scale3", SCALE3);
		m_thirdCubeChooser->AddObject("Switch3", SWITCH3);

		SmartDashboard::PutData("Start Pos", m_posChooser);
		SmartDashboard::PutData("First Cube", m_firstCubeChooser);
		SmartDashboard::PutData("Second Cube", m_secondCubeChooser);
		SmartDashboard::PutData("Third Cube", m_thirdCubeChooser);

//		CommandBase::m_driveTrain->GetObserver()->ResetPose();

		CommandBase::m_driveTrain->GetObserver()->ResetPose(RigidTransform2D(Translation2D(46.4, 19.5),
																		  Rotation2D::fromDegrees(0)));


//		SmartDashboard::PutData(new AutoRoutineLeftStartLeftScaleLeftCube1SwitchCommandGroup);
//		SmartDashboard::PutData(new AutoRoutineLeftStartLeftScaleLeftCube6SwitchCommandGroup);
//		SmartDashboard::PutData(new AutoRoutineRightStartRightScaleRightCube6SwitchCommandGroup);
//		SmartDashboard::PutData(new AutoRoutineRightStartRightScaleRightCube1SwitchCommandGroup);

//		SmartDashboard::PutData(new AutoRoutineCommandGroup());
		SmartDashboard::PutData("Zero Pose Left Start", new ObserverResetPosCommand(RigidTransform2D(Translation2D(46.4, 19.5), Rotation2D::fromDegrees(0))));
		SmartDashboard::PutData("Zero Pose Right Start", new ObserverResetPosCommand(RigidTransform2D(Translation2D(324 - 46.4, 19.5), Rotation2D::fromDegrees(0))));
		SmartDashboard::PutData(new ArmZeroCommandGroup());

		SmartDashboard::PutData("Zero Pose", new ObserverResetPosCommand(RigidTransform2D(Translation2D(0, 0), Rotation2D::fromDegrees(0))));
//		SmartDashboard::PutData("Zero Pose Left Scale", new ObserverResetPosCommand(RigidTransform2D(Translation2D(83, 20), Rotation2D::fromDegrees(10))));

		SmartDashboard::PutData(new DriveTrainZeroGyroCommand());

		SmartDashboard::PutData(new ArmClearStickyFaults());

		SmartDashboard::PutData(new ReloadNewPathsCommand());

//		SmartDashboard::PutData(new AutoCubeCommandGroup("/home/lvuser/PathLeftScaleToRightCube1.csv"));

		SmartDashboard::PutData("reset observer left to right", new ObserverResetPosCommand(RigidTransform2D(Translation2D(78.6, 278.5), Rotation2D::fromDegrees(-10))));

		SmartDashboard::PutData("reset observer left", new ObserverResetPosCommand(RigidTransform2D(Translation2D(46.4, 19.5), Rotation2D::fromDegrees(0))));


		SmartDashboard::PutData("log observer", new LogObserverCommand());

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
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	void DisabledInit() override {

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

		Autos baseAutoMode = static_cast<Autos>(
								m_posChooser->GetSelected() |
							  ((m_fieldConfig.GetOurSwitchPlate() == FieldConfiguration::LEFT) ? SWITCH_LEFT : SWITCH_RIGHT) |
						      ((m_fieldConfig.GetScalePlate() == FieldConfiguration::LEFT) ? SCALE_LEFT : SCALE_RIGHT));

		Command* startCommand;
		if(m_posChooser->GetSelected() == POS_LEFT) {
			startCommand = new ObserverResetPosCommand(RigidTransform2D(Translation2D(46.4, 19.5), Rotation2D::fromDegrees(0)));
		}
		else if(m_posChooser->GetSelected() == POS_CENTER) {
			startCommand = new ObserverResetPosCommand(RigidTransform2D(Translation2D(0, 19.5), Rotation2D::fromDegrees(0))); //get start for center
		}
		else {
			startCommand = new ObserverResetPosCommand(RigidTransform2D(Translation2D(277.6, 19.5), Rotation2D::fromDegrees(0)));
		}

		Autos autoTask1 = static_cast<Autos>(baseAutoMode | m_firstCubeChooser->GetSelected());
		Command* task1 = AutoTasks->at(autoTask1);

		Autos autoTask2 = static_cast<Autos>(baseAutoMode | m_secondCubeChooser->GetSelected());
		Command* task2 = AutoTasks->at(autoTask2);

		Autos autoTask3 = static_cast<Autos>(baseAutoMode | m_thirdCubeChooser->GetSelected());
		Command* task3 = AutoTasks->at(autoTask3);

		autonomousCommand = std::unique_ptr<Command>(new AutoRoutineCommandGroup(startCommand, task1, task2, task3));

		if(autonomousCommand.get() != nullptr) {
			autonomousCommand->Start();
		}

//		if(m_posChooser->GetSelected() == POS_LEFT) {
//			CommandBase::m_driveTrain->GetObserver()->SetRobotPos(RigidTransform2D(Translation2D(46.4, 19.5),
//																  Rotation2D::fromDegrees(0)), GetFPGATime());
//		}
//		else if(m_posChooser->GetSelected() == POS_CENTER) {
//			CommandBase::m_driveTrain->GetObserver()->SetRobotPos(RigidTransform2D(Translation2D()));
//		}
//		else {
//			CommandBase::m_driveTrain->GetObserver()->SetRobotPos(RigidTransform2D(Translation2D(279.656, 16.5),
//																  Rotation2D::fromDegrees(0), GetFPGATime()));
//		}


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
		AutoTasks = new std::map<int, Command*>();

		(*AutoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_LEFT | SCALE1] = new AutoScaleCommandGroup<ArmToMidScaleFront>("/home/lvuser/PathLeftStartToLeftScale.csv");
		(*AutoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_LEFT | SWITCH1] = new AutoSwitchCommandGroup("/home/lvuser/PathLeftStartToLeftSwitch.csv", true);
		(*AutoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_LEFT | NOTHING1] = new AutoCommand();

		(*AutoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_LEFT | SCALE2] = new AutoCubeAndScaleCommandGroup<ArmToMidScale2Front>("/home/lvuser/PathLeftScaleToLeftCube1.csv", "", -1, -1);
		(*AutoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_LEFT | SWITCH2] = new AutoCubeAndSwitchCommandGroup("/home/lvuser/PathLeftScaleToLeftCube1.csv", "/home/lvuser/PathLeftCube1ToSwitch.csv", -1, -1, -1, -1);
		(*AutoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_LEFT | NOTHING2] = new AutoCommand();

		(*AutoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_LEFT | SCALE3] = new AutoCommand();
		(*AutoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_LEFT | SWITCH3] = new AutoCommand();
		(*AutoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_LEFT | NOTHING3] = new AutoCommand();


		(*AutoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_RIGHT | SCALE1] = new AutoScaleCommandGroup<ArmToMidScaleFront>("/home/lvuser/PathLeftStartToLeftScale.csv");
		(*AutoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_RIGHT | SWITCH1] = new AutoCommand();
		(*AutoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_RIGHT | NOTHING1] = new AutoCommand();

		(*AutoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_RIGHT | SCALE2] = new AutoCubeAndScaleCommandGroup<ArmToMidScale2Front>("/home/lvuser/PathLeftScaleToLeftCube1.csv", "", -1, -1);
		(*AutoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_RIGHT | SWITCH2] = new AutoCubeAndSwitchCommandGroup("/home/lvuser/PathLeftScaleToLeftCube6.csv", "/home/lvuser/PathLeftCube6ToSwitch.csv", 155, -1, -1, -1);
		(*AutoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_RIGHT | NOTHING2] = new AutoCommand();

		(*AutoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_RIGHT | SCALE3] = new AutoCommand();
		(*AutoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_RIGHT | SWITCH3] = new AutoCommand();
		(*AutoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_RIGHT | NOTHING3] = new AutoCommand();


		(*AutoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_LEFT | SCALE1] = new AutoScaleCommandGroup<ArmToMidScaleFront>("/home/lvuser/PathLeftStartToRightScale.csv");
		(*AutoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_LEFT | SWITCH1] = new AutoStartSwitchCommandGroup("/home/lvuser/PathLeftStartToLeftSwitch.csv", -1, 160);
		(*AutoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_LEFT | NOTHING1] = new AutoCommand();

		(*AutoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_LEFT | SCALE2] = new AutoCubeAndScaleCommandGroup<ArmToMidScale2Front>("/home/lvuser/PathLeftSwitchToLeftCube4.csv", "/home/lvuser/PathLeftCube4ToRightScale.csv", 105, -1);
		(*AutoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_LEFT | SWITCH2] = new AutoCubeAndSwitchCommandGroup("/home/lvuser/PathLeftSwitchToLeftCube4.csv", "", -1, -1, -1, -1); //change first -1
		(*AutoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_LEFT | NOTHING2] = new AutoCommand();

		(*AutoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_LEFT | SCALE3] = new AutoCommand();
		(*AutoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_LEFT | SWITCH3] = new AutoCommand();
		(*AutoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_LEFT | NOTHING3] = new AutoCommand();


		(*AutoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_RIGHT | SCALE1] = new AutoScaleCommandGroup<ArmToMidScaleFront>("/home/lvuser/PathLeftStartToRightScale.csv");
		(*AutoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_RIGHT | SWITCH1] = new AutoCommand();
		(*AutoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_RIGHT | NOTHING1] = new AutoCommand();

		(*AutoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_RIGHT | SCALE2] = new AutoCubeAndScaleCommandGroup<ArmToMidScale2Front>("/home/lvuser/PathRightScaleToLeftCube6.csv", "", -1, -1);
		(*AutoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_RIGHT | SWITCH2] = new AutoCubeAndSwitchCommandGroup("/home/lvuser/PathRightScaleToLeftCube6.csv", "/home/lvuser/PathLeftCube6ToSwitch2.csv", -1, -1, -1, -1); //change first -1
		(*AutoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_RIGHT | NOTHING2] = new AutoCommand();

		(*AutoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_RIGHT | SCALE3] = new AutoCommand();
		(*AutoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_RIGHT | SWITCH3] = new AutoCommand();
		(*AutoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_RIGHT | NOTHING3] = new AutoCommand();


		(*AutoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_LEFT | SCALE1] = new AutoScaleCommandGroup<ArmToMidScaleFront>("/home/lvuser/PathRightStartToLeftScale.csv");
		(*AutoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_LEFT | SWITCH1] = new AutoCommand();
		(*AutoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_LEFT | NOTHING1] = new AutoCommand();

		(*AutoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_LEFT | SCALE2] = new AutoCommand();
		(*AutoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_LEFT | SWITCH2] = new AutoCubeAndSwitchCommandGroup("/home/lvuser/PathLeftScaleToRightCube6.csv", "/home/lvuser/PathRightCube6ToSwitch.csv", -1, -1, -1, -1);
		(*AutoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_LEFT | NOTHING2] = new AutoCommand();

		(*AutoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_LEFT | SCALE3] = new AutoCommand();
		(*AutoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_LEFT | SWITCH3] = new AutoCommand();
		(*AutoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_LEFT | NOTHING3] = new AutoCommand();


		(*AutoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_RIGHT | SCALE1] = new AutoScaleCommandGroup<ArmToMidScaleFront>("/home/lvuser/PathRightStartToLeftScale.csv");
		(*AutoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_RIGHT | SWITCH1] = new AutoSwitchCommandGroup("/home/lvuser/PathRightStartToRightSwitch.csv", false);
		(*AutoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_RIGHT | NOTHING1] = new AutoCommand();

		(*AutoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_RIGHT | SCALE2] = new AutoCubeAndScaleCommandGroup<ArmToMidScale2Front>("/home/lvuser/PathRightSwitchToRightCube4.csv", "/home/lvuser/PathRightCube4ToLeftScale.csv", -1, -1);
		(*AutoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_RIGHT | SWITCH2] = new AutoCubeAndSwitchCommandGroup("/home/lvuser/PathRightSwitchToRightCube4.csv", "", -1, -1, -1, -1);
		(*AutoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_RIGHT | NOTHING2] = new AutoCommand();

		(*AutoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_RIGHT | SCALE3] = new AutoCommand();
		(*AutoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_RIGHT | SWITCH3] = new AutoCommand();
		(*AutoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_RIGHT | NOTHING3] = new AutoCommand();


		(*AutoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_LEFT | SCALE1] = new AutoScaleCommandGroup<ArmToMidScaleFront>("/home/lvuser/PathRightStartToRightScale.csv");
		(*AutoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_LEFT | SWITCH1] = new AutoCommand();
		(*AutoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_LEFT | NOTHING1] = new AutoCommand();

		(*AutoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_LEFT | SCALE2] = new AutoCubeAndScaleCommandGroup<ArmToMidScaleFront>("/home/lvuser/PathRightScaleToRightCube6.csv", "", -1, -1);
		(*AutoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_LEFT | SWITCH2] = new AutoCubeAndSwitchCommandGroup("/home/lvuser/PathRightScaleToRightCube1.csv", "/home/lvuser/PathRightCube1ToSwitch.csv", -1, -1, -1, -1); //change first -1
		(*AutoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_LEFT | NOTHING2] = new AutoCommand();

		(*AutoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_LEFT | SCALE3] = new AutoCommand();
		(*AutoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_LEFT | SWITCH3] = new AutoCommand();
		(*AutoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_LEFT | NOTHING3] = new AutoCommand();


		(*AutoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_RIGHT | SCALE1] = new AutoScaleCommandGroup<ArmToMidScaleFront>("/home/lvuser/PathRightStartToRightScale.csv");
		(*AutoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_RIGHT | SWITCH1] = new AutoSwitchCommandGroup("/home/lvuser/PathRightStartToRightSwitch.csv", false);
		(*AutoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_RIGHT | NOTHING1] = new AutoCommand();

		(*AutoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_RIGHT | SCALE2] = new AutoCubeAndScaleCommandGroup<ArmToMidScale2Front>("/home/lvuser/PathRightScaleToRightCube1.csv", "", -1, -1);
		(*AutoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_RIGHT | SWITCH2] = new AutoCubeAndSwitchCommandGroup("/home/lvuser/PathRightScaleToRightCube1.csv", "/home/lvuser/PathRightCube1ToSwitch.csv", -1, -1, -1, -1);
		(*AutoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_RIGHT | NOTHING2] = new AutoCommand();

		(*AutoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_RIGHT | SCALE3] = new AutoCommand();
		(*AutoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_RIGHT | SWITCH3] = new AutoCommand();
		(*AutoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_RIGHT | NOTHING3] = new AutoCommand();


		(*AutoTasks)[POS_CENTER | SCALE_LEFT | SWITCH_LEFT | SWITCH1] = new AutoCommand();
		(*AutoTasks)[POS_CENTER | SCALE_LEFT | SWITCH_RIGHT | SWITCH1] = new AutoCommand();
		(*AutoTasks)[POS_CENTER | SCALE_RIGHT | SWITCH_LEFT | SWITCH1] = new AutoCommand();
		(*AutoTasks)[POS_CENTER | SCALE_RIGHT | SWITCH_RIGHT | SWITCH1] = new AutoCommand();

		(*AutoTasks)[POS_CENTER | SCALE_LEFT | SWITCH_LEFT | EXCHANGE1] = new AutoCommand();
		(*AutoTasks)[POS_CENTER | SCALE_LEFT | SWITCH_RIGHT | EXCHANGE1] = new AutoCommand();
		(*AutoTasks)[POS_CENTER | SCALE_RIGHT | SWITCH_LEFT | EXCHANGE1] = new AutoCommand();
		(*AutoTasks)[POS_CENTER | SCALE_RIGHT | SWITCH_RIGHT | EXCHANGE1] = new AutoCommand();

		(*AutoTasks)[POS_CENTER | SCALE_LEFT | SWITCH_LEFT | EXCHANGE2] = new AutoCommand();
		(*AutoTasks)[POS_CENTER | SCALE_LEFT | SWITCH_RIGHT | EXCHANGE2] = new AutoCommand();
		(*AutoTasks)[POS_CENTER | SCALE_RIGHT | SWITCH_LEFT | EXCHANGE2] = new AutoCommand();
		(*AutoTasks)[POS_CENTER | SCALE_RIGHT | SWITCH_RIGHT | EXCHANGE2] = new AutoCommand();

};

private:
	std::unique_ptr<frc::Command> autonomousCommand;
	frc::SendableChooser<frc::Command*> chooser;
	frc::SendableChooser<frc::Command*> autoPos;
	frc::SendableChooser<frc::Command*> firstCube;
	frc::SendableChooser<frc::Command*> secondCube;
	frc::SendableChooser<frc::Command*> thirdCube;
};

START_ROBOT_CLASS(Robot)
