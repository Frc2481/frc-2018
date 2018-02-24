#include <Commands/AutoScaleCommandGroup.h>
#include <memory>

#include "WPILib.h"
#include <Commands/Command.h>
#include <Commands/DriveTrainFollowPath.h>
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
#include "Commands/AutoRoutineLeftStartLeftScaleLeftCube1LeftSwitchCommandGroup.h"
#include "Commands/AutoRoutineLeftStartLeftScaleRightCube1RightSwitchCommandGroup.h"
#include "Commands/AutoRoutineRightStartRightScaleLeftCube1LeftSwitchCommandGroup.h"
#include "Commands/AutoRoutineRightStartRightScaleRightCube1RightSwitchCommandGroup.h"
#include "Commands/DriveTrainZeroGyroCommand.h"
#include "Commands/ArmClearStickyFaults.h"

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

class Robot: public IterativeRobot {
public:
	int m_intakePos;

private:
	SendableChooser<Autos>* m_posChooser;
	SendableChooser<Autos>* m_firstCubeChooser;
	SendableChooser<Autos>* m_secondCubeChooser;
	SendableChooser<Autos>* m_thirdCubeChooser;

	FieldConfiguration m_fieldConfig;

	std::map<int, Command*> *AutoTasks;


	void RobotInit() {
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

		m_posChooser = new SendableChooser<Autos>();
		m_posChooser->AddObject("Left", POS_LEFT);
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

		SmartDashboard::PutData("PathLeftStartToLeftScale", new AutoScaleCommandGroup("/home/lvuser/PathLeftStartToLeftScale.csv", "/home/lvuser/PathLeftScaleBackUp.csv"));
		SmartDashboard::PutData("PathRightStartToRightScale", new AutoScaleCommandGroup("/home/lvuser/PathRightStartToRightScale.csv", "/home/lvuser/PathRightScaleBackUp.csv"));

		SmartDashboard::PutData("PathLeftStartToRightScale", new AutoScaleCommandGroup("/home/lvuser/PathLeftStartToRightScale.csv", "/home/lvuser/PathRightScaleBackUp.csv"));
		SmartDashboard::PutData("PathRightStartToLeftScale", new AutoScaleCommandGroup("/home/lvuser/PathRightStartToLeftScale.csv", "/home/lvuser/PathLeftScaleBackUp.csv"));

		SmartDashboard::PutData("PathLeftStartToLeftSwitch", new AutoSwitchCommandGroup("/home/lvuser/PathLeftStartToLeftSwitch.csv", true));
		SmartDashboard::PutData("PathRightStartToRightSwitch", new AutoSwitchCommandGroup("/home/lvuser/PathRightStartToRightSwitch.csv", true));

		SmartDashboard::PutData("PathLeftScaleToLeftCube1", new AutoCubeCommandGroup("home/lvuser/PathLeftScaleToLeftCube1.csv"));
		SmartDashboard::PutData("PathLeftScaleToRightCube1", new AutoCubeCommandGroup("home/lvuser/PathLeftScaleToRightCube1.csv"));


		SmartDashboard::PutData("PathLeftCube1ToLeftSwitch", new AutoSwitchCommandGroup("/home/lvuser/PathLeftCube1ToLeftSwitch.csv", false));


		SmartDashboard::PutData(new AutoRoutineLeftStartLeftScaleLeftCube1LeftSwitchCommandGroup);
		SmartDashboard::PutData(new AutoRoutineLeftStartLeftScaleRightCube1RightSwitchCommandGroup);
		SmartDashboard::PutData(new AutoRoutineRightStartRightScaleLeftCube1LeftSwitchCommandGroup);
		SmartDashboard::PutData(new AutoRoutineRightStartRightScaleRightCube1RightSwitchCommandGroup);

//		SmartDashboard::PutData(new AutoRoutineCommandGroup());
		SmartDashboard::PutData("Zero Pose Left Start", new ObserverResetPosCommand(RigidTransform2D(Translation2D(46.4, 19.5), Rotation2D::fromDegrees(0))));
		SmartDashboard::PutData("Zero Pose Right Start", new ObserverResetPosCommand(RigidTransform2D(Translation2D(324 - 46.4, 19.5), Rotation2D::fromDegrees(0))));
		SmartDashboard::PutData(new ArmZeroCommandGroup());

		SmartDashboard::PutData("Zero Pose", new ObserverResetPosCommand(RigidTransform2D(Translation2D(0, 0), Rotation2D::fromDegrees(0))));
//		SmartDashboard::PutData("Zero Pose Left Scale", new ObserverResetPosCommand(RigidTransform2D(Translation2D(83, 20), Rotation2D::fromDegrees(10))));

		SmartDashboard::PutData(new DriveTrainZeroGyroCommand());

		SmartDashboard::PutData(new ArmClearStickyFaults());
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
		/* std::string autoSelected = frc::SmartDashboard::GetString("Auto Selector", "Default");
		if (autoSelected == "My Auto") {
			autonomousCommand.reset(new MyAutoCommand());
		}
		else {
			autonomousCommand.reset(new ExampleCommand());
		} */

		Autos baseAutoMode = m_posChooser->GetSelected() |
						 m_fieldConfig.GetOurSwitchPlate() == FieldConfiguration::LEFT ? SWITCH_LEFT : SWITCH_RIGHT |
						 m_fieldConfig.GetScalePlate() == FieldConfiguration::LEFT ? SCALE_LEFT : SCALE_RIGHT;

		Autos autoTask1 = static_cast<Autos>(baseAutoMode | m_firstCubeChooser->GetSelected());
		Command* task1 = AutoTasks->at(autoTask1);

		Autos autoTask2 = static_cast<Autos>(baseAutoMode | m_secondCubeChooser->GetSelected());
		Command* task2 = AutoTasks->at(autoTask2);

		Autos autoTask3 = static_cast<Autos>(baseAutoMode | m_thirdCubeChooser->GetSelected());
		Command* task3 = AutoTasks->at(autoTask3);


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
		frc::Scheduler::GetInstance()->Run();
	}

	void TestPeriodic() override {
		frc::LiveWindow::GetInstance()->Run();
	}

	void AutoTasksFunction(){
		AutoTasks = new std::map<int, Command*>();

		(*AutoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_LEFT | SCALE1] = new AutoScaleCommandGroup("/home/lvuser/PathLeftStartToLeftScale.csv", "/home/lvuser/PathLeftScaleBackUp.csv");

		(*AutoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_LEFT | SWITCH1] = new AutoSwitchCommandGroup("/home/lvuser/PathLeftCube1ToLeftSwitch.csv", false);
		(*AutoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_LEFT | NOTHING1] = new AutoCommand();

		(*AutoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_LEFT | SCALE2] = new AutoCommand();
		(*AutoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_LEFT | SWITCH2] = new AutoCommand();
		(*AutoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_LEFT | NOTHING2] = new AutoCommand();

		(*AutoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_LEFT | SCALE3] = new AutoCommand();
		(*AutoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_LEFT | SWITCH3] = new AutoCommand();
		(*AutoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_LEFT | NOTHING3] = new AutoCommand();


		(*AutoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_RIGHT | SCALE1] = new AutoCommand();
		(*AutoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_RIGHT | SWITCH1] = new AutoCommand();
		(*AutoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_RIGHT | NOTHING1] = new AutoCommand();

		(*AutoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_RIGHT | SCALE2] = new AutoCommand();
		(*AutoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_RIGHT | SWITCH2] = new AutoCommand();
		(*AutoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_RIGHT | NOTHING2] = new AutoCommand();

		(*AutoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_RIGHT | SCALE3] = new AutoCommand();
		(*AutoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_RIGHT | SWITCH3] = new AutoCommand();
		(*AutoTasks)[POS_LEFT | SCALE_LEFT | SWITCH_RIGHT | NOTHING3] = new AutoCommand();


		(*AutoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_LEFT | SCALE1] = new AutoCommand();
		(*AutoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_LEFT | SWITCH1] = new AutoCommand();
		(*AutoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_LEFT | NOTHING1] = new AutoCommand();

		(*AutoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_LEFT | SCALE2] = new AutoCommand();
		(*AutoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_LEFT | SWITCH2] = new AutoCommand();
		(*AutoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_LEFT | NOTHING2] = new AutoCommand();

		(*AutoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_LEFT | SCALE3] = new AutoCommand();
		(*AutoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_LEFT | SWITCH3] = new AutoCommand();
		(*AutoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_LEFT | NOTHING3] = new AutoCommand();


		(*AutoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_RIGHT | SCALE1] = new AutoCommand();
		(*AutoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_RIGHT | SWITCH1] = new AutoCommand();
		(*AutoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_RIGHT | NOTHING1] = new AutoCommand();

		(*AutoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_RIGHT | SCALE2] = new AutoCommand();
		(*AutoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_RIGHT | SWITCH2] = new AutoCommand();
		(*AutoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_RIGHT | NOTHING2] = new AutoCommand();

		(*AutoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_RIGHT | SCALE3] = new AutoCommand();
		(*AutoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_RIGHT | SWITCH3] = new AutoCommand();
		(*AutoTasks)[POS_LEFT | SCALE_RIGHT | SWITCH_RIGHT | NOTHING3] = new AutoCommand();


		(*AutoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_LEFT | SCALE1] = new AutoCommand();
		(*AutoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_LEFT | SWITCH1] = new AutoCommand();
		(*AutoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_LEFT | NOTHING1] = new AutoCommand();

		(*AutoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_LEFT | SCALE2] = new AutoCommand();
		(*AutoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_LEFT | SWITCH2] = new AutoCommand();
		(*AutoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_LEFT | NOTHING2] = new AutoCommand();

		(*AutoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_LEFT | SCALE3] = new AutoCommand();
		(*AutoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_LEFT | SWITCH3] = new AutoCommand();
		(*AutoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_LEFT | NOTHING3] = new AutoCommand();


		(*AutoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_RIGHT | SCALE1] = new AutoCommand();
		(*AutoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_RIGHT | SWITCH1] = new AutoCommand();
		(*AutoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_RIGHT | NOTHING1] = new AutoCommand();

		(*AutoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_RIGHT | SCALE2] = new AutoCommand();
		(*AutoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_RIGHT | SWITCH2] = new AutoCommand();
		(*AutoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_RIGHT | NOTHING2] = new AutoCommand();

		(*AutoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_RIGHT | SCALE3] = new AutoCommand();
		(*AutoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_RIGHT | SWITCH3] = new AutoCommand();
		(*AutoTasks)[POS_RIGHT | SCALE_LEFT | SWITCH_RIGHT | NOTHING3] = new AutoCommand();


		(*AutoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_LEFT | SCALE1] = new AutoCommand();
		(*AutoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_LEFT | SWITCH1] = new AutoCommand();
		(*AutoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_LEFT | NOTHING1] = new AutoCommand();

		(*AutoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_LEFT | SCALE2] = new AutoCommand();
		(*AutoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_LEFT | SWITCH2] = new AutoCommand();
		(*AutoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_LEFT | NOTHING2] = new AutoCommand();

		(*AutoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_LEFT | SCALE3] = new AutoCommand();
		(*AutoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_LEFT | SWITCH3] = new AutoCommand();
		(*AutoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_LEFT | NOTHING3] = new AutoCommand();


		(*AutoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_RIGHT | SCALE1] = new AutoCommand();
		(*AutoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_RIGHT | SWITCH1] = new AutoCommand();
		(*AutoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_RIGHT | NOTHING1] = new AutoCommand();

		(*AutoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_RIGHT | SCALE2] = new AutoCommand();
		(*AutoTasks)[POS_RIGHT | SCALE_RIGHT | SWITCH_RIGHT | SWITCH2] = new AutoCommand();
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
