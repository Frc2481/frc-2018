/*
 * Arm.cpp
 *
 *  Created on: Jan 25, 2018
 *      Author: FIRSTMentor
 */

#include <Subsystems/Arm.h>
#include "Commands/ArmExtensionEncoderZeroCommand.h"
#include "Commands/ArmToRetractedThresholdCommand.h"
#include "Commands/ArmToExtendedThresholdCommand.h"
#include "Commands/ArmPivotEncoderZeroCommand.h"
#include "Commands/ArmToPivotSetpointCommand.h"
#include "Commands/ArmBaseCommand.h"

Arm::Arm() : Subsystem("Arm"){
	m_extenderMaster = new TalonSRX(EXTENDER_MASTER);
	m_extenderSlave = new TalonSRX(EXTENDER_SLAVE);

	m_extenderMaster->SelectProfileSlot(0, 0);
	m_extenderMaster->Set(ControlMode::PercentOutput, 0);
	m_extenderMaster->Config_kP(0, RobotParameters::k_extenderUpP, 0);
	m_extenderMaster->Config_kI(0, RobotParameters::k_extenderI, 0);
	m_extenderMaster->Config_kD(0, RobotParameters::k_extenderD, 0);
	m_extenderMaster->Config_kF(0, RobotParameters::k_extenderF, 0);

	m_extenderMaster->Config_kP(1, RobotParameters::k_extenderDownP, 0);
	m_extenderMaster->Config_kI(1, RobotParameters::k_extenderI, 0);
	m_extenderMaster->Config_kD(1, RobotParameters::k_extenderD, 0);
	m_extenderMaster->Config_kF(1, RobotParameters::k_extenderF, 0);
	m_extenderMaster->Config_IntegralZone(0, 200, 0);
	m_extenderMaster->SetSensorPhase(false);
	m_extenderMaster->SetInverted(true);

	m_extenderMaster->ConfigNominalOutputForward(0.0, 0.0);
	m_extenderMaster->ConfigNominalOutputReverse(0.0, 0.0);
	m_extenderMaster->ConfigPeakOutputForward(RobotParameters::k_extenderPeakOutputForward, 0.0);
	m_extenderMaster->ConfigPeakOutputReverse(RobotParameters::k_extenderPeakOutputReverse, 0.0);

	m_extenderMaster->ConfigForwardSoftLimitThreshold(22300, 0); //change threshold-> in native units //23237 //5000 off extreme
	m_extenderMaster->ConfigReverseSoftLimitThreshold(0, 0); //change threshold-> in native units //5000 off extreme
	m_extenderMaster->ConfigForwardSoftLimitEnable(true, 0);
	m_extenderMaster->ConfigReverseSoftLimitEnable(true, 0);

	m_extenderMaster->ConfigSelectedFeedbackSensor(CTRE_MagEncoder_Relative, 0, 0);

	m_extenderMaster->ConfigMotionCruiseVelocity(RobotParameters::k_extenderVelocity, 0); //convert to talon speed: encoder count/100 ms / 2.0
	m_extenderMaster->ConfigMotionAcceleration(RobotParameters::k_extenderAcceleration, 0); //9200

	m_extenderMaster->SetStatusFramePeriod(Status_2_Feedback0_, 100, 0);
	m_extenderMaster->SetStatusFramePeriod(Status_10_MotionMagic, 100, 0);

	m_extenderMaster->ConfigAllowableClosedloopError(0, 0, 0);

	m_extenderSlave->SetInverted(true);
	m_extenderSlave->Set(ControlMode::Follower, 15);

	m_desiredExtensionSetpoint = 0;

	m_pivot = new TalonSRX(PIVOT);
	m_pivot->SelectProfileSlot(0, 0);
	m_pivot->Set(ControlMode::PercentOutput, 0);
	m_pivot->Config_kP(0, RobotParameters::k_pivotP, 0);
	m_pivot->Config_kI(0, RobotParameters::k_pivotI, 0);
	m_pivot->Config_kD(0, RobotParameters::k_pivotD, 0);
	m_pivot->Config_kF(0, RobotParameters::k_pivotF, 0);
	m_pivot->Config_IntegralZone(0, 200, 0);
	m_pivot->SetSensorPhase(true);
	m_pivot->SetInverted(true);

	m_pivot->ConfigNominalOutputForward(0.0, 0.0);
	m_pivot->ConfigNominalOutputReverse(0.0, 0.0);
	m_pivot->ConfigPeakOutputForward(RobotParameters::k_pivotPeakOutputForward, 0.0);
	m_pivot->ConfigPeakOutputReverse(RobotParameters::k_pivotPeakOutputReverse, 0.0);

	m_pivot->ConfigForwardSoftLimitEnable(true, 0);
	m_pivot->ConfigForwardSoftLimitThreshold(5740, 0);
	m_pivot->ConfigReverseSoftLimitEnable(true, 0);
	m_pivot->ConfigReverseSoftLimitThreshold(-5530, 0);

	m_pivot->ConfigSelectedFeedbackSensor(CTRE_MagEncoder_Relative, 0, 0);

	m_pivot->ConfigMotionCruiseVelocity(RobotParameters::k_pivotVelocity, 0);
	m_pivot->ConfigMotionAcceleration(RobotParameters::k_pivotAcceleration, 0);

	m_pivot->SetStatusFramePeriod(Status_2_Feedback0_, 100, 0);
	m_pivot->SetStatusFramePeriod(Status_10_MotionMagic, 100, 0);

	m_pivot->ConfigAllowableClosedloopError(0, 0, 0);


	m_prevExtensionTravellingDown = false;

	SmartDashboard::PutData(new ArmExtensionEncoderZeroCommand());
//	SmartDashboard::PutData(new ArmToExtendedThresholdCommand());
//	SmartDashboard::PutData(new ArmToRetractedThresholdCommand());
	SmartDashboard::PutData(new ArmPivotEncoderZeroCommand());
//	SmartDashboard::PutData(new ArmPivotToCenterCommand());
//	SmartDashboard::PutData(new ArmPivotTo90Command());
//	SmartDashboard::PutData(new ArmPivotToNeg90Command());
//
//
//	SmartDashboard::PutData(new ArmTo90Front("ArmTo90Front"));
//	SmartDashboard::PutData(new ArmTo90Back("ArmTo90Back"));
//	SmartDashboard::PutData(new ArmToIntakeFront("ArmToIntakeFront"));
//	SmartDashboard::PutData(new ArmToIntakeBack("ArmToIntakeBack"));
//	SmartDashboard::PutData(new ArmToMidScaleFront("ArmToMidScaleFront"));
//	SmartDashboard::PutData(new ArmToMidScaleBack("ArmToMidScaleBack"));
//
//	SmartDashboard::PutData(new ArmToLowScaleFront("ArmToLowScaleFront"));
//	SmartDashboard::PutData(new ArmToStow("ArmToStow"));
//
//	SmartDashboard::PutData(new ArmToHighScaleFront("ArmToHighScaleFront"));
//	SmartDashboard::PutData(new ArmToHighScaleBack("ArmToHighScaleBack"));


}

Arm::~Arm() {
	// TODO Auto-generated destructor stub
}

void Arm::SetExtensionPostion(double position) {
	position = ConvertInchesToEncTicks(position);
	m_extenderMaster->Set(ControlMode::MotionMagic, position);

	SmartDashboard::PutNumber("set position extension", position);

//	bool goingDown = GetExtensionPosition() > position; //handle pivot
//	if(goingDown != m_prevExtensionTravellingDown) {
//		if(goingDown) {
//			m_extender->SelectProfileSlot(1, 0);
//			m_extender->ConfigMotionAcceleration(1650, 0);
//		}
//		else {
//			m_extender->SelectProfileSlot(0, 0);
			m_extenderMaster->ConfigMotionAcceleration(13800, 0);
//		}
//		m_prevExtensionTravellingDown = goingDown;
//		SmartDashboard::PutNumber("isGoingDown", goingDown);

//	}
}

void Arm::SetDesiredExtension(double extension) {
	m_desiredExtensionSetpoint = extension;
}

double Arm::GetExtensionPosition() {
	return ConvertEncTicksToInches(m_extenderMaster->GetSelectedSensorPosition(0));
}

void Arm::ZeroExtension() {
	m_extenderMaster->SetSelectedSensorPosition(0, 0, 10);
}

void Arm::SetExtensionOpenLoop(double speed) {
	m_extenderMaster->Set(ControlMode::PercentOutput, speed);
}

bool Arm::IsExtensionOnTarget() {
	return fabs(m_extenderMaster->GetClosedLoopError(0)) < 300;
}

void Arm::SetPivotOpenLoop(double speed) {
	m_pivot->Set(ControlMode::PercentOutput, speed);
}

bool Arm::IsPivotOnTarget() {
	return fabs(m_pivot->GetClosedLoopError(0)) < 60;
}

void Arm::SetPivotAccel(int accel) {
	m_pivot->ConfigMotionAcceleration(accel, 0);
}

void Arm::Periodic() {
	//use a slower rate of deacceleration to counteract gravity when coming down

	SetExtensionPostion(GetAllowedExtensionPos());


//	SmartDashboard::PutNumber("extension speed", m_extenderMaster->GetSelectedSensorVelocity(0));
//	SmartDashboard::PutNumber("extension distance", ConvertEncTicksToInches(m_extenderMaster->GetSelectedSensorPosition(0)));
//	SmartDashboard::PutNumber("extension current", m_extenderMaster->GetOutputCurrent());
//	SmartDashboard::PutNumber("extension error", m_extenderMaster->GetClosedLoopError(0));
//	SmartDashboard::PutNumber("active trajectory position extender", m_extenderMaster->GetActiveTrajectoryPosition());
//	SmartDashboard::PutNumber("active trajectory velocity extender", m_extenderMaster->GetActiveTrajectoryVelocity());
//	SmartDashboard::PutNumber("applied motor output extender", m_extenderMaster->GetMotorOutputVoltage());
//
//	SmartDashboard::PutNumber("pivot speed", m_pivot->GetSelectedSensorVelocity(0));
//	SmartDashboard::PutNumber("pivot angle", GetPivotAngle().getDegrees());
//	SmartDashboard::PutNumber("pivot current", m_pivot->GetOutputCurrent());
//	SmartDashboard::PutNumber("pivot error", m_pivot->GetClosedLoopError(0));
//	SmartDashboard::PutNumber("active trajectory position pivot", m_pivot->GetActiveTrajectoryPosition());
//	SmartDashboard::PutNumber("active trajectory velocity pivot", m_pivot->GetActiveTrajectoryVelocity());
//	SmartDashboard::PutNumber("applied motor output pivot", m_pivot->GetMotorOutputVoltage());
//
//	SmartDashboard::PutNumber("extender distance ticks", m_extenderMaster->GetSelectedSensorPosition(0));
}

void Arm::SetPivotAngle(Rotation2D angle) {
	m_pivot->Set(ControlMode::MotionMagic, angle.getDegrees() * RobotParameters::k_encoderTicksPerPivotDegree); //control mode?
	m_pivotAngle = angle;
	SmartDashboard::PutNumber("pivotAngle", angle.getDegrees());
}

Rotation2D Arm::GetPivotAngle() {
	return Rotation2D::fromDegrees(m_pivot->GetSelectedSensorPosition(0) / RobotParameters::k_encoderTicksPerPivotDegree);
}

void Arm::ZeroPivot() {
	m_pivot->SetSelectedSensorPosition(0, 0, 10);
}

double Arm::GetAllowedExtensionPos() {
//	return m_armConstraints.Constrain(m_desiredExtensionSetpoint, m_pivotAngle);
	return m_armConstraints.Constrain(m_desiredExtensionSetpoint, GetPivotAngle());

}

double Arm::ConvertInchesToEncTicks(double inches) {
	return inches * RobotParameters::k_encoderTicksPerExtensionInch;
}

double Arm::GetDesiredExtension() {
	return m_desiredExtensionSetpoint;
}

Rotation2D Arm::GetDesiredPivotAngle() {
	return m_pivotAngle;
}

double Arm::ConvertEncTicksToInches(double ticks) {
	return ticks / RobotParameters::k_encoderTicksPerExtensionInch;
}

double Arm::GetLastCommandedSetpoint() {
//	return
}
