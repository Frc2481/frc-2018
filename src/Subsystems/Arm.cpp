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

	m_calLed = new DigitalOutput(0);
	m_canifier = new CANifier(0);

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

	m_extenderMaster->ConfigForwardLimitSwitchSource(LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_Disabled, 0); //todo when wired right enabled
	m_extenderMaster->ConfigReverseLimitSwitchSource(LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_Disabled, 0);

	m_extenderMaster->SetNeutralMode(Brake);

	m_extenderMaster->ConfigSelectedFeedbackSensor(CTRE_MagEncoder_Relative, 0, 0);

	m_extenderMaster->ConfigMotionCruiseVelocity(RobotParameters::k_extenderVelocityUp, 0); //convert to talon speed: encoder count/100 ms / 2.0
	m_extenderMaster->ConfigMotionAcceleration(RobotParameters::k_extenderAccelerationUp, 0); //9200
	m_scale = 1.0;

	m_extenderMaster->SetStatusFramePeriod(Status_2_Feedback0_, 10, 0);
	m_extenderMaster->SetStatusFramePeriod(Status_10_MotionMagic, 100, 0);

	m_extenderMaster->ConfigAllowableClosedloopError(0, 0, 0);

	m_extenderMaster->ConfigPeakCurrentDuration(0, 0);
	m_extenderMaster->ConfigContinuousCurrentLimit(30, 0);
	m_extenderMaster->EnableCurrentLimit(false);
	m_extenderMaster->ConfigPeakCurrentLimit(0, 0);

	m_extenderSlave->ConfigPeakCurrentDuration(0, 0);
	m_extenderSlave->ConfigContinuousCurrentLimit(30, 0);
	m_extenderSlave->EnableCurrentLimit(false);
	m_extenderSlave->ConfigPeakCurrentLimit(0, 0);

	m_extenderSlave->SetInverted(true);
	m_extenderSlave->Set(ControlMode::Follower, 15);

	m_desiredExtensionSetpoint = 0;

	m_pivotMaster = new TalonSRX(PIVOT_MASTER);
	m_pivotSlave = new VictorSPX(PIVOT_SLAVE);

	m_pivotMaster->SelectProfileSlot(0, 0);
	m_pivotMaster->Set(ControlMode::PercentOutput, 0);
	m_pivotMaster->Config_kP(0, RobotParameters::k_pivotP, 0);
	m_pivotMaster->Config_kI(0, RobotParameters::k_pivotI, 0);
	m_pivotMaster->Config_kD(0, RobotParameters::k_pivotD, 0);
	m_pivotMaster->Config_kF(0, RobotParameters::k_pivotF, 0);
	m_pivotMaster->Config_IntegralZone(0, 200, 0);
	m_pivotMaster->SetSensorPhase(true);
	m_pivotMaster->SetInverted(true);

	m_pivotMaster->ConfigNominalOutputForward(0.0, 0.0);
	m_pivotMaster->ConfigNominalOutputReverse(0.0, 0.0);
	m_pivotMaster->ConfigPeakOutputForward(RobotParameters::k_pivotPeakOutputForward, 0.0);
	m_pivotMaster->ConfigPeakOutputReverse(RobotParameters::k_pivotPeakOutputReverse, 0.0);

	m_pivotMaster->ConfigForwardSoftLimitEnable(true, 0);
	m_pivotMaster->ConfigForwardSoftLimitThreshold(5740, 0);
	m_pivotMaster->ConfigReverseSoftLimitEnable(true, 0);
	m_pivotMaster->ConfigReverseSoftLimitThreshold(-5650, 0);

	m_pivotMaster->ConfigForwardLimitSwitchSource(LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyOpen, 0);
	m_pivotMaster->ConfigReverseLimitSwitchSource(LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyOpen, 0);

	m_pivotMaster->ConfigSetParameter(ParamEnum::eClearPosOnLimitF, 0, 0, 0, 0);
	m_pivotMaster->ConfigSetParameter(ParamEnum::eClearPosOnLimitR, 0, 0, 0, 0);

	m_pivotMaster->ConfigContinuousCurrentLimit(25, 0);
	m_pivotMaster->ConfigPeakCurrentLimit(0, 0);
	m_pivotMaster->ConfigPeakCurrentDuration(0, 0);
	m_pivotMaster->EnableCurrentLimit(false); //ToDo: enable current limit

	m_pivotMaster->SetNeutralMode(Brake);

	m_pivotMaster->ConfigSelectedFeedbackSensor(CTRE_MagEncoder_Relative, 0, 0);

	m_pivotMaster->ConfigMotionCruiseVelocity(RobotParameters::k_pivotVelocity, 0);
	m_pivotMaster->ConfigMotionAcceleration(RobotParameters::k_pivotAcceleration, 0);

	m_pivotMaster->SetStatusFramePeriod(Status_2_Feedback0_, 10, 0);
	m_pivotMaster->SetStatusFramePeriod(Status_10_MotionMagic, 100, 0);

	m_pivotMaster->ConfigAllowableClosedloopError(0, 0, 0);

//	m_pivotSlave->Set(ControlMode::Follower, 17);
	m_pivotSlave->Follow(*m_pivotMaster);

	m_pivotSlave->SetInverted(false);

	m_prevExtensionTravellingDown = false;

	m_isPivotZeroed = false;
	m_isExtensionZeroed = false;
	m_armLegal = true;

//	SmartDashboard::PutNumber("pivot velocity config", 0);
//	SmartDashboard::PutNumber("pivot accel config", 0);
//	SmartDashboard::PutNumber("extension velocity config", 0);
//	SmartDashboard::PutNumber("extension accel config", 0);

	m_maxIllegal = 0;
}

Arm::~Arm() {
	// TODO Auto-generated destructor stub
}

void Arm::SetExtensionPosition(double position) {
	if((m_isExtensionZeroed == true) && (m_isPivotZeroed == true)) {
		position = ConvertInchesToEncTicks(position);
		m_extensionSetpoint = position;
		m_extenderMaster->Set(ControlMode::MotionMagic, position);

		SmartDashboard::PutNumber("set position extension", position);

		bool goingDown = GetExtensionPosition() > position; //handle pivot
		if(goingDown != m_prevExtensionTravellingDown) {
			if(goingDown) {
//				m_extenderMaster->SelectProfileSlot(1, 0);
//				m_extenderMaster->ConfigMotionAcceleration(RobotParameters::k_extenderAccelerationDown, 0);
//				m_extenderMaster->ConfigMotionCruiseVelocity(RobotParameters::k_extenderVelocityDown, 0);
			}
			else {
//				m_extenderMaster->SelectProfileSlot(0, 0);
//				m_extenderMaster->ConfigMotionAcceleration(RobotParameters::k_extenderAccelerationUp, 0);
//				m_extenderMaster->ConfigMotionCruiseVelocity(RobotParameters::k_extenderVelocityUp, 0);
			}
			m_prevExtensionTravellingDown = goingDown;
			SmartDashboard::PutNumber("isGoingDown", goingDown);

		}
	}
}

void Arm::SetDesiredExtension(double extension) {
	m_desiredExtensionSetpoint = extension;
}

double Arm::GetExtensionPosition() {
	return ConvertEncTicksToInches(m_extenderMaster->GetSelectedSensorPosition(0));
}

void Arm::ZeroExtension(int pos) {
	for(int i = 0; i < 5; i++) {
		ErrorCode error = m_extenderMaster->SetSelectedSensorPosition(pos, 0, 10);
		if(error == OK) {
			break;
		}
	}
	m_isExtensionZeroed = true;
}

void Arm::SetExtensionOpenLoop(double speed) {
	m_extenderMaster->Set(ControlMode::PercentOutput, speed);
}

bool Arm::IsExtensionOnTarget() {
	return fabs(GetExtensionPosition() - m_extensionSetpoint) < 1.0;
}

void Arm::SetPivotOpenLoop(double speed) {
	m_pivotMaster->Set(ControlMode::PercentOutput, speed);
}

bool Arm::IsPivotOnTarget() {
	return fabs(m_pivotAngle.getDegrees() - GetPivotAngle().getDegrees()) < 3.0;
}

void Arm::SetPivotAccel(int accel) {
	m_pivotMaster->ConfigMotionAcceleration(accel, 0);
}

void Arm::Periodic() {
	//use a slower rate of deacceleration to counteract gravity when coming down

//	if (DriverStation::GetInstance().IsDisabled()) {

		if (m_isExtensionZeroed == false && m_extenderMaster->GetSensorCollection().IsFwdLimitSwitchClosed()) {
			ZeroExtension();
		}

		if (m_isPivotZeroed == false && m_pivotMaster->GetSensorCollection().IsFwdLimitSwitchClosed()) {
			// Zero pivot with offset.
//			ZeroPivot(5476); // Practice
			ZeroPivot(5623); // Competition

		} else if (m_isPivotZeroed == false && m_pivotMaster->GetSensorCollection().IsRevLimitSwitchClosed()) {
			// Zero pivot with offset.
			ZeroPivot(-5429);
		}
//	}

	m_calLed->Set(m_isPivotZeroed && m_isExtensionZeroed);

	if (m_isPivotZeroed && m_isExtensionZeroed) {
		m_canifier->SetLEDOutput(1, CANifier::LEDChannelA);
		m_canifier->SetLEDOutput(0, CANifier::LEDChannelB);
	} else {
		m_canifier->SetLEDOutput(0, CANifier::LEDChannelA);
		m_canifier->SetLEDOutput(1, CANifier::LEDChannelB);
	}

	if(!m_isPivotZeroed) {
		m_pivotMaster->ConfigForwardSoftLimitEnable(false, 0);
		m_pivotMaster->ConfigReverseSoftLimitEnable(false, 0);
	}
	else {
		m_pivotMaster->ConfigForwardSoftLimitEnable(true, 0);
		m_pivotMaster->ConfigReverseSoftLimitEnable(true, 0);
	}

	if(!m_isExtensionZeroed) {
		m_extenderMaster->ConfigForwardSoftLimitEnable(false, 0);
		m_extenderMaster->ConfigReverseSoftLimitEnable(false, 0);
	}
	else {
		m_extenderMaster->ConfigForwardSoftLimitEnable(true, 0);
		m_extenderMaster->ConfigReverseSoftLimitEnable(true, 0);
	}

	StickyFaults masterExtensionFaults;
	m_extenderMaster->GetStickyFaults(masterExtensionFaults);

	StickyFaults slaveExtensionFaults;
	m_extenderSlave->GetStickyFaults(slaveExtensionFaults);

	StickyFaults masterPivotFaults;
	m_pivotMaster->GetStickyFaults(masterPivotFaults);

	StickyFaults slavePivotFaults;
	m_pivotSlave->GetStickyFaults(slavePivotFaults);

	// Save the extension and pivot.  Don't get fouls.
	if ((masterPivotFaults.ResetDuringEn == true) && m_isPivotZeroed) {
		m_desiredExtensionSetpoint = 0;
		SetExtensionPosition(m_desiredExtensionSetpoint);
		m_isPivotZeroed = false;
	} else {
		SetExtensionPosition(GetAllowedExtensionPos());
	}

	SmartDashboard::PutNumber("extension speed", m_extenderMaster->GetSelectedSensorVelocity(0));
	SmartDashboard::PutNumber("extension distance", ConvertEncTicksToInches(m_extenderMaster->GetSelectedSensorPosition(0)));
//	SmartDashboard::PutNumber("extension master current", m_extenderMaster->GetOutputCurrent());
//	SmartDashboard::PutNumber("extension slave current", m_extenderSlave->GetOutputCurrent());
//	SmartDashboard::PutNumber("extension error", m_extenderMaster->GetClosedLoopError(0));

	bool armLegal = ConvertEncTicksToInches(m_extenderMaster->GetSelectedSensorPosition(0)) < GetAllowedExtensionPos();

	SmartDashboard::PutBoolean("arm legal", armLegal);

	double allowedExt = GetAllowedExtensionPos();
	double actualExt = GetExtensionPosition();
	double errorExt = allowedExt - actualExt;

	if(!m_armLegal && armLegal) {
		printf("allowed: %f, actual: %f, setpoint: %f, error: %f, max illegal: %f\n", allowedExt, actualExt, m_extensionSetpoint, errorExt, m_maxIllegal);
		m_maxIllegal = 0;
	}

	if(!armLegal) {
		m_maxIllegal = std::max(m_maxIllegal, errorExt);
	}

	m_armLegal = armLegal;
	if(m_extenderMaster->GetControlMode() == ControlMode::MotionMagic) {
		SmartDashboard::PutNumber("active trajectory position extender",
				ConvertEncTicksToInches(m_extenderMaster->GetActiveTrajectoryPosition()));
//		SmartDashboard::PutNumber("active trajectory velocity extender", m_extenderMaster->GetActiveTrajectoryVelocity());
	}

//	SmartDashboard::PutNumber("applied motor output extender", m_extenderMaster->GetMotorOutputVoltage());

//	SmartDashboard::PutNumber("pivot speed", m_pivotMaster->GetSelectedSensorVelocity(0));
	SmartDashboard::PutNumber("pivot angle", GetPivotAngle().getDegrees());
//	SmartDashboard::PutNumber("pivot ticks", m_pivot->GetSelectedSensorPosition(0));
	SmartDashboard::PutNumber("pivot master current", m_pivotMaster->GetOutputCurrent());
	SmartDashboard::PutNumber("pivot slave current", m_pivotSlave->GetOutputCurrent());
//	SmartDashboard::PutNumber("pivot error", m_pivotMaster->GetClosedLoopError(0) / RobotParameters::k_encoderTicksPerPivotDegree);

//	if(m_pivotMaster->GetControlMode() == ControlMode::MotionMagic) {
//		SmartDashboard::PutNumber("active trajectory position pivot", m_pivotMaster->GetActiveTrajectoryPosition() / RobotParameters::k_encoderTicksPerPivotDegree);
//		SmartDashboard::PutNumber("active trajectory velocity pivot", m_pivotMaster->GetActiveTrajectoryVelocity());
//	}
	SmartDashboard::PutNumber("applied motor output pivot", m_pivotMaster->GetMotorOutputVoltage());

	SmartDashboard::PutNumber("Arm Extension Position", GetExtensionPosition());

//	SmartDashboard::PutNumber("extender distance ticks", m_extenderMaster->GetSelectedSensorPosition(0));
	SmartDashboard::PutBoolean("extension reset master", !masterExtensionFaults.ResetDuringEn);
	SmartDashboard::PutBoolean("extension reset slave", !slaveExtensionFaults.ResetDuringEn);
	SmartDashboard::PutBoolean("pivot reset master", !masterPivotFaults.ResetDuringEn);
	SmartDashboard::PutBoolean("pivot reset slave", !slavePivotFaults.ResetDuringEn);

//	SmartDashboard::PutBoolean("extension limit switch", m_extenderMaster->GetSensorCollection().IsRevLimitSwitchClosed());
//	SmartDashboard::PutBoolean("pivot limit switch forward", m_pivot->GetSensorCollection().IsFwdLimitSwitchClosed());
//	SmartDashboard::PutBoolean("pivot limit switch reverse", m_pivot->GetSensorCollection().IsRevLimitSwitchClosed());
	SmartDashboard::PutBoolean("extension encoder connected", m_extenderMaster->GetSensorCollection().GetPulseWidthRiseToRiseUs() > 0);
	SmartDashboard::PutBoolean("pivot encoder connected", m_pivotMaster->GetSensorCollection().GetPulseWidthRiseToRiseUs() > 0);
	SmartDashboard::PutBoolean("pivot zeroed", m_isPivotZeroed);
	SmartDashboard::PutBoolean("extension zeroed", m_isExtensionZeroed);

//use for tuning
//	m_pivotMaster->ConfigMotionCruiseVelocity(SmartDashboard::GetNumber("pivot velocity config",0), 0);
//	m_pivotMaster->ConfigMotionAcceleration(SmartDashboard::GetNumber("pivot accel config",0), 0);
//	m_extenderMaster->ConfigMotionCruiseVelocity(SmartDashboard::GetNumber("extension velocity config",0), 0);
//	m_extenderMaster->ConfigMotionAcceleration(SmartDashboard::GetNumber("extension accel config",0), 0);
}

void Arm::SetPivotAngle(Rotation2D angle) {
	if((m_isExtensionZeroed == true) && (m_isPivotZeroed == true)) {
		m_pivotMaster->Set(ControlMode::MotionMagic, angle.getDegrees() * RobotParameters::k_encoderTicksPerPivotDegree); //control mode?
		m_pivotAngle = angle;
		SmartDashboard::PutNumber("pivotAngle", angle.getDegrees());
	}
}

Rotation2D Arm::GetPivotAngle() {
	return Rotation2D::fromDegrees(m_pivotMaster->GetSelectedSensorPosition(0) / RobotParameters::k_encoderTicksPerPivotDegree);
}

void Arm::ZeroPivot(int pos) {
	for(int i = 0; i < 5; i++) {
		ErrorCode error = m_pivotMaster->SetSelectedSensorPosition(pos, 0, 10);
		if(error == OK) {
			break;
		}
	}
	m_isPivotZeroed = true;
	m_pivotMaster->ClearStickyFaults(0);
	m_pivotSlave->ClearStickyFaults(0);
}

double Arm::GetAllowedExtensionPos() {
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

void Arm::SetExtentionMotionScaling(double scale) {
	m_scale = scale;
	m_extenderMaster->ConfigMotionCruiseVelocity(RobotParameters::k_extenderVelocityUp * m_scale, 0);
	m_extenderMaster->ConfigMotionAcceleration(RobotParameters::k_extenderAccelerationUp * m_scale, 0);
}

void Arm::ClearStickyFaults() {
	m_extenderMaster->ClearStickyFaults(0);
	m_extenderSlave->ClearStickyFaults(0);
	m_pivotMaster->ClearStickyFaults(0);
	m_pivotSlave->ClearStickyFaults(0);
}

bool Arm::IsPivotZeroed() {
	return m_isPivotZeroed;
}

bool Arm::IsExtensionZeroed() {
	return m_isExtensionZeroed;
}
