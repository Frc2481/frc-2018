/*
 * SwerveModuleV2.cpp
 *
 *  Created on: Jul 17, 2017
 *      Author: Team2481
 */

#include "RobotParameters.h"
#include <math.h>
#include <sstream>
#include "Components/CTREMagEncoder.h"
#include "Components/GreyhillEncoder.h"
#include "Components/SwerveModule.h"

SwerveModule::SwerveModule(uint32_t driveID, uint32_t steerID,
		const std::string name) {
	m_name = name;
	std::stringstream ss;
	ss << name << "_STEER_ENCODER";
	m_steerMotor = new TalonSRX(steerID);
	m_steerEncoder = new CTREMagEncoder(m_steerMotor, ss.str());
	ss.str("");
	ss << name << "_DRIVE_ENCODER";
	m_driveMotor = new TalonSRX(driveID);
	m_driveEncoder = new GreyhillEncoder(m_driveMotor, ss.str(),
			RobotParameters::k_ticksPerEncoderRev,
			RobotParameters::k_inchesPerWheelRev,
			RobotParameters::k_encoderRevPerWheelRevLowGear,
			RobotParameters::k_encoderRevPerWheelRevHighGear
			);

	m_isCloseLoopControl = false;
	m_angleOptimized = false;
	m_optimizationEnabled = true;
	m_isMoving = false;
	m_motionMagic = false;

	m_driveMotor->SelectProfileSlot(0, 0);
	m_driveMotor->Set(ControlMode::PercentOutput, 0);
	m_driveMotor->Config_kP(0, RobotParameters::k_speedP, 0);
	m_driveMotor->Config_kI(0, RobotParameters::k_speedI, 0);
	m_driveMotor->Config_kD(0, RobotParameters::k_speedD, 0);
	m_driveMotor->Config_kF(0, 0.1722, 0);
	m_driveMotor->Config_IntegralZone(0, 200, 0);
	m_driveMotor->SetSensorPhase(true);
	m_driveMotor->SetInverted(true);

	m_driveMotor->ConfigNominalOutputForward(0.0, 0.0);
	m_driveMotor->ConfigNominalOutputReverse(0.0, 0.0);
	m_driveMotor->ConfigPeakOutputForward(1.0, 0.0);
	m_driveMotor->ConfigPeakOutputReverse(-1.0, 0.0);
//	m_driveMotor->SetMotionMagicAcceleration(m_accel);
//	m_driveMotor->SetMotionMagicCruiseVelocity(m_velocity);

	m_driveMotor->SetNeutralMode(Brake);

	m_steerMotor->SelectProfileSlot(0, 0.0); //Profile 1 PIDf are P = 0.2 f = 1.1
	m_steerMotor->ConfigNominalOutputForward(0,0);
	m_steerMotor->ConfigNominalOutputReverse(0,0);
	m_steerMotor->ConfigPeakOutputForward(1.0, 0.0);
	m_steerMotor->ConfigPeakOutputReverse(-1.0, 0.0);
	m_steerMotor->SetNeutralMode(Brake);
	m_steerMotor->Config_kP(0, RobotParameters::k_steerP, 0);
	m_steerMotor->Config_kI(0, RobotParameters::k_steerI, 0);
	m_steerMotor->Config_kD(0, RobotParameters::k_steerD, 0);
	m_steerMotor->SetSensorPhase(true);
	m_steerMotor->SetInverted(false);
//	m_steerMotor->SetSelectedSensorPosition(m_steerMotor->GetSelectedSensorPosition(0) & 0xFFF, 0, 0);
	m_steerMotor->ConfigAllowableClosedloopError(0, 40, 0);
	m_steerMotor->SetStatusFramePeriod(Status_2_Feedback0, 10, 0);
//	m_steerMotor->SetStatusFrameRateMs(TalonSRX::StatusFrameRateGeneral, 10);
}

SwerveModule::~SwerveModule() {
	// TODO Auto-generated destructor stub
}

Rotation2D SwerveModule::GetAngle() const {
	return m_steerEncoder->GetAngle();
}

void SwerveModule::SetOptimized(bool isOptimized) {
	m_optimizationEnabled = isOptimized;
}

void SwerveModule::SetAngle(Rotation2D angle, bool force) {
	if(m_isMoving || force) {
		Rotation2D currentAngle = m_steerEncoder->GetAngle();
		Rotation2D deltaAngle = currentAngle.rotateBy(angle.inverse());
		if(m_optimizationEnabled &&
		   fabs(deltaAngle.getRadians()) > M_PI_2 &&
		   fabs(deltaAngle.getRadians()) < 3 * M_PI_2) {
			static Rotation2D k180 = Rotation2D::fromRadians(M_PI);
			angle = angle.rotateBy(k180);
			m_angleOptimized = true;
		}
		else {
			m_angleOptimized = false;
		}
		int setpoint = m_steerEncoder->ConvertAngleToSetpoint(angle);
		m_steerMotor->Set(ControlMode::Position, setpoint);
	}
}

bool SwerveModule::IsSteerOnTarget() const {
	return fabs(m_steerMotor->GetClosedLoopError(0)) <= 20;
}

void SwerveModule::SetOpenLoopSpeed(double speed) {
	if(m_angleOptimized) {
		speed *= -1;
	}
	m_driveMotor->Set(ControlMode::PercentOutput, speed);
	m_isMoving = fabs(speed) > .05;
	m_isCloseLoopControl = false;
}

void SwerveModule::SetOpenLoopSteer(double speed) {
	m_steerMotor->Set(ControlMode::PercentOutput, speed);
}

double SwerveModule::GetSpeed()const {
	return m_driveEncoder->GetEncoderSpeed();
}

void SwerveModule::SetCloseLoopDriveDistance(Translation2D distance) {
	double distInches = distance.getX();
	if(m_angleOptimized) {
		distInches *= -1;
	}
	m_driveMotor->Set(ControlMode::MotionMagic, distInches);
	m_isMoving = true;
	m_isCloseLoopControl = true;
}

void SwerveModule::DisableCloseLoopDrive() {
	SetOpenLoopSpeed(0);
}

Translation2D SwerveModule::GetDistance() const {
	return m_driveEncoder->GetDistance();
}

void SwerveModule::ZeroDriveDistance() {
	m_driveEncoder->ResetDistance();
}

double SwerveModule::GetDistanceError() const {
	return m_driveMotor->GetClosedLoopError(0);
}

bool SwerveModule::IsDriveOnTarget() const {
	return GetDistanceError() < 4; //absolute value?
}

void SwerveModule::Set(double speed, Rotation2D angle) {
	if(m_isCloseLoopControl){
		SetAngle(angle, true);
	}
	else {
		SetOpenLoopSpeed(speed);
		SetAngle(angle, false);
	}
}

void SwerveModule::SetBrake(bool brake) {
	m_driveMotor->SetNeutralMode(brake ? Brake : Coast);
}

void SwerveModule::SetMagicAccel(double accel) {
	m_driveMotor->ConfigMotionAcceleration(accel, 0);
}

bool SwerveModule::GetOptimized() {
	return m_angleOptimized;
}

CTREMagEncoder* SwerveModule::GetSteerEncoder() {
	return m_steerEncoder;
}

GreyhillEncoder* SwerveModule::GetDriveEncoder() {
	return m_driveEncoder;
}

double SwerveModule::GetSteerCurrent() const {
	return m_steerMotor->GetOutputCurrent();
}

double SwerveModule::GetDriveCurrent() const {
	return m_driveMotor->GetOutputCurrent();
}

void SwerveModule::Periodic() {
	m_steerEncoder->Periodic();
	m_driveEncoder->Periodic();
}
