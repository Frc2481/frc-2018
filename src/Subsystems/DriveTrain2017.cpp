/*
 * DriveTrain2017.cpp
 *
 *  Created on: Aug 28, 2017
 *      Author: Team2481
 */

#include <Subsystems/DriveTrain2017.h>
#include <Commands/DriveWithJoystickCommand.h>
#include <Commands/CalibrateDriveTrainCommand.h>
#include <Kinematics.h>
#include "../RobotMap.h"

#include <cmath>
#include <algorithm>
//#include "Components/PersistedSettings.h"
//#include "RoboUtils.h"
#include "AHRS.h"
#include "SwerveModuleV2.h"
#include "Components/SwerveModuleV2Constants.h"
#include "../InverseKinematicsV2.h"


DriveTrain2017::DriveTrain2017() : Subsystem("DriveTrain2017"),
	m_flWheel(new SwerveModuleV2(FRONT_LEFT_DRIVE, FRONT_LEFT_STEER, "FRONT_LEFT")),
	m_frWheel(new SwerveModuleV2(FRONT_RIGHT_DRIVE, FRONT_RIGHT_STEER, "FRONT_RIGHT")),
	m_brWheel(new SwerveModuleV2(BACK_RIGHT_DRIVE, BACK_RIGHT_STEER, "BACK_RIGHT")),
	m_blWheel(new SwerveModuleV2(BACK_LEFT_DRIVE, BACK_LEFT_STEER, "BACK_LEFT")),
	m_shifter(new Solenoid(SHIFTER)),
	m_imu(new AHRS(SPI::kMXP)),
	m_isFieldCentric(false),
	m_isForward(true),
	m_xPos(0), m_yPos(0), m_twist(0), m_heading(0), m_headingCorrection(0), m_roll(0), m_pitch(0) {

	m_prevAngle = 90.0;

	m_headingCorrection = false;
	m_encRotationPerDegrees = Preferences::GetInstance()->GetDouble("ENCODER_ROTATIONS_PER_DEGREE",0);
	m_pHeadingCorrection = Preferences::GetInstance()->GetDouble("P_HEADING_CORRECTION",0);
	m_originX = 0.0f;
	m_originY = 0.0f;

	SmartDashboard::PutData(new CalibrateDriveTrainCommand());


}

DriveTrain2017::~DriveTrain2017() {
	delete m_flWheel;
	delete m_frWheel;
	delete m_blWheel;
	delete m_brWheel;
	delete m_shifter;
	delete m_imu;
}

void DriveTrain2017::InitDefaultCommand() {
	SetDefaultCommand(new DriveWithJoystickCommand());
}

void DriveTrain2017::Drive(double xPos, double yPos, double twist) {
	m_xPos = xPos;
	m_yPos = yPos;
	m_twist = twist;

	Translation2D translation(xPos, yPos); //xPos = STR & yPos = FWD
	Rotation2D rotation = Rotation2D::fromDegrees(twist); //don't store twist as angle

	Rotation2D gyroAngle = GetHeading();

	if (m_headingCorrection) {
		gyroAngle.rotateBy(m_headingCorrectionOffset);
		twist = gyroAngle.getDegrees() * m_pHeadingCorrection;
	}
	twist *= -1;

	if (m_isFieldCentric) {
		m_heading = -(gyroAngle.getDegrees());
		translation.rotateBy(gyroAngle);
		twist *= 0.1;
	}
	else {
		  //limit twist speed while not in field centric
		twist *= .05;
	}



	if (!m_isForward) { //used for gare-e
		translation.setY(-translation.getY());
		translation.setX(-translation.getX());
	}

	if(fabs(m_originX) > 0.1f || fabs(m_originY) > 0.1f){
		twist *= -1;
	}

	double flWheelSpeed;
	double frWheelSpeed;
	double blWheelSpeed;
	double brWheelSpeed;
	Rotation2D flWheelAngle;
	Rotation2D frWheelAngle;
	Rotation2D blWheelAngle;
	Rotation2D brWheelAngle;

	InverseKinematicsV2::SwerveInverseKinematics(translation, twist,
			flWheelSpeed, frWheelSpeed, blWheelSpeed, brWheelSpeed,
			flWheelAngle, frWheelAngle, blWheelAngle, brWheelAngle);

	m_flWheel->Set(flWheelSpeed, flWheelAngle);
	SmartDashboard::PutNumber("flWheelSpeed", flWheelSpeed);
	SmartDashboard::PutNumber("flWheelAngle", flWheelAngle.getDegrees());
	m_frWheel->Set(frWheelSpeed, frWheelAngle);
	m_blWheel->Set(blWheelSpeed, blWheelAngle);
	m_brWheel->Set(brWheelSpeed, brWheelAngle);

	SmartDashboard::PutNumber("twist", twist);
	SmartDashboard::PutNumber("xPos", translation.getX());
	SmartDashboard::PutNumber("yPos", translation.getY());
}

void DriveTrain2017::SetOrigin(double xPos, double yPos) {
	m_originX = xPos;
	m_originY = yPos;
}

double DriveTrain2017::GetXOrigin() const{
	return m_originX;
}

double DriveTrain2017::GetYOrigin() const{
	return m_originY;
}

float DriveTrain2017::GetRoll() const{
	return m_imu->GetRoll();
}

float DriveTrain2017::GetPitch() const{
	return m_imu->GetPitch();
}

void DriveTrain2017::PeriodicUpdate() {
	Drive(m_xPos, m_yPos, m_twist);
}

void DriveTrain2017::SetBrake(bool brake) {
	m_flWheel->SetBrake(brake);
	m_frWheel->SetBrake(brake);
	m_blWheel->SetBrake(brake);
	m_brWheel->SetBrake(brake);
}

const Rotation2D& DriveTrain2017::GetGyroCorrectionOffset() const{
	return m_headingCorrectionOffset;
}

void DriveTrain2017::SetGyroCorrectionOffset(Rotation2D &offset) {
	m_headingCorrectionOffset = offset;
}

void DriveTrain2017::Shift(bool state){
	m_shifter->Set(state);
}

SwerveModuleV2* DriveTrain2017::GetModule(DriveTrain2017::SwerveModuleType module) const{
	if(module == FRONT_RIGHT_MODULE){
		return m_frWheel;
	}
	else if(module == FRONT_LEFT_MODULE){
		return m_flWheel;
	}
	else if(module == BACK_RIGHT_MODULE){
		return m_brWheel;
	}
	else if(module == BACK_LEFT_MODULE){
		return m_blWheel;
	}
	return 0;
}

Rotation2D DriveTrain2017::GetHeading() const{
	return Rotation2D::fromDegrees(m_imu->GetAngle());
}

//Rotation2D DriveTrain2017::GetHeading() const{
//	return Rotation2D::fromDegrees(m_imu->GetAngle()); // corrected version
//}
//
//Rotation2D DriveTrain2017::GetIMUTimestamp() const{
//	return Rotation2D::fromDegrees(m_imu->GetLastSensorTimestamp());
//}

void DriveTrain2017::DriveCloseLoopDistance(Translation2D setpoint) {
	m_motionSetpoint = setpoint;
	m_flWheel->SetCloseLoopDriveDistance(m_motionSetpoint);
	m_frWheel->SetCloseLoopDriveDistance(m_motionSetpoint.inverse());
	m_blWheel->SetCloseLoopDriveDistance(m_motionSetpoint);
	m_brWheel->SetCloseLoopDriveDistance(m_motionSetpoint.inverse());
}

Translation2D DriveTrain2017::GetMotionMagicSetpoint() const{
	return m_motionSetpoint;
}

double DriveTrain2017::ComputeDriveDistanceInchestoEncoderRotations(double inches) {
	double revolutions;
	revolutions = inches / SwerveModuleV2Constants::k_inchesPerRev;
	revolutions *= SwerveModuleV2Constants::k_encoderRevPerWheelRev;
	return revolutions;
}

void DriveTrain2017::Stop() {
	Drive(0, 0, 0);
}

void DriveTrain2017::SetFieldCentric(bool fieldCentric) {
	m_isFieldCentric = fieldCentric;
}

void DriveTrain2017::SetForward(bool forward) {
	m_isForward = forward;
}

void DriveTrain2017::SetHeadingCorrection(bool headingCorrection) {
	m_headingCorrection = headingCorrection;
}

void DriveTrain2017::ZeroGyro() {
	m_imu->ZeroYaw();
}

bool DriveTrain2017::IsHeadingCorrection() const {
	return m_headingCorrection;
}

bool DriveTrain2017::IsSteerOnTarget() const{
	return m_flWheel->IsSteerOnTarget() && m_frWheel->IsSteerOnTarget()
			&& m_blWheel->IsSteerOnTarget() && m_brWheel->IsSteerOnTarget();
}

void DriveTrain2017::SetMotionMagicAccel(double accel) {
	m_flWheel->SetMagicAccel(accel);
	m_frWheel->SetMagicAccel(accel);
	m_blWheel->SetMagicAccel(accel);
	m_brWheel->SetMagicAccel(accel);
}

double DriveTrain2017::GetDriveDistance() const{
	return std::max(fabs(m_flWheel->GetDistance().getX()),
			std::max(fabs(m_frWheel->GetDistance().getX()),
			std::max(fabs(m_blWheel->GetDistance().getX()),
					fabs(m_brWheel->GetDistance().getX()))));
}

bool DriveTrain2017::IsShifted() const{
	return m_shifter->Get();
}

double DriveTrain2017::ComputeDegreesToEncoderRotations(double degrees) {
	return (degrees * m_encRotationPerDegrees);
}

bool DriveTrain2017::IsDriveOnTarget() const {
	return m_flWheel->IsDriveOnTarget() && m_frWheel->IsDriveOnTarget()
			&& m_blWheel->IsDriveOnTarget() && m_brWheel->IsDriveOnTarget(); //make robust against encoder failure
}

void DriveTrain2017::Periodic() {
	//RigidTransform2D robotCenterVel = ForwardKinematicsDriveTrain(flWheelAngle, flWheelSpeed, frWheelAngle, frWheelSpeed,
	//		blWheelAngle, blWheelSpeed, brWheelAngle, brWheelSpeed);

	//m_observer.AddDriveTrainObservation(robotCenterVel, 0.0); //To-do: timestamp

	SmartDashboard::PutNumber("BR Raw Angle", m_brWheel->GetEncoder()->GetRawAngle().getDegrees());
	SmartDashboard::PutNumber("BR Angle", m_brWheel->GetEncoder()->GetAngle().getDegrees());

	SmartDashboard::PutNumber("BL Raw Angle", m_blWheel->GetEncoder()->GetRawAngle().getDegrees());
	SmartDashboard::PutNumber("BL Angle", m_blWheel->GetEncoder()->GetAngle().getDegrees());

	SmartDashboard::PutNumber("FR Raw Angle", m_frWheel->GetEncoder()->GetRawAngle().getDegrees());
	SmartDashboard::PutNumber("FR Angle", m_frWheel->GetEncoder()->GetAngle().getDegrees());

	SmartDashboard::PutNumber("FL Raw Angle", m_flWheel->GetEncoder()->GetRawAngle().getDegrees());
	SmartDashboard::PutNumber("FL Angle", m_flWheel->GetEncoder()->GetAngle().getDegrees());
}
