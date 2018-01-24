/*
 * DriveTrain.cpp
 *
 *  Created on: Aug 28, 2017
 *      Author: Team2481
 */

#include <Commands/DriveWithJoystickCommand.h>
#include <Commands/CalibrateDriveTrainCommand.h>
#include <Kinematics.h>
#include <Subsystems/DriveTrain.h>
#include <Components/SwerveModule.h>
#include "../RobotMap.h"

#include <cmath>
#include <algorithm>
#include "AHRS.h"
#include "Components/CTREMagEncoder.h"
#include "Components/GreyhillEncoder.h"
#include "RobotParameters.h"
#include "../Commands/ObserverResetPosCommand.h"
#include "WPILib.h"

DriveTrain2017::DriveTrain2017() : Subsystem("DriveTrain2017"),
	m_flWheel(new SwerveModuleV2(FRONT_LEFT_DRIVE, FRONT_LEFT_STEER, "FRONT_LEFT")),
	m_frWheel(new SwerveModuleV2(FRONT_RIGHT_DRIVE, FRONT_RIGHT_STEER, "FRONT_RIGHT")),
	m_brWheel(new SwerveModuleV2(BACK_RIGHT_DRIVE, BACK_RIGHT_STEER, "BACK_RIGHT")),
	m_blWheel(new SwerveModuleV2(BACK_LEFT_DRIVE, BACK_LEFT_STEER, "BACK_LEFT")),
	m_shifter(new Solenoid(SHIFTER)),
	m_imu(new AHRS(SPI::kMXP)),
	m_isFieldCentric(false),
	m_isForward(true),
	m_xPos(0), m_yPos(0), m_twist(0), m_heading(0), m_headingCorrection(0), m_roll(0), m_pitch(0), m_timestamp(1.0) {

	m_prevAngle = 90.0;

	m_headingCorrection = false;
	m_encRotationPerDegrees = Preferences::GetInstance()->GetDouble("ENCODER_ROTATIONS_PER_DEGREE",0);
	m_pHeadingCorrection = Preferences::GetInstance()->GetDouble("P_HEADING_CORRECTION",0);
	m_originX = 0.0f;
	m_originY = 0.0f;

	m_oldFlAngle = Rotation2D(1, 0, true);
	m_oldFrAngle = Rotation2D(1, 0, true);
	m_oldBlAngle = Rotation2D(1, 0, true);
	m_oldBrAngle = Rotation2D(1, 0, true);

	m_oldFlDistance = Translation2D(0, 0);
	m_oldFrDistance = Translation2D(0, 0);
	m_oldBlDistance = Translation2D(0, 0);
	m_oldBrDistance = Translation2D(0, 0);

	SmartDashboard::PutData(new CalibrateDriveTrainCommand());
	SmartDashboard::PutData(new ObserverResetPosCommand());

	m_observer.SetRobotPos(RigidTransform2D(Translation2D(0, 0), Rotation2D(1, 0, true)), 0.0);

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

	Kinematics::SwerveInverseKinematics(translation, twist,
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
	revolutions = inches / RobotParameters::k_inchesPerWheelRev;
	revolutions *= RobotParameters::k_encoderRevPerWheelRev;
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

void DriveTrain2017::ResetRobotPose() {
	m_observer.ResetPose();
}

void DriveTrain2017::Periodic() {
	double timeStamp = RobotController::GetFPGATime();
	double deltaTimestamp = timeStamp - m_timestamp;
	m_timestamp = timeStamp;

	Rotation2D newFlAngle = m_flWheel->GetAngle();
//	if(m_flWheel->GetOptimized()) {
//		newFlAngle = newFlAngle.inverse();
//	}

	Rotation2D deltaFlAngle = newFlAngle.rotateBy(m_oldFlAngle.inverse());
	m_oldFlAngle = newFlAngle;

	Translation2D newFlDistance = m_flWheel->GetDistance(); //TODO: Inverse ??
	Translation2D deltaFlDistance = newFlDistance.translateBy(m_oldFlDistance.inverse());
	SmartDashboard::PutNumber("old FL distance", m_oldFlDistance.getX());
	m_oldFlDistance = newFlDistance;

	Rotation2D newFrAngle = m_frWheel->GetAngle();
//	if(m_frWheel->GetOptimized()) {
//			newFrAngle = newFrAngle.inverse();
//	}

	Rotation2D deltaFrAngle = newFrAngle.rotateBy(m_oldFrAngle.inverse());
	m_oldFrAngle = newFrAngle;

	Translation2D newFrDistance = m_frWheel->GetDistance();
	Translation2D deltaFrDistance = newFrDistance.translateBy(m_oldFrDistance.inverse());
	m_oldFrDistance = newFrDistance;


	Rotation2D newBlAngle = m_blWheel->GetAngle();
//	if(m_blWheel->GetOptimized()) {
//			newBlAngle = newBlAngle.inverse();
//	}

	Rotation2D deltaBlAngle = newBlAngle.rotateBy(m_oldBlAngle.inverse());
	m_oldBlAngle = newBlAngle;

	Translation2D newBlDistance = m_blWheel->GetDistance();
	Translation2D deltaBlDistance = newBlDistance.translateBy(m_oldBlDistance.inverse());
	m_oldBlDistance = newBlDistance;


	Rotation2D newBrAngle = m_brWheel->GetAngle();
//	if(m_brWheel->GetOptimized()) {
//			newBrAngle = newBrAngle.inverse();
//	}

	Rotation2D deltaBrAngle = newBrAngle.rotateBy(m_oldBrAngle.inverse());
	m_oldBrAngle = newBrAngle;

	Translation2D newBrDistance = m_brWheel->GetDistance();
	Translation2D deltaBrDistance = newBrDistance.translateBy(m_oldBrDistance.inverse());
	m_oldBrDistance = newBrDistance;

	RigidTransform2D::Delta deltaFlVelocity = RigidTransform2D::Delta::fromDelta(deltaFlDistance.getX(), 0, 0, deltaTimestamp);
	RigidTransform2D::Delta deltaFrVelocity = RigidTransform2D::Delta::fromDelta(deltaFrDistance.getX(), 0, 0, deltaTimestamp);
	RigidTransform2D::Delta deltaBlVelocity = RigidTransform2D::Delta::fromDelta(deltaBlDistance.getX(), 0, 0, deltaTimestamp);
	RigidTransform2D::Delta deltaBrVelocity = RigidTransform2D::Delta::fromDelta(deltaBrDistance.getX(), 0, 0, deltaTimestamp);

	double obsDistanceThresh = 500;
	if(fabs(deltaFlDistance.getX() < obsDistanceThresh) && fabs(deltaFrDistance.getX() < obsDistanceThresh) &&
	   fabs(deltaBlDistance.getX() < obsDistanceThresh) && fabs(deltaBrDistance.getX() < obsDistanceThresh)) {

		m_observer.AddDriveTrainObservation(newFlAngle, deltaFlVelocity,
											newFrAngle, deltaFrVelocity,
											newBlAngle, deltaBlVelocity,
											newBrAngle, deltaBrVelocity, m_timestamp);
	}

	RigidTransform2D observerPos =  m_observer.GetRobotPos(m_timestamp);

	SmartDashboard::PutNumber("Field X", observerPos.getTranslation().getX());
	SmartDashboard::PutNumber("Field Y", observerPos.getTranslation().getY());
	SmartDashboard::PutNumber("Field Heading", observerPos.getRotation().getDegrees());

	SmartDashboard::PutNumber("FL angle", m_flWheel->GetAngle().getDegrees());
	SmartDashboard::PutNumber("FR angle", m_frWheel->GetAngle().getDegrees());
	SmartDashboard::PutNumber("BL angle", m_blWheel->GetAngle().getDegrees());
	SmartDashboard::PutNumber("BR angle", m_brWheel->GetAngle().getDegrees());

	SmartDashboard::PutNumber("FL distance", m_flWheel->GetDistance().getX());
	SmartDashboard::PutNumber("FR distance", m_frWheel->GetDistance().getX());
	SmartDashboard::PutNumber("BL distance", m_blWheel->GetDistance().getX());
	SmartDashboard::PutNumber("BR distance", m_brWheel->GetDistance().getX());
}

//This Method must be called when when all 8 swerve modules are on.
void DriveTrain2017::CheckDiagnostics() {
//	TODO:figure out how to see if sensor is present
	SmartDashboard::PutBoolean("FL Drive Enc Present", std::abs(m_flWheel->GetDriveEncoder()->GetEncoderTicks()) > 100);
	SmartDashboard::PutBoolean("FR Drive Enc Present", std::abs(m_frWheel->GetDriveEncoder()->GetEncoderTicks()) > 100);
	SmartDashboard::PutBoolean("BL Drive Enc Present", std::abs(m_blWheel->GetDriveEncoder()->GetEncoderTicks()) > 100);
	SmartDashboard::PutBoolean("BR Drive Enc Present", std::abs(m_brWheel->GetDriveEncoder()->GetEncoderTicks()) > 100);

	SmartDashboard::PutBoolean("FL Steer Enc Calibrated", m_flWheel->GetSteerEncoder()->IsCalibrated());
	SmartDashboard::PutBoolean("FR Steer Enc Calibrated", m_frWheel->GetSteerEncoder()->IsCalibrated());
	SmartDashboard::PutBoolean("BL Steer Enc Calibrated", m_blWheel->GetSteerEncoder()->IsCalibrated());
	SmartDashboard::PutBoolean("BR Steer Enc Calibrated", m_brWheel->GetSteerEncoder()->IsCalibrated());

	bool flDriveMotorPresent = m_flWheel->GetDriveCurrent() > 0.1;
	bool frDriveMotorPresent = m_frWheel->GetDriveCurrent() > 0.1;
	bool blDriveMotorPresent = m_blWheel->GetDriveCurrent() > 0.1;
	bool brDriveMotorPresent = m_brWheel->GetDriveCurrent() > 0.1;

	bool flSteerMotorPresent = m_flWheel->GetSteerCurrent() > 0.1;
	bool frSteerMotorPresent = m_frWheel->GetSteerCurrent() > 0.1;
	bool blSteerMotorPresent = m_blWheel->GetSteerCurrent() > 0.1;
	bool brSteerMotorPresent = m_brWheel->GetSteerCurrent() > 0.1;

	SmartDashboard::PutBoolean("Front Left Drive Motor Present", flDriveMotorPresent);
	SmartDashboard::PutBoolean("Front Right Drive Motor Present", frDriveMotorPresent);
	SmartDashboard::PutBoolean("Back Left Drive Motor Present", blDriveMotorPresent);
	SmartDashboard::PutBoolean("Back Right Drive Motor Present", brDriveMotorPresent);

	SmartDashboard::PutBoolean("Front Left Steer Motor Present", flSteerMotorPresent);
	SmartDashboard::PutBoolean("Front Right Steer Motor Present", frSteerMotorPresent);
	SmartDashboard::PutBoolean("Back Left Steer Motor Present", blSteerMotorPresent);
	SmartDashboard::PutBoolean("Back Right Steer Motor Present", brSteerMotorPresent);

	bool allMotorsPresent = flDriveMotorPresent && frDriveMotorPresent && blDriveMotorPresent && brDriveMotorPresent &&
					flSteerMotorPresent && frSteerMotorPresent && blSteerMotorPresent && brSteerMotorPresent;

	SmartDashboard::PutBoolean("All Drive Train Motors Present", allMotorsPresent);
}
