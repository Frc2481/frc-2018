//#pragma once
//#include "WPILib.h"
//#include <utils/Constants.h>
//#include <utils/GoalTracker.h>
//#include <utils/InterpolatingDouble.h>
//#include <utils/InterpolatingMap.h>
//#include <utils/Kinematics.h>
//#include <utils/RigidTransform2D.h>
//#include <utils/Rotation2D.h>
//#include <utils/Translation2D.h>
//#include <vision/LiftTarget.h>
//#include <mutex>
//
//#include "AimingParameters.h"
//#include "Vision/TargetInfo.h"
//
//class RobotChains {
//private:
//	RobotChains();
//	InterpolatingMap<InterpolatingDouble, InterpolatingDouble> m_skewAngleMap;
//	RigidTransform2D kVehicleToTurretFixed;
//	RigidTransform2D kVehicleToGearCameraFixed;
//	RigidTransform2D kTurretRotatingToCamera;
//	RigidTransform2D kVehicleToGearFlickerFixed;
//	Rotation2D m_angle;
//	std::recursive_mutex m_mutex;
//	double m_gearCurrentAngle;
//	double m_gearTargetAngle;
//	double m_gearDistance;
//	double m_boilerCurrentAngle;
//	double m_boilerTargetAngle;
//	double m_boilerDistance;
//
//protected:
//	InterpolatingMap<InterpolatingDouble, RigidTransform2D> m_fieldToVehicle;
//	RigidTransform2D::Delta m_vehicleVelocity;
//	InterpolatingMap<InterpolatingDouble, Rotation2D> m_turretRotation;
//	GoalTracker m_goalLiftTracker;
//	GoalTracker m_goalBoilerTracker;
//	Rotation2D m_gearCameraPitchCorrection;
//	Rotation2D m_gearCameraYawCorrection;
//	Rotation2D m_boilerCameraPitchCorrection;
//	Rotation2D m_boilerCameraYawCorrection;
//	double m_gearDifferentialHeight;
//	double m_boilerDifferentialHeight;
//
//public:
//	//static RobotState getInstance();
//	const int kObservationBufferSize = 100;
//	const double kMaxTargetAge = 0.4;
//	//const RigidTransform2D kVehicleToTurretFixed = RigidTransform2D(Translation2D(Constants::kTurretXOffset, Constants::kTurretYOffset), Rotation2D::fromDegrees(Constants::kTurretAngleOffsetDegrees));
//	//const RigidTransform2D kTurretRotatingToCamera = RigidTransform2D(Translation2D(Constants::kCameraXOffset, Constants::kCameraYOffset), Rotation2D());
//	void reset(double startTime, const RigidTransform2D &initialFieldToVehicle, const Rotation2D &initialTurretRotation);
//	RigidTransform2D getFieldToVehicle(double timeStamp);
//	RigidTransform2D getPredictedFieldToVehicle(double lookAheadTime);
//	Rotation2D getTurretRotation(double timeStamp);
//	RigidTransform2D getFieldToTurretRotated(double timeStamp);
//	RigidTransform2D getFieldToCamera(double timeStamp);
//	void addFieldToVehicleObservation(double timeStamp, RigidTransform2D observation);
//	void addTurretRotationObservation(double timeStamp, Rotation2D observation);
//	void addObservations(double timeStamp, RigidTransform2D field_to_vehicle,/* Rotation2D turret_rotation,*/ RigidTransform2D::Delta velocity);
//	void addVisionUpdate(double timeStamp, TargetInfo visionUpdate);
//	void addVisionUpdateGear(double timeStamp, LiftTarget gearTarget);
//	void addVisionUpdateBoiler(double timeStamp, TargetInfo boilerTarget);
//	std::list<AimingParameters> getGearAimingParameters(double currentTimeStamp); //std::comparator);
//	std::list<AimingParameters> getBoilerAimingParameters(double currentTimeStamp);
//	std::list<RigidTransform2D> getCaptureTimeFieldToGoal();
//	Rotation2D getLatestTurretRotation();
//	RigidTransform2D getLatestFieldToVehicle();
//	void resetVision();
//	RigidTransform2D generateOdometryFromSensors(double frEncoderDeltaDistance, double flEncoderDeltaDistance, double brEncoderDeltaDistance,
//		double blEncoderDeltaDistance, double frRotationDeltaDistance, double flRotationDeltaDistance, double brRotationDeltaDistance, double blRotationDeltaDistance, Rotation2D currentGyroAngle);
//	void outputToSmartDashboard();
//	void setVehicleToTurretFixed();
//	void setTurretRotatingToCamera();
//	GoalTracker getGoalTracker();
//	static RobotChains* getInstance();
//	//double getInterpolatedGearAngle(double skew);
//};
