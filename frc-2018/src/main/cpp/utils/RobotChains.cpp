//#include <utils/RobotChains.h>
//#include <cmath>
//
//RobotChains::RobotChains() : m_vehicleVelocity(0,0,0) {
//	reset(0, RigidTransform2D(), Rotation2D());
//	m_skewAngleMap.put(InterpolatingDouble(-12.58), InterpolatingDouble(-45));
//	m_skewAngleMap.put(InterpolatingDouble(-6.78),InterpolatingDouble(-30));
//	m_skewAngleMap.put(InterpolatingDouble(-1.838), InterpolatingDouble(-15));
//	m_skewAngleMap.put(InterpolatingDouble(-0.52),InterpolatingDouble(0));
//	m_skewAngleMap.put(InterpolatingDouble(1.838),InterpolatingDouble(15));
//	m_skewAngleMap.put(InterpolatingDouble(6.78),InterpolatingDouble(30));
//	m_skewAngleMap.put(InterpolatingDouble(12.58),InterpolatingDouble(45));
////	m_instance = RobotState();
//}
////
////RobotState RobotState::getInstance() {
////	return m_instance;
////}
//
//void RobotChains::reset(double startTime, const RigidTransform2D &initialFieldToVehicle, const Rotation2D &initialTurretRotation) {
//	std::lock_guard<std::recursive_mutex> lk(m_mutex);
//	m_fieldToVehicle = InterpolatingMap<InterpolatingDouble, RigidTransform2D>(kObservationBufferSize);
//	m_fieldToVehicle.put(InterpolatingDouble(startTime), initialFieldToVehicle);
//	m_vehicleVelocity = RigidTransform2D::Delta(0, 0, 0);
//	m_turretRotation = InterpolatingMap<InterpolatingDouble, Rotation2D>(kObservationBufferSize);
//	m_turretRotation.put(InterpolatingDouble(startTime), initialTurretRotation);
//	m_goalLiftTracker = GoalTracker();
////	m_goalBoilerTracker = GoalTracker();
//	m_gearCameraPitchCorrection = Rotation2D::fromDegrees(-Constants::kGearCameraPitchAngleDegrees);
//	m_gearCameraYawCorrection = Rotation2D::fromDegrees(-Constants::kGearCameraYawAngleDegrees);
//	m_boilerCameraPitchCorrection = Rotation2D::fromDegrees(-Constants::kBoilerCameraPitchAngleDegrees);
//	m_boilerCameraYawCorrection = Rotation2D::fromDegrees(-Constants::kBoilerCameraYawAngleDegrees);
//	m_gearDifferentialHeight = Constants::kGearCenterOfTargetHeight - Constants::kGearCameraZOffset;
//	m_boilerDifferentialHeight = Constants::kBoilerCenterOfTargetHeight - Constants::kBoilerCameraZOffset;
//}
//
//RigidTransform2D RobotChains::getFieldToVehicle(double timeStamp) {
//	std::lock_guard<std::recursive_mutex> lk(m_mutex);
//	return m_fieldToVehicle.getInterpolated(InterpolatingDouble(timeStamp));
//}
//
//RigidTransform2D RobotChains::getPredictedFieldToVehicle(double lookAheadTime) {
//	std::lock_guard<std::recursive_mutex> lk(m_mutex);
//	return getLatestFieldToVehicle().transformBy(RigidTransform2D::fromVelocity(RigidTransform2D::Delta(m_vehicleVelocity.m_dx * lookAheadTime,
//		m_vehicleVelocity.m_dy * lookAheadTime, m_vehicleVelocity.m_dtheta * lookAheadTime)));
//}
//
//Rotation2D RobotChains::getTurretRotation(double timeStamp) {
//	std::lock_guard<std::recursive_mutex> lk(m_mutex);
//	return m_turretRotation.getInterpolated(InterpolatingDouble(timeStamp));
//}
//
//RigidTransform2D RobotChains::getFieldToTurretRotated(double timeStamp) {
//	std::lock_guard<std::recursive_mutex> lk(m_mutex);
//	InterpolatingDouble key = InterpolatingDouble(timeStamp);
//	return m_fieldToVehicle.getInterpolated(key).transformBy(kVehicleToTurretFixed).transformBy(RigidTransform2D::fromRotation(m_turretRotation.getInterpolated(key)));
//}
//
//RigidTransform2D RobotChains::getFieldToCamera(double timeStamp) {
//	std::lock_guard<std::recursive_mutex> lk(m_mutex);
//	return getFieldToTurretRotated(timeStamp).transformBy(kVehicleToGearCameraFixed);
//}
//
//void RobotChains::addFieldToVehicleObservation(double timeStamp, RigidTransform2D observation) {
//	std::lock_guard<std::recursive_mutex> lk(m_mutex);
//	m_fieldToVehicle.put(InterpolatingDouble(timeStamp), observation);
//}
//
//void RobotChains::addTurretRotationObservation(double timeStamp, Rotation2D observation) {
//	std::lock_guard<std::recursive_mutex> lk(m_mutex);
//	m_turretRotation.put(InterpolatingDouble(timeStamp), observation);
//}
//
//void RobotChains::addObservations(double timeStamp, RigidTransform2D field_to_vehicle, /*Rotation2D turret_rotation,*/ RigidTransform2D::Delta velocity) {
//	std::lock_guard<std::recursive_mutex> lk(m_mutex);
//	addFieldToVehicleObservation(timeStamp, field_to_vehicle);
//	//addTurretRotationObservation(timeStamp, turret_rotation);
//	m_vehicleVelocity = velocity;
//}
//
//void RobotChains::addVisionUpdateGear(double timeStamp, LiftTarget gearTarget) {
//	std::lock_guard<std::recursive_mutex> lk(m_mutex);
//	RigidTransform2D fieldToCamera = getFieldToCamera(timeStamp);
//
//	double yDeadBand = (gearTarget.getY() > -Constants::kCameraDeadBand && gearTarget.getY() < Constants::kCameraDeadBand) ? 0.0 : gearTarget.getY();
//
//	double xYaw = gearTarget.getX() * m_gearCameraYawCorrection.getCos() + yDeadBand * m_gearCameraYawCorrection.getSin();
//	double yYaw = yDeadBand * m_gearCameraYawCorrection.getCos() - gearTarget.getX() * m_gearCameraYawCorrection.getSin();
//	double zYaw = gearTarget.getZ();
//
//	double xPitch = zYaw * m_gearCameraPitchCorrection.getSin() + xYaw * m_gearCameraPitchCorrection.getCos();
//	double yPitch = yYaw;
//	double zPitch = zYaw * m_gearCameraPitchCorrection.getCos() - xYaw * m_gearCameraPitchCorrection.getSin();
//	printf("XPitch %d\n", xPitch);
//	printf("YPitch %d\n", yPitch);
//	printf("ZPitch %d\n", zPitch);
//
//	if (zPitch < 0) {
//		double scaling = m_gearDifferentialHeight / zPitch;
//		double distance = hypot(xPitch, yPitch) * scaling;
//		Rotation2D robotAngle = Rotation2D(xPitch, yPitch, true);
//		Rotation2D targetAngle = Rotation2D::fromDegrees(m_skewAngleMap.getInterpolated(gearTarget.GetSkew().getDegrees()).m_value);
//		m_gearDistance = distance;
//		m_gearCurrentAngle = robotAngle.getDegrees();
//		m_gearTargetAngle = targetAngle.getDegrees();
////		SmartDashboard::PutNumber("VisionUpdate Distance", m_gearDistance);
////		SmartDashboard::PutNumber("AngleOfRobot", m_gearCurrentAngle);
////		SmartDashboard::PutNumber("AngleToTarget", m_gearTargetAngle);
//
//		RigidTransform2D fieldToGoal = (fieldToCamera.transformBy(RigidTransform2D(
//			Translation2D(distance * robotAngle.getCos(), distance * robotAngle.getSin()),
//				targetAngle)));
//		m_goalLiftTracker.update(timeStamp, fieldToGoal);
//	}
//}
//
//std::list<AimingParameters> RobotChains::getGearAimingParameters(double currentTimeStamp) {
//	std::lock_guard<std::recursive_mutex> lk(m_mutex);
//	std::list<AimingParameters> rv;
////	const std::set<GoalTracker::TrackReport> &reports(m_goalLiftTracker.getTracks());
////
////	RigidTransform2D latestGearFlickerFixedToField = getPredictedFieldToVehicle(Constants::kAutoAimPredictionTime).
////			transformBy(kVehicleToGearFlickerFixed).inverse();
////	//printf("GearFlickerToField %d\n", latestGearFlickerFixedToField.)
////
////	for (GoalTracker::TrackReport report : reports) {
////		if (currentTimeStamp - report.m_latestTimestamp > kMaxTargetAge) {
////			continue;
////		}
////		RigidTransform2D latestGearFlickerFixedToGoal = latestGearFlickerFixedToField.transformBy(report.m_fieldToGoal);
////
////		rv.push_back(AimingParameters(latestGearFlickerFixedToGoal, report.m_id));
////	}
//	rv.push_back(AimingParameters(RigidTransform2D(Translation2D(0, m_gearDistance).rotateBy(Rotation2D::fromDegrees(m_gearCurrentAngle)),
//			Rotation2D::fromDegrees(m_gearTargetAngle)),0));
//	return rv;
//}
//
//std::list<RigidTransform2D> RobotChains::getCaptureTimeFieldToGoal() {
//	std::lock_guard<std::recursive_mutex> lk(m_mutex);
//	std::list<RigidTransform2D> rv;
//
//	for (GoalTracker::TrackReport report : m_goalLiftTracker.getTracks()) {
//		rv.push_back(report.m_fieldToGoal);
//	}
//	return rv;
//}
//
//void RobotChains::resetVision() {
//	std::lock_guard<std::recursive_mutex> lk(m_mutex);
//	m_goalLiftTracker.reset();
//}
//
//RigidTransform2D RobotChains::generateOdometryFromSensors(double frEncoderDeltaDistance, double flEncoderDeltaDistance, double brEncoderDeltaDistance,
//	double blEncoderDeltaDistance, double frRotationDelta, double flRotationDelta, double brRotationDelta, double blRotationDelta, Rotation2D currentGyroAngle) {
//	std::lock_guard<std::recursive_mutex> lk(m_mutex);
//	RigidTransform2D lastMeasurement = getLatestFieldToVehicle();
//	return Kinematics::integrateForwardKinematics(lastMeasurement, frEncoderDeltaDistance, flEncoderDeltaDistance, brEncoderDeltaDistance, blEncoderDeltaDistance,
//		frRotationDelta, flRotationDelta, brRotationDelta, blRotationDelta, currentGyroAngle);
//}
//
//RigidTransform2D RobotChains::getLatestFieldToVehicle() {
//	std::lock_guard<std::recursive_mutex> lk(m_mutex);
//	return m_fieldToVehicle.rbegin()->second;
//}
//
//Rotation2D RobotChains::getLatestTurretRotation() {
//	std::lock_guard<std::recursive_mutex> lk(m_mutex);
//	return m_turretRotation.rbegin()->second;
//}
//
//void RobotChains::setVehicleToTurretFixed() {
//	std::lock_guard<std::recursive_mutex> lk(m_mutex);
//	kVehicleToTurretFixed = RigidTransform2D(Translation2D(Constants::kTurretXOffset, Constants::kTurretYOffset), Rotation2D::fromDegrees(Constants::kTurretAngleOffsetDegrees));
//}
//
//void RobotChains::setTurretRotatingToCamera() {
//	std::lock_guard<std::recursive_mutex> lk(m_mutex);
//	kVehicleToGearCameraFixed = RigidTransform2D(Translation2D(Constants::kGearCameraXOffset, Constants::kGearCameraYOffset), Rotation2D());
//}
//
//GoalTracker RobotChains::getGoalTracker() {
//	std::lock_guard<std::recursive_mutex> lk(m_mutex);
//	return m_goalLiftTracker;
//}
//
//RobotChains* RobotChains::getInstance() {
//	static RobotChains instance;
//	return &instance;
//}
//
//void RobotChains::outputToSmartDashboard() {
//	const RigidTransform2D &odometry = getLatestFieldToVehicle();
//	SmartDashboard::PutNumber("Robot Pose X", odometry.getTranslation().getX());
//	SmartDashboard::PutNumber("Robot Pose Y", odometry.getTranslation().getY());
//	SmartDashboard::PutNumber("Robot Pose Theta", odometry.getRotation().getDegrees());
//	std::list<RigidTransform2D> poses = getCaptureTimeFieldToGoal();
//	for(RigidTransform2D pose : poses){
//		SmartDashboard::PutNumber("Goal Pose X", pose.getTranslation().getX());
//		SmartDashboard::PutNumber("Goal Pose Y", pose.getTranslation().getY());
//		break;
//	}
//}
//
//void RobotChains::addVisionUpdateBoiler(double timeStamp, TargetInfo boilerTarget) {
//	std::lock_guard<std::recursive_mutex> lk(m_mutex);
//		RigidTransform2D fieldToCamera = getFieldToCamera(timeStamp);
//
//		double yDeadBand = (boilerTarget.getY() > -Constants::kCameraDeadBand && boilerTarget.getY() < Constants::kCameraDeadBand) ? 0.0 : boilerTarget.getY();
//
//		double xYaw = boilerTarget.getX() * m_boilerCameraYawCorrection.getCos() + yDeadBand * m_boilerCameraYawCorrection.getSin();
//		double yYaw = yDeadBand * m_boilerCameraYawCorrection.getCos() - boilerTarget.getX() * m_boilerCameraYawCorrection.getSin();
//		double zYaw = boilerTarget.getZ();
//
//		double xPitch = zYaw * m_boilerCameraPitchCorrection.getSin() + xYaw * m_boilerCameraPitchCorrection.getCos();
//		double yPitch = yYaw;
//		double zPitch = zYaw * m_boilerCameraPitchCorrection.getCos() - xYaw * m_boilerCameraPitchCorrection.getSin();
//		printf("XPitch %d\n", xPitch);
//		printf("YPitch %d\n", yPitch);
//		printf("ZPitch %d\n", zPitch);
//
//		if (zPitch < 0) {
//			double scaling = m_boilerDifferentialHeight / zPitch;
//			double distance = hypot(xPitch, yPitch) * scaling;
//			Rotation2D robotAngle = Rotation2D(xPitch, yPitch, true);
//			m_boilerDistance = distance;
//			m_boilerCurrentAngle = robotAngle.getDegrees();
//			SmartDashboard::PutNumber("VisionUpdate Distance", m_boilerDistance);
//			SmartDashboard::PutNumber("AngleOfRobot", m_boilerCurrentAngle);
//
//			RigidTransform2D fieldToGoal = (fieldToCamera.transformBy(RigidTransform2D::fromTranslation(
//				Translation2D(distance * robotAngle.getCos(), distance * robotAngle.getSin()))));
//			m_goalLiftTracker.update(timeStamp, fieldToGoal);
//		}
//}
//
//std::list<AimingParameters> RobotChains::getBoilerAimingParameters(double currentTimeStamp) {
//	std::lock_guard<std::recursive_mutex> lk(m_mutex);
//	std::list<AimingParameters> rv;
//	rv.push_back(AimingParameters(RigidTransform2D::fromTranslation(Translation2D(0, m_boilerDistance).rotateBy(Rotation2D::fromDegrees(m_boilerCurrentAngle))), 0));
//	return rv;
//}
