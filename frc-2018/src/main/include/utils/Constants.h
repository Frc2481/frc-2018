#pragma once

class Constants {
public:
	static constexpr double kMaxGoalTrackAge = 0.3;
	static constexpr double kMaxTrackerDistance = 18.0;
	static constexpr double kCameraFrameRate = 30.0;
	static constexpr double kStabilityWeight = 0.5;
	static constexpr double kAgeWeight = 1.0;
	static constexpr double kSwitchingWeight = 3.0;
	static constexpr double kTrackScrubFactor = 0.5;
	static constexpr double kTrackLengthInches = 8.265;
	static constexpr double kTrackWidthInches = 23.8;
	static constexpr double kTrackEffectiveDiameter = (kTrackWidthInches * kTrackWidthInches + kTrackLengthInches * kTrackLengthInches) / kTrackWidthInches;
	static constexpr double kGearCameraPitchAngleDegrees = -21; //Change back to -20 for real
	static constexpr double kGearCameraYawAngleDegrees = 0.0;
	static constexpr double kBoilerCameraPitchAngleDegrees = 20.911; //Change back to -20 for real
	static constexpr double kBoilerCameraYawAngleDegrees = 0.0;
	static constexpr double kGearCenterOfTargetHeight = 13;
	static constexpr double kBoilerCenterOfTargetHeight = 87.5; //change
	static constexpr double kGearCameraZOffset = 19;
	static constexpr double kBoilerCameraZOffset = 26; //change
	static constexpr double kGearCameraXOffset = 13;
	static constexpr double kBoilerCameraXOffset = 0.0; //change
	static constexpr double kGearCameraYOffset = 0.0; //Maybe Wrong
	static constexpr double kBoilerCameraYOffset = 0.0; //Maybe Wrong
	static constexpr double kTurretXOffset = -7.376; //Probably Wrong
	static constexpr double kTurretYOffset = 0.0; //Maybe Wrong
	static constexpr double kTurretAngleOffsetDegrees = 0.0;
	static constexpr double kAutoAimPredictionTime = 0.25;
	static constexpr double kCameraDeadBand = 0.0;
	static constexpr double kGearYPosThreshold = 20.0; //tune
	static constexpr double kGearCameraWidth = 3000; //tune
	static constexpr double kBoilerYPosThreshold = 20.0; //tune
	static constexpr double kBoilerZPosThreshold = 100.0; //tune
};
	
