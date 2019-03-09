#ifndef COMMAND_BASE_H
#define COMMAND_BASE_H

#include "frc/WPILib.h"
#include "Subsystems/DriveTrain.h"
#include <string>
#include "frc/Commands/Command.h"
#include "OI.h"
#include "Subsystems/Intake.h"
#include "Subsystems/LimeLight.h"
#include "Subsystems/Arm.h"
#include "utils/PathManager.h"

/**
 * The base for all commands. All atomic commands should subclass CommandBase.
 * CommandBase stores creates and stores each control system. To access a
 * subsystem elsewhere in your code in your code use CommandBase.examplesubsystem
 */
class CommandBase: public frc::Command
{
public:
	CommandBase(const std::string &name);
	CommandBase();
	static void init();
	// Create a single static instance of all of your subsystems
	static std::unique_ptr<DriveTrain> m_driveTrain;
	static std::unique_ptr<Intake> m_intake;
	static std::unique_ptr<Arm> m_arm;
	static std::unique_ptr<frc::Compressor> m_compressor;
	static std::unique_ptr<OI> oi;
	static std::unique_ptr<LimeLight> m_limeLight;
	static std::unique_ptr<PathManager> m_pathManager;

	static bool m_pause;
};

#endif
