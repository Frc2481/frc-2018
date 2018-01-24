#ifndef COMMAND_BASE_H
#define COMMAND_BASE_H

#include <Subsystems/DriveTrain.h>
#include <string>
#include "Commands/Command.h"
#include "OI.h"
#include "WPILib.h"
#include "Subsystems/Intake.h"

/**
 * The base for all commands. All atomic commands should subclass CommandBase.
 * CommandBase stores creates and stores each control system. To access a
 * subsystem elsewhere in your code in your code use CommandBase.examplesubsystem
 */
class CommandBase: public Command
{
public:
	CommandBase(const std::string &name);
	CommandBase();
	static void init();
	// Create a single static instance of all of your subsystems
	static std::unique_ptr<DriveTrain> m_driveTrain;
	static std::unique_ptr<Intake> m_intake;
	static std::unique_ptr<OI> oi;

	static bool m_pause;
};

#endif
