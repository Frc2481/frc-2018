#include "CommandBase.h"
#include "Commands/Scheduler.h"

// Initialize a single static instance of all of your subsystems to NULL
std::unique_ptr<DriveTrain2017> CommandBase::m_driveTrain;
std::unique_ptr<OI> CommandBase::oi;
bool CommandBase::m_pause;

CommandBase::CommandBase(const std::string &name) :
		Command(name)
{
}

CommandBase::CommandBase() :
		Command()
{

}

void CommandBase::init()
{
	// Create a single static instance of all of your subsystems. The following
	// line should be repeated for each subsystem in the project.
	m_driveTrain.reset(new DriveTrain2017());

	m_pause = false;

	//oi goes last!!!
	oi.reset(new OI());
}
