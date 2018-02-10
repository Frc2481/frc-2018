#include "CommandBase.h"
#include "Commands/Scheduler.h"

// Initialize a single static instance of all of your subsystems to NULL
std::unique_ptr<DriveTrain> CommandBase::m_driveTrain;
std::unique_ptr<Intake> CommandBase::m_intake;
std::unique_ptr<OI> CommandBase::oi;
std::unique_ptr<LimeLight> CommandBase::m_limeLight;
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
	m_driveTrain.reset(new DriveTrain());
	m_intake.reset(new Intake());
	m_limeLight.reset(new LimeLight());
	m_pause = false;

	//oi goes last!!!
	oi.reset(new OI());
}
