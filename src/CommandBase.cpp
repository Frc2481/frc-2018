#include "CommandBase.h"
#include "Commands/Scheduler.h"

// Initialize a single static instance of all of your subsystems to NULL
std::unique_ptr<DriveTrain> CommandBase::m_driveTrain;
std::unique_ptr<Intake> CommandBase::m_intake;
std::unique_ptr<Arm> CommandBase::m_arm;
std::unique_ptr<Compressor> CommandBase::m_compressor;
std::unique_ptr<OI> CommandBase::oi;
std::unique_ptr<LimeLight> CommandBase::m_limeLight;
std::unique_ptr<PathManager> CommandBase::m_pathManager;
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
	m_pathManager.reset(new PathManager());
	m_pause = false;
	m_arm.reset(new Arm());
	m_compressor.reset(new Compressor());
	m_compressor->Start();
	//oi goes last!!!
	oi.reset(new OI());
}
