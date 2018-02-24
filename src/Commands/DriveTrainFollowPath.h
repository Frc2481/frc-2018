/*
 * DriveTrainFollowTestPath.h
 *
 *  Created on: Feb 6, 2018
 *      Author: Team2481
 */

#ifndef SRC_COMMANDS_DRIVETRAINFOLLOWPATH_H_
#define SRC_COMMANDS_DRIVETRAINFOLLOWPATH_H_

#include "DriveTrainDrivePathCommand.h"

class DriveTrainFollowPath : public DriveTrainDrivePathCommand {
public:
	DriveTrainFollowPath(std::string path) : DriveTrainDrivePathCommand() {
		m_path = m_pathManager->GetPath(path);
	}
};

#endif /* SRC_COMMANDS_DRIVETRAINFOLLOWPATH_H_ */
