/*
 * PathManager.h
 *
 *  Created on: Feb 23, 2018
 *      Author: FIRSTMentor
 */

#ifndef SRC_PATHMANAGER_H_
#define SRC_PATHMANAGER_H_

#include <map>
#include <string>
#include "utils/PathLoader.h"

class PathManager {
private:
	std::map<std::string, Path2D> m_paths;
	bool m_missingPath;
public:
	PathManager();
	virtual ~PathManager();
	Path2D& GetPath(std::string);
	bool HasMissingPath();
	void ReloadPaths();
};

#endif /* SRC_PATHMANAGER_H_ */
