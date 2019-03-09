/*
 * PathManager.cpp
 *
 *  Created on: Feb 23, 2018
 *      Author: FIRSTMentor
 */

#include "utils/PathManager.h"

PathManager::PathManager() {
	// TODO Auto-generated constructor stub
	m_missingPath = false;

}

PathManager::~PathManager() {
	// TODO Auto-generated destructor stub
}

Path2D& PathManager::GetPath(std::string pathName) {
	auto result = m_paths.find(pathName);
	if(result == m_paths.end()) {
		Path2D& path = m_paths[pathName];
		PathLoader pathLoader(path);
		pathLoader.LoadPath(pathName);

		if(path.empty()) {
			m_missingPath = true;
			printf("missing path %s\n", pathName.c_str());
		}
	}
	return m_paths[pathName];
}

bool PathManager::HasMissingPath() {
	return m_missingPath;
}

void PathManager::ReloadPaths() {
	m_missingPath = false;
	for(auto pathPair : m_paths) {
		Path2D& path = pathPair.second;
		path.clear();
		PathLoader pathLoader(path);
		pathLoader.LoadPath(pathPair.first);

		if(path.empty() && !pathPair.first.empty()) {
			m_missingPath = true;
			printf("missing path %s\n", pathPair.first.c_str());
		}
	}
}
