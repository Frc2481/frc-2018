/*
 * DrivePathGenerator.cpp
 *
 *  Created on: Feb 12, 2018
 *      Author: FIRSTMentor
 */

#include <utils/DrivePathGenerator.h>

DrivePathGenerator::DrivePathGenerator() {
	// TODO Auto-generated constructor stub

}

DrivePathGenerator::~DrivePathGenerator() {
	// TODO Auto-generated destructor stub
}

void DrivePathGenerator::GeneratePath(std::vector<Waypoint> &waypoints,
		double maxSpeed, double maxAccel, double sampleRate) {
	// TODO input error checking
	// waypoints is empty
	// max speed is not positive real number
	// max accel is not positive real number
	// sample rate is not positive real number

	//add start point to path
	std::vector<Translation2D> tempPath;
	std::vector<TempWaypoint> waypoints2;
	tempPath.push_back(waypoints.front().pose.getTranslation());

	TempWaypoint tempWayPnt;
	tempWayPnt.pose = waypoints.front().pose;
	tempWayPnt.tempPathIndex = 1;
	waypoints2.push_back(tempWayPnt);

	double arcHeight;
	double arcRadius;
	double maxDistAway;
	double distToTangent;
	double maxPhi;

	Rotation2D theta;

	printf("got here 1\n");

	//loop through waypoints & round corners
	for(std::vector<Waypoint>::iterator it = waypoints.begin(); it != waypoints.end(); ++it) {
		//get 3 consecutive points
		Translation2D p1(it->pose.getTranslation());
		Translation2D p2((it + 1)->pose.getTranslation());
		Translation2D p3((it + 2)->pose.getTranslation());

		//get vectors between points
		Translation2D v21(p1.translateBy(p2.inverse())); //vector between first & second
		Translation2D v23(p3.translateBy(p2.inverse())); //vector between second & third

		//get max distance
		maxDistAway = (it + 1)->maxDistAway;

		//check if vectors have zero length
		if ((v21.norm() == 0) || (v23.norm() == 0)) {
			theta = Rotation2D::fromRadians(0);
		}
		else {
			double temp = (v21.getX() * v23.getX() + v21.getY() * v23.getY()) /
					(v21.norm() * v23.norm());
			theta = Rotation2D::fromRadians(acos(temp)); //angle between v21 & v23
		}

		//check if points lie on straight line
		if(theta.getRadians() == M_PI) {
			//do not insert rounded corner
			tempPath.push_back(p2);
			tempWayPnt.pose = RigidTransform2D(p2, it->pose.getRotation());
			tempWayPnt.tempPathIndex = tempPath.size();
			waypoints2.push_back(tempWayPnt);
		}
		//check if redundant points
		else if(theta.getRadians() == 0) {
			//skip waypoint
		}
		else {
			//calculate arc between points
			arcRadius = maxDistAway * sin(theta.getRadians() / 2) / (1 - sin(theta.getRadians() / 2)); //arc radius
			arcHeight = arcRadius * (1 - cos(M_PI - theta.getRadians())); //arc height
			maxPhi = 2 * arcRadius * sin(M_PI - theta.getRadians()); //arc chord length
			distToTangent = sqrt((maxDistAway + arcHeight) * (maxDistAway + arcHeight) + (maxPhi / 2) * (maxPhi / 2)); //distance b/t second point & where arc is tangent to straight line

			// limit arc radius if bigger than distance between points and update other parameters
			double limitL = std::min(v21.norm(), v23.norm()) / 2;
			if (distToTangent > limitL) {
				distToTangent = limitL;
				maxPhi = 2 * distToTangent * sin(theta.getRadians() / 2);
				arcRadius = maxPhi * sin(theta.getRadians() / 2) / sin(M_PI - theta.getRadians());
				arcHeight = arcRadius * (1 - cos((M_PI - theta.getRadians()) / 2));
				maxDistAway = (distToTangent * cos(theta.getRadians() / 2)) - arcHeight;
			}

			// calculate points on arc tangent to vectors between points 1, 2, 3
			Translation2D v24(distToTangent * v21.getX() / v21.norm(), distToTangent * v21.getY() / v21.norm());
			Translation2D v25(distToTangent * v23.getX() / v23.norm(), distToTangent * v23.getY() / v23.norm());
			Translation2D p4(p2.translateBy(v24));
			Translation2D p5(p2.translateBy(v25));

			// add point 4 to path
			tempPath.push_back(p4);

			// calculate center point of arc
			Translation2D v26(((v21.getX() /v21.norm() + v23.getX() / v23.norm()) / 2), (v21.getY() /v21.norm() + v23.getY() / v23.norm()) / 2);
			Translation2D p6(p2.getX() + (maxDistAway + arcRadius) * v26.getX() / v26.norm(), p2.getY() + (maxDistAway + arcRadius) * v26.getY() / v26.norm()); // center point of arc
			double crossV21V23 = v21.getX() * v23.getY() - v21.getY() * v23.getX();
			double signCrossV21V23 = 0;
			if(crossV21V23 > 0) {
				signCrossV21V23 = 1;
			}
			else if(crossV21V23 < 0) {
				signCrossV21V23 = -1;
			}

			// generate points along arc
			double A = signCrossV21V23 * (M_PI - theta.getRadians()) / 2;
			for(double phi = A; phi <= -A; phi = phi - 2 * A / 21) {
				Translation2D p7;
				p7.setX(p6.getX() - arcRadius * (cos(phi) * v26.getX() - sin(phi) * v26.getY()) / v26.norm());
				p7.setY(p6.getY() - arcRadius * (sin(phi) * v26.getX() + cos(phi) * v26.getY()) / v26.norm());

				//add p7 to path
				tempPath.push_back(p7);
				if(phi == 0) {
					tempWayPnt.pose = RigidTransform2D(p7, it->pose.getRotation());
					tempWayPnt.tempPathIndex = tempPath.size();
					waypoints2.push_back(tempWayPnt);
				}
				printf("got here 4\n");
			}

			// add p5 to path
			tempPath.push_back(p5);
			tempWayPnt.pose = RigidTransform2D(p5, it->pose.getRotation());
			tempWayPnt.tempPathIndex = tempPath.size();
			waypoints2.push_back(tempWayPnt);
		}
	}
	printf("got here 2\n");

	// add end point to path
	tempPath.push_back(waypoints.back().pose.getTranslation());
	tempWayPnt.pose = waypoints.back().pose;
	tempWayPnt.tempPathIndex = tempPath.size();
	waypoints2.push_back(tempWayPnt);

	// calculate total path length and add to interpolating map
	InterpolatingMap<InterpolatingDouble, Translation2D> interpPath;
	waypoints2.begin()->distTraveled = 0;
	for(std::vector<TempWaypoint>::iterator it2 = waypoints2.begin() + 1; it2 != waypoints2.end(); ++it2) {
		it2->distTraveled = (it2 + 1)->distTraveled + it2->pose.getTranslation().translateBy((it2 + 1)->pose.getTranslation().inverse()).norm();
		interpPath.put(it2->distTraveled, it2->pose.getTranslation());
	}

	double totalPathLength = waypoints2.back().distTraveled;
	printf("got here 3\n");

	// calculate acceleration time and distance
	double accelTime = maxSpeed / maxAccel;
	double accelDist = 0.5 * maxAccel * pow(accelTime, 2);

	// recalculate max speed if total path length is too small for max acceleration
	if (2 * accelDist > totalPathLength) {
		accelTime = sqrt(totalPathLength / 2 * 2 / maxAccel);
		accelDist = 0.5 * maxAccel * pow(accelTime, 2);
		maxSpeed = accelTime * maxAccel;
	}

	//calculate times during path
	double totalTime = 2 * accelTime + (totalPathLength - 2 * accelDist) / maxSpeed;
	double startMaxSpeedTime = accelTime;
	double stopMaxSpeedTime = totalTime - accelTime;

	std::vector<double> time;
	std::vector<double> dist;
	std::vector<double> speed;
	time.push_back(0);
	dist.push_back(0);
	speed.push_back(0);
	printf("got here 5\n");

	//calculate speed with respect to time
	for(int i = 2; i <= (floor(totalTime * sampleRate) / sampleRate); i++) {
		time.push_back(time.back() + 1 / sampleRate);

		if(time.back() < startMaxSpeedTime) {
			speed.push_back(maxAccel * time.back());
		}
		else if(time.back() > stopMaxSpeedTime) {
			speed.push_back(maxAccel * (totalTime - time.back()));
		}
		else {
			speed.push_back(maxSpeed);
		}
	}
	printf("got here 6\n");

	//calculate distance traveled with respect to time
	for(int i = 2; i <= (floor(totalTime * sampleRate) / sampleRate); i++) {
		printf("got here cnt: %d\n", i);
		if((startMaxSpeedTime < time.at(i)) && (time.at(i) < (startMaxSpeedTime + 1 / sampleRate))) {
			printf("if\n");
			dist.push_back(dist.back() + (time.at(i) - startMaxSpeedTime) * maxSpeed + (startMaxSpeedTime - time.at(i - 1)) * speed.at(i - 1));
		}
		else if((stopMaxSpeedTime < time.at(i)) && (time.at(i) < (stopMaxSpeedTime + 1 / sampleRate))) {
			dist.push_back(dist.back() + (stopMaxSpeedTime - time.at(i - 1)) * maxSpeed + (time.at(i) - stopMaxSpeedTime) * speed.at(i));
			printf("else if\n");
		}
		else {
			dist.push_back(dist.back() + (speed.at(i) + speed.at(i - 1)) / (2 * sampleRate));
			printf("else\n");
		}
	}
	printf("got here 7\n");

	// calculate distance traveled during last point
	dist.push_back(totalPathLength);
	speed.push_back(0);
	time.push_back(totalTime);

	// TODO maybe limit distance to path length
	printf("got here 8\n");

	// interpolate distance traveled to get final path
	std::vector<FinalPath> finalPath;
	for(std::vector<double>::iterator it3 = dist.begin(); it3 != dist.end(); ++it3) {
		FinalPath finalPathPoint;
		finalPathPoint.pose.setTranslation(interpPath.getInterpolated(*it3));
		finalPathPoint.time = time.at(it3 - dist.begin());
		finalPath.push_back(finalPathPoint);
	}
	printf("got here 9\n");

	// get distance traveled corresponding to waypoints
	for(std::vector<TempWaypoint>::iterator it4 = waypoints2.begin(); it4 != waypoints2.end(); ++it4) {
		it4->distTraveled = dist.at(it4->tempPathIndex);
	}

	// get final path index corresponding to waypoints
	waypoints2.front().finalPathIndex = 1;
	int j = 2;
	for(std::vector<FinalPath>::iterator it5 = finalPath.begin() + 1; it5 != finalPath.end() - 1; ++it5) {
		if(dist.at(it5 - finalPath.begin()) >= waypoints2.at(j).distTraveled) {
			waypoints2.at(j).finalPathIndex = it5 - finalPath.begin();
			j++;
		}
	}
	waypoints2.back().finalPathIndex = finalPath.size();

	// get yaw rate corresponding to final path
	double deltaYaw;
	std::vector<double> yawRate;
	for(std::vector<TempWaypoint>::iterator it6 = waypoints2.begin() + 1; it6 != waypoints2.end(); ++it6) {
		deltaYaw = it6->pose.getRotation().getDegrees();

		// angle wraparound
		if (deltaYaw > 180) {
			deltaYaw = 360 - deltaYaw;
		} else if (deltaYaw < -180) {
			deltaYaw = 360 + deltaYaw;
		}

		int deltaIdx = it6->finalPathIndex - (it6 - 1)->finalPathIndex;
		yawRate.push_back(deltaYaw / (double)deltaIdx);
	}

	// add yaw to final path
	finalPath.front().pose.setRotation(waypoints2.front().pose.getRotation());
	j = 1;
	double newYaw;
	for(std::vector<FinalPath>::iterator it7 = finalPath.begin() + 1; it7 != finalPath.end(); ++it7) {
		if((it7 - finalPath.begin()) > waypoints2.at(j).finalPathIndex) {
			j++;
		}

	    newYaw = (it7 - 1)->pose.getRotation().getDegrees() + yawRate.at(j);
		it7->pose.setRotation(Rotation2D::fromDegrees(newYaw));
	}

	// write final path to .csv file
	std::ofstream myFile;
	myFile.open("/home/lvuser/robotPath.csv");
	for(std::vector<FinalPath>::iterator it8 = finalPath.begin(); it8 != finalPath.end(); ++it8) {
		myFile << it8->pose.getTranslation().getX() << ",";
		myFile << it8->pose.getTranslation().getY() << ",";
		myFile << it8->pose.getRotation().getDegrees() << ",";
		myFile << it8->time << "\n";
	}
	myFile.close();
}
