/**
 * 
 *  Developer:  Iason Batzianoulis
 *  email:      iasonbatz@gmail.com
 *  
 *  
*/

#ifndef __DS_OBSTACLE_AVOIDANCE_H__
#define __DS_OBSTACLE_AVOIDANCE_H__

#include <vector>
#include <iostream>
#include "Eigen/Eigen"


struct Obstacle
{
    Eigen::Vector3f _a, _p, _x0;
    double _safetyFactor, _rho, _thR;
    bool _tailEffect, _bContour;
};

class DSObstacleAvoidance
{
	private:

		Obstacle _obs;
		std::vector<Obstacle> _obstacles;
		Eigen::Vector3f _modulatedVel;
    	Eigen::Matrix3f _modulationMatrix, _rotationMatrix, _basisMatrix;
		std::vector<Eigen::Matrix3f> _basisMatrixes;
    	double _gamma;
		std::vector<double> _gammas;

	public:

		DSObstacleAvoidance();

		void setObstacle(Obstacle &obs);

		void addObstacle(Obstacle &obs);

		void addObstacles(std::vector<Obstacle> obstacles);

		void clearObstacles();

		Eigen::Vector3f obsModulationEllipsoid(Eigen::Vector3f x, Eigen::Vector3f xd, bool bContour);

	private:

		void computeBasisMatrix(Eigen::Vector3f x);
		void computeBasisMatrix(Eigen::Vector3f x, int nbObj);

};

#endif