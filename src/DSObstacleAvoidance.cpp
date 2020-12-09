/**
 * 
 *  Developer:  Iason Batzianoulis
 *  email:      iasonbatz@gmail.com
 *  
 *  
*/

#include "DSObstacleAvoidance.h"

DSObstacleAvoidance::DSObstacleAvoidance()
{
	std::cout << "Obstacle modulator generated: waiting for obstacle parameters" << std::endl;
}

void DSObstacleAvoidance::setObstacle(Obstacle &obs)
{
	_obs = obs;
	_obs._a = _obs._a*_obs._safetyFactor;
	// ROS_INFO_STREAM("Obstacle parameters recieved.");
}

void DSObstacleAvoidance::addObstacle(Obstacle &obs){
	Obstacle t_obs = obs;
	t_obs._a = t_obs._a*t_obs._safetyFactor;
	_obstacles.push_back(t_obs);
	_gammas.push_back(0.0);
	_basisMatrixes.push_back(Eigen::Matrix3f());
}

void DSObstacleAvoidance::addObstacles(std::vector<Obstacle> obstacles){
	for (size_t i=0; i<obstacles.size(); i++){
		addObstacle(obstacles[i]);
	}
}

void DSObstacleAvoidance::clearObstacles(){
	_obstacles.clear();
	_gammas.clear();
	_basisMatrixes.clear();
}

Eigen::Vector3f DSObstacleAvoidance::obsModulationEllipsoid(Eigen::Vector3f x, Eigen::Vector3f xd, bool bContour)
{
	_modulationMatrix.setIdentity();

	_rotationMatrix.setIdentity();

	
	for (int i = 0; i<(int)_obstacles.size(); i++){
		Eigen::Vector3f x_t =  _rotationMatrix*(x - _obstacles[i]._x0);
		computeBasisMatrix(x_t, i);
	}

	Eigen::Vector3f eig_vals, d0;
	for (int i = 0; i<(int)_obstacles.size(); ++i){
		d0.setConstant(1.0f);
		d0(0) = -1.0f;
		eig_vals = d0/(pow(_gammas[i],1.0f/_obstacles[i]._rho));
		if((!_obstacles[i]._tailEffect) and xd.dot(_rotationMatrix*_basisMatrixes[i].col(0))>=0){
			eig_vals(0) = 0.0;
		}
		
		if(eig_vals(0)<-1.0)
		{
			eig_vals.tail(2).setConstant(1.0f);
			if(xd.dot(_rotationMatrix*_basisMatrixes[i].col(0))<0){
				eig_vals(0) = -1.0f;
			}
		}

		eig_vals = eig_vals.array() + 1;
		_modulationMatrix = _rotationMatrix*_basisMatrixes[i]*(eig_vals.asDiagonal())*(_basisMatrixes[i].inverse())*_rotationMatrix.transpose()*_modulationMatrix;
	}

	if((!bContour) and eig_vals(0)<-0.98 and xd.dot(_basisMatrixes.back().col(0))<0 and (_modulationMatrix*xd).norm()<0.02)
		bContour = true;

	if(bContour)
	{
		Eigen::Vector3f contour_dir;
		contour_dir = _basisMatrixes.back().col(1);
		contour_dir.normalize();

		if(xd.dot(_basisMatrixes.back().col(0))>0)
		{
			bContour = false;
			_modulatedVel = _modulationMatrix*xd;
		}
		else
			_modulatedVel = contour_dir*(xd.norm());
	}
	else
		_modulatedVel = _modulationMatrix*xd;
	

	return _modulatedVel;
}

void DSObstacleAvoidance::computeBasisMatrix(Eigen::Vector3f x)
{
	Eigen::Vector3f nv;
	_gamma = 0.0f;
	for (int i = 0; i < 3; ++i)
	{
		_gamma += pow(x[i]/_obs._a[i],2*_obs._p[i]);
		nv[i] = pow(2*(_obs._p[i]/_obs._a[i])*(x[i]/_obs._a[i]),2*_obs._p[i]-1);
	}
	_basisMatrix.setConstant(0.0f);

	_basisMatrix.col(0) = nv;
	_basisMatrix.block(0,1,1,2) = nv.tail(2).transpose();
	_basisMatrix.block(1,1,2,2).setIdentity();
	_basisMatrix.block(1,1,2,2) *= -nv(0)+1e-5;
}


void DSObstacleAvoidance::computeBasisMatrix(Eigen::Vector3f x, int nbObj)
{
	Eigen::Vector3f nv;
	_gammas[nbObj] = 0.0f;
	
	for (int i = 0; i < 3; i++)
	{
		_gammas[nbObj] += pow(x[i]/_obstacles[nbObj]._a[i],2*_obstacles[nbObj]._p[i]);
		nv[i] = pow(2*(_obstacles[nbObj]._p[i]/_obstacles[nbObj]._a[i])*(x[i]/_obstacles[nbObj]._a[i]),2*_obstacles[nbObj]._p[i]-1);
	}

	_basisMatrixes[nbObj].setConstant(0.0f);
	_basisMatrixes[nbObj].col(0) = nv;
	_basisMatrixes[nbObj].block(0,1,1,2) = nv.tail(2).transpose();
	_basisMatrixes[nbObj].block(1,1,2,2).setIdentity();
	_basisMatrixes[nbObj].block(1,1,2,2) *= -nv(0)+1e-5;

	
}