/**********************************************************************
 *
 * This code is part of the MRcore project
 * Author:  Rodrigo Azofra Barrio & Miguel Hernando Gutierrez
 * 
 *
 * MRcore is licenced under the Common Creative License,
 * Attribution-NonCommercial-ShareAlike 3.0
 *
 * You are free:
 *   - to Share - to copy, distribute and transmit the work
 *   - to Remix - to adapt the work
 *
 * Under the following conditions:
 *   - Attribution. You must attribute the work in the manner specified
 *     by the author or licensor (but not in any way that suggests that
 *     they endorse you or your use of the work).
 *   - Noncommercial. You may not use this work for commercial purposes.
 *   - Share Alike. If you alter, transform, or build upon this work,
 *     you may distribute the resulting work only under the same or
 *     similar license to this one.
 *
 * Any of the above conditions can be waived if you get permission
 * from the copyright holder.  Nothing in this license impairs or
 * restricts the author's moral rights.
 *
 * It is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  
 **********************************************************************/

#ifndef __MRCORE__ACTUATOR__H
#define __MRCORE__ACTUATOR__H


#include "simplejoint.h"
#include "math/mrmath.h"
#include <vector>

using namespace std;

namespace mr
{
	class SimpleJoint;
	
	//types of movements
	enum InterpolatorType 
	{
		CPT,
		TVP,
		SPLINE,
	};


/**
    \class Actuator
    Actuator	->	Implementation to simulate simplejoints movement
*/

class Actuator: public PositionableEntity
{
	DECLARE_MR_OBJECT(Actuator)
	//friend class World;

public:
	
//constructors

//Basic Constructor
	Actuator(double _speed=1,double _maxSpeed=1,double _acceleration=0.1, double _maxAcceleration=0.5);
	//Destructor
	virtual ~Actuator(void);


//general simulation methods
	void setSimulationParameters(double _maxSpeed, double _maxAcceleration=PI/6);//max acelerate default=30�/sec^2

	double setSpeed(double sp);
	double setAcceleration(double ac);
	
	double getSpeed(){return speed;}
	double getMaxSpeed(){return maxSpeed;}
	double getAcceleration(){return acceleration;}
	double getMaxAcceleration(){return maxAcceleration;}
	
	bool isMoving(){return targetActive;}
	void linkTo (PositionableEntity *p);

	virtual void simulate(double delta_t);

	bool setTarget(double val);
	double getTarget(){return target;}

	virtual void setFrequency (float _freq){frequency = _freq;}
	virtual float getFrequency () {return frequency;}

	vector<double> getCoeficientsPolinomial(){
		vector<double> coef; 
		coef.push_back(a0);
		coef.push_back(a1);
		coef.push_back(a2);
		coef.push_back(a3);
		return coef;
	}
        
//selection movement
	void setInterpolatorType (InterpolatorType traj=CPT){moveType=traj;}
	InterpolatorType getInterpolatorType (){return moveType;}

//specific methods trapezoidal Velocity Profile Trajectory (TVP)
	bool setInterpolatorTypeTVP(string _type);
	string getInterpolatorTypeTVP(){return InterpolatorTypeTVP;}
	void simulateInterpolatorTVP(double qInit,double q_target,int signMovement,double _time, 
									double targetTime, double TVP_acceleration_time);

//specific methods spline (interpolation points) and cubical trajetory 
	void setCubicPolinomialCoeficients(double path,double targetTime);
	void setVelocIntermediates (vector<double> veloc);

//Attributes
protected:
	SimpleJoint* s_Joint;

	//kinematic simulation atributes
	double speed, maxSpeed; // m/s rad/s
	double acceleration, div_target, maxAcceleration; //rad/sec^2
	
	double target, targetIntermediate;
	bool targetActive; //true if target have to be reached
	string InterpolatorTypeTVP;
	//spline algorithm
	double a0,a1,a2,a3; //polinomial coeficients

	int index;//index of velocities intermediates

	vector<double> velocInter;
	
	InterpolatorType moveType;
	float frequency; //Hz

};

};
#endif  //__MRCORE__ACTUATOR__H
