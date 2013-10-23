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

#include "robotsim.h"
#include "../world/world.h"
#include "../world/referencesystem.h"
#include <iostream>

namespace mr
{
bool RobotSim::checkRobotColision()
{
//checks the collision of robot and the rest of the objects
	bool prev=isIntersectable();
	this->setIntersectable(false);
	World *w=getWorld();
	int i;
	for(i=0;i<(int)(objects.size());i++)
	{	
		SolidEntity *aux=dynamic_cast<SolidEntity *>(objects[i]);
		//en cuanto hay colision devuelvo true
		if(aux){
			if(aux!=this->links[0])
			if(w->checkCollisionWith(*aux))return true;
		}
	}
//checks the collision of grasped objects with the world
	if(tcp!=0){
		ReferenceSystem *tcpr=tcp->getReferenciableLocation();
		int n=tcpr->getNumberOfDependents();

		for(int j=0;j<n;j++)
		{
			ReferenceSystem* raux=tcpr->getDependent(j);
			SolidEntity *aux=dynamic_cast<SolidEntity *>(raux->getOwner());
			if(aux){
				if(w->checkCollisionWith(*aux))return true;
			}
		}

	}

//checks the collision of robot links between them
	setIntersectable(prev);
return false;
}
bool RobotSim::checkJointValues(const vector<double> & _q) const
{
//check dimension
	if(_q.size()!=joints.size())return false;
	for(int i=0;i<(int)joints.size();i++){
		if(joints[i]->checkValue(_q[i])==false)return false;
	}
	return true;
}
bool RobotSim::forwardKinematicsAbs(vector<double> _q, Transformation3D& t)
{
	int i;
	if(tcp==0)return false;
	if(links.size()==0)return false;

	if(!checkJointValues(_q))return false; //limits checking
	
	vector<double> qold;
	//save current joint values and update the joint values
	for(i=0;i<(int)joints.size();i++){
		qold.push_back(joints[i]->getValue());
		joints[i]->setValue(_q[i]);
	}

	// Compute the relative transformation between the tcp and the robot base
	t=tcp->getAbsoluteT3D();

	// Move the robot to the qold values
	for(i=0;i<(int)joints.size();i++)joints[i]->setValue(qold[i]);
	return true;
}
bool RobotSim::inverseKinematicsAbs(Transformation3D t, vector<double> &_q, unsigned char conf)
{
	Transformation3D t_base=links[0]->getAbsoluteT3D();
	Transformation3D aux=t_base.inverted();
	Transformation3D t_p=(aux)*(t);
	return inverseKinematics(t_p,_q,conf);
	
}
bool  RobotSim::forwardKinematics(const vector<double> & _q, Transformation3D& t)//conf puntero salida
{

	// Compute the relative transformation between the tcp and the robot base
	Transformation3D _t;
	forwardKinematicsAbs(_q,_t);
	Transformation3D aux=links[0]->getAbsoluteT3D();
	t=(aux.inverted())*_t;

	return true;
}
unsigned char RobotSim::getCurrentConfiguration()
{
	vector<double> q;
	unsigned char conf;
	//save current joint values and update the joint values
	for(int i=0;i<(int)joints.size();i++)q.push_back(joints[i]->getValue());
	getConfigurationOf(q,conf);
	return conf;
	
}

unsigned char RobotSim::getCurrentConfiguration(vector<double> _q) //used by linear path
{
	unsigned char conf;
	//we already know joints' values to next target
	getConfigurationOf(_q,conf);
	return conf;
}

bool  RobotSim::moveToAbs(Transformation3D t3d, unsigned char conf)
{

	Transformation3D t_base=links[0]->getAbsoluteT3D();
	Transformation3D aux=t_base.inverted();
	Transformation3D t_p=(aux)*(t3d);

	return moveTo(t_p,conf);
}
Transformation3D RobotSim::getTcpAbsLocation()
{
	return tcp->getAbsoluteT3D();
}
Transformation3D RobotSim::getTcpLocation()
{
	Transformation3D _t=tcp->getAbsoluteT3D();
	Transformation3D aux=links[0]->getAbsoluteT3D();
	return (aux.inverted())*_t;
}


void RobotSim::simulate(double delta_t)
{

	if((int)q_target.size() == (int)joints.size())
	{
		double error=0;
		for(int i=0;i<(int)joints.size();i++)
			error+=fabs(q_target[i]-joints[i]->getValue());//*

		if(time >= (targetTime*0.85)) //if reach the 85% target time, we change to via point movement
		{
			if (all_space_points.size())
				
				

		}
		
		if( (time+delta_t) >= targetTime)//*
		{
			for(int i=0;i<(int)joints.size();i++)
				actuators[i]->setTarget(q_target[i]);

			q_target.clear();
			time=0.0;
			targetTime=0.0;
			
			check_init_pos=true; // TVP
			joint_initValue.clear(); // TVP
			
			changed_target=false; //end of vía point

			if (path_type==LINEAR)
			{
				if ((index_pos+1) != (int)all_joints_value.size())
				{
					index_pos++;
					updateTargetAndTagetTime(index_pos);
				}
				else
				{
					all_joints_value.clear();
					all_space_points.clear();
					return;
				}
			}				
		}
		else
		{
			if (changed_target)
			//We activate vía point
			{
				/* Path to via point
				   T = Tb - Ta
				*/

			}
			/*************************************************************
				IF TRAJECTORY SELECTED IS CUBIC POLINOMIAL TRAJECTORY TYPE 
			**************************************************************/
			if (trajectory_type==CPT)
			{
				double val=0,sp=0;
				for(int i=0;i<(int)actuators.size();i++)
				{
					coef=actuators[i]->getCoeficientsPolinomial();
					
					val=coef[0] + coef[1]*time + coef[2]*square(time) + coef[3]*square(time)*time;
					sp=coef[1]+ 2*coef[2]*time + 3*coef[3]*square(time);
					
					actuators[i]->setSpeed(sp);
					actuators[i]->setTarget(val);
				}

			}
			/**************************************************************
				IF TRAJECTORY SELECTED IS TRAPEZOIDAL VELOCITY PROFILE TYPE 
			 **************************************************************/
			else if (trajectory_type==TVP)
			{
				double timeInit=0,timeFinal=0,val=0;
				//double target;
				timeInit=0.0;
				timeFinal= targetTime;
				int signMovement=1;

				if(q_target.size()!=actuators.size())return;
				if(joints.size()!=actuators.size())return;

				if (check_init_pos) //we need the initial values of the joints
				{
					for (int i=0;i<actuators.size();i++)
					{
						joint_initValue.push_back(joints[i]->getValue());

					}
					check_init_pos=false;
				}

				for (int i=0;i<actuators.size();i++)
				{
					if ((q_target[i]-joint_initValue[i])<0)
						signMovement=-1;//if we go to target in the negative cuadrant
					else 
						signMovement=1;


					if(actuators[i]->getTrajectoryTypeTVP()=="MaximumSpeedAcceleration")
					{
						//Acceleration phase
						if (time<(timeInit+ta) && time>=timeInit)
							val=joint_initValue[i]+signMovement*((actuators[i]->getAcceleration()*0.5)*square(time-timeInit));

						//Velocity constant phase
						if (time>=(timeInit+ta) && time<=(timeFinal-ta))
							val=joint_initValue[i]+signMovement*(actuators[i]->getSpeed()*(time-timeInit-ta*0.5));

						//Deceleration phase
						if (time>(timeFinal-ta) && time<=timeFinal)
							val=q_target[i]-signMovement*((actuators[i]->getAcceleration()*0.5)*square(timeFinal-time));
					}
					else if(actuators[i]->getTrajectoryTypeTVP()=="BangBang")
					{
						//Acceleration phase
						if (time<(timeFinal*0.5) && time>=timeInit)
							val=joint_initValue[i]+signMovement*((actuators[i]->getMaxAcceleration()*0.5)*square(time-timeInit));

						//Deceleration phase
						if (time>=(timeFinal*0.5) && time<=timeFinal)
							val=q_target[i]-signMovement*((actuators[i]->getMaxAcceleration()*0.5)*square(timeFinal-time));
					}

					actuators[i]->setTarget(val);
				}

			}

			/*************************************************************
				IF TRAJECTORY SELECTED IS SPLINE TRAJECTORY TYPE 
			**************************************************************/
			else if (trajectory_type==SPLINE)
			{
				double strech=0.1,Tk=0.1,val=0;
				for(int i=0;i<(int)actuators.size();i++)
				{
					coef=actuators[i]->getCoeficientsPolinomial();

					actuators[i]->setCubicPolinomialCoeficients(strech,Tk);
					val=val=coef[0] + coef[1]*time + coef[2]*square(time) + coef[3]*square(time)*time;
					actuators[i]->setTarget(val);
				}
			}
			/*******************************************
				BAD TRAJECTORY => TYPE==ERROR
			********************************************/
			else
				return;


			time+=delta_t;
		}
	}

	ComposedEntity::simulate(delta_t);
}



void RobotSim::goTo(vector<double> q)
{
	q_target=q;//Target loaded

	double error=0;
	for(int i=0;i<(int)joints.size();i++)
		error+=fabs(q_target[i] - joints[i]->getValue());//*

	if(fabs(error) < EPS)
	{
		q_target.clear();
		return;
	}
	else
	{
		time=0.0;
		calculateTargetTime();
	}

	return;
}



void RobotSim::updateTargetAndTagetTime(int index)
{ 
	q_target = all_joints_value[index];
	calculateTargetTime();
}

void RobotSim::calculateTargetTime()
{
	/*
		Following options are depending on type of trajectory and each one 
		calculates the target time for the actuators which control the movement.

	*/

	/*************************************************************
		IF TRAJECTORY SELECTED IS CUBIC POLINOMIAL  TYPE 
	**************************************************************/
	if (trajectory_type==CPT)
	{
		/*
			Here we are calculating the max time which one of the 
			actuators last in reaching the target
		*/

		double lowestSpeed=100.0;
		double longestPath=0.0;
		vector<double> pathJoint;

		for(int i=0;i<(int)actuators.size();i++)
		{
			pathJoint.push_back(q_target[i] - joints[i]->getValue());//Path each coordinate: target minus current coordinates

			if(actuators[i]->getMaxSpeed() <= lowestSpeed)
				lowestSpeed = actuators[i]->getMaxSpeed();
			if(fabs(pathJoint[i])>= longestPath)
				longestPath =fabs(pathJoint[i]);//*
		}
		targetTime = longestPath / lowestSpeed;

		for(int i=0;i<(int)actuators.size();i++)
		{
			actuators[i]->setCubicPolinomialCoeficients(pathJoint[i],targetTime);
		}

	}

	/******************************************************************
		IF TRAJECTORY SELECTED IS TRAPEZOIDAL VELOCITY PROFILE TYPE 
	*******************************************************************/
	else if (trajectory_type==TVP)
	{
		/* Check which is the maximum time to get the target by all the joints.
		For this we obtain the max values of the speed and acceleration of the joints and check
		with their own path to travel  */

		if (joints.size()!=actuators.size())return;
		if (q_target.size()!=joints.size())return;

		check_init_pos=true;

		double auxTargetTime=0.0,auxTa=0.0;
		double maxTa=0.0,maxTargetTime=0.0;
		double maxSpeed=0.0,maxAcceleration=0.0;
		double condition=0.0;
		vector<double> path;
		int index=0;

		for(int i=0;i<(int)actuators.size();i++)
		{	
			maxSpeed=actuators[i]->getMaxSpeed();
			maxAcceleration=actuators[i]->getMaxAcceleration();

			condition=(maxSpeed*maxSpeed)/maxAcceleration;

			path.push_back(fabs(joints[i]->getValue()-q_target[i]));

			if (condition>path[i])//Bang Bang movement
			{
				auxTargetTime=2*sqrt(path[i]/maxAcceleration);//targetTime
				auxTa=0.5*auxTargetTime;//ta
			}
			else // speed and acceleration with maximus values
			{		
				auxTargetTime=path[i]/maxSpeed + maxSpeed/maxAcceleration;//targetTime
				auxTa=maxSpeed/maxAcceleration;//ta
			}

			if (maxTargetTime<auxTargetTime)//choose the max values
			{
				index=i;
				maxTargetTime=auxTargetTime;
				maxTa=auxTa;
			}

		}

		targetTime=maxTargetTime;
		ta=maxTa;

		if(maxTa==(0.5*maxTargetTime))
			actuators[index]->setTrajectoryTypeTVP("BangBang");
		else 
		{
			actuators[index]->setSpeed(actuators[index]->getMaxSpeed());
			actuators[index]->setAcceleration(actuators[index]->getMaxAcceleration());
			actuators[index]->setTrajectoryTypeTVP("MaximumSpeedAcceleration");
		}

		/*
			Now we are going to adjust the motion of all joints in TVP trajectory
		*/


		//we adjust the speed and acceleration to get all the joints are finishing the movement at the same time
		if(maxTargetTime<=0 && maxTa<=0)return;
		
		double speed=0,acel=0,maxSp=0,maxAcel=0;

		for(int i=0;i<(int)actuators.size();i++)
		{
			if(i==index)continue;
			maxSp=actuators[i]->getMaxSpeed();
			maxAcel=actuators[i]->getMaxAcceleration();

			//movement with speed and acceleration values adjust to timeTarget global
			speed=path[i]/(maxTargetTime-maxTa);
			acel=speed/maxTa;

			if (speed>maxSp || acel>maxAcel)//if true, try to change the movement to Bang Bang
			{
				acel=path[i]/(maxTa*maxTa);
				speed=acel*maxTa;

				if (speed>maxSp || acel>maxAcel)
					return; //we have to recalculate the ta time to this actuator
				actuators[i]->setAcceleration(acel);
				actuators[i]->setSpeed(speed);
				actuators[i]->setTrajectoryTypeTVP("BangBang");
			}
			else
			{
				actuators[i]->setSpeed(speed);
				actuators[i]->setAcceleration(acel);
				actuators[i]->setTrajectoryTypeTVP("MaximumSpeedAcceleration");
			}
		}

		
	}

	/******************************************************************
		IF TRAJECTORY SELECTED IS SPLINE TYPE
	*******************************************************************/

	else if (trajectory_type==SPLINE)
	{
		double lowestSpeed=100.0;
		double longestPath=0.0;
		double maxTargetTime=0.0;
		vector<double> pathJoint;


		for(int i=0;i<(int)actuators.size();i++)
		{
			pathJoint.push_back(q_target[i] - joints[i]->getValue());//Path each coordinate: target minus current coordinates

			if(actuators[i]->getMaxSpeed() <= lowestSpeed)
				lowestSpeed = actuators[i]->getMaxSpeed();
			if(fabs(pathJoint[i])>= longestPath)
				longestPath =fabs(pathJoint[i]);
		}
		
		targetTime = longestPath / lowestSpeed;

		double Tk=0.1,stretch=0.1;
		int nIterations=(int)(targetTime/Tk);
		vector<double> a,b,c,d;
		for (int i=0;i<nIterations+2;i++)
		{
			a.push_back(Tk);
			b.push_back(4*Tk);//2*(Tk+Tk)
			c.push_back(Tk);
			d.push_back(6*stretch);//(3/(Tk^2))*(2*Tk^2*(qk+1-qk))
		}
		vector<double> veloc= TDMA(a,b,c,d,nIterations);
		for (int i=1;i<(int)actuators.size();i++)
		{
			actuators[i]->setVelocIntermediates(veloc);
		}
	}
	/*******************************************
			BAD TRAJECTORY => TYPE==ERRORMOVEMENT
	********************************************/
	else 
		return;

}





/*
	Method to calculate to move the robot in a lineal path
*/
void RobotSim::linearPath (Transformation3D td3_final)
{	
	/*
		Calculating the vector of positions to cross and
		intermediate positions
	*/
	Vector3D posIni(getTcpLocation().position);
	Vector3D posFin(td3_final.position);
	all_space_points.clear();
	all_joints_value.clear();
	
	Vector3D direct_vec = posFin - posIni; //direction vector
	double total_path = direct_vec.module(); //distance L = total path
	
	if (total_path<EPS)
		return;
	
	double speed_to_achieve = 5.00; //velocity which will be imposed to all actuators
	double targe_time_max = total_path / speed_to_achieve;
	
	double divisions = targe_time_max * frequency;
	int div = (int)divisions;
	if (div == 0)div=1;
	direct_vec=direct_vec/div;
	
	for (int i=0;i<div;i++)
	{
		posIni.x+=direct_vec.x;
		posIni.y+=direct_vec.y;
		posIni.z+=direct_vec.z;
		Transformation3D _t(posIni);
		all_space_points.push_back(_t);
	} // we already have all the intermediate positions to calculate

}


void RobotSim::viaPoint()
{
	/*
		When robot have gone over the 90% of total distance 
		if there is a new target to go, it'll start to change 
		its direction to the other target --> Via Point

		Data: q_init, q_via_point, q_final, t1, t2 and tau
	*/
	double tau = 0.1*targetTime;
	double val = 0.00;
	double p1 = 0.00;

	//change the other target
	if (trajectory_type==TVP)
		for (int i=0;i<(int)actuators.size();i++)
		{
			//if (tInit<=time && time<=(t1-tau))
			//	val = p1 - ((t1 - time)/t1)*L1;

			if ((t1-tau)<time && time<=(t1+tau))
				val = p1 - (square(time - t1 - tau)/(4*tau*t1))*L1 + (square(time - t1 + tau)/(4*tau*t2))*L2;

			//else if ((t1+tau)<time && time<=(t1+t2))
			//	val = p1 + ((time - t1)/t2)*L2;

		}
	return val;
}

/*
	Different methods to calculate to move the joints or robot
*/

bool RobotSim::moveTo(double *_q)
{
	int num=(int)joints.size();
	vector<double> q;
	for(int i=0;i<num;i++)q.push_back(_q[i]);
	return moveTo(q);
}

bool  RobotSim::moveTo(Transformation3D t3d, unsigned char conf)
{
/*
the T3D is a relative transformation to the base (link[0])
*/

	if(conf==0x0)conf=getCurrentConfiguration();

	vector<double> q;
	if(inverseKinematics(t3d,q,conf))
	{
		for(int i=0;i<(int)joints.size();i++)
			joints[i]->setValue(q[i]);
		return true;
	}
	else return false;

}
bool RobotSim::moveTo(const vector<double> & _q)
{

	if(_q.size()!=joints.size())return false;
	if(!checkJointValues(_q))return false; //limits checking
	for(int i=0;i<(int)joints.size();i++)joints[i]->setValue(_q[i]);
	//goTo(_q);
	return true;
}

/*
	Go to absolute and relative coordinates
*/

bool RobotSim::goToAbs(Transformation3D t)
{
	unsigned char conf=getCurrentConfiguration();
	vector<double> q;
	
	if (path_type==LINEAR)
	{
		linearPath (t);
		int size = (int)all_space_points.size();
		if (!size)
			return false;
		for (int i=0;i<size;i++)
		{
			q.clear();
			if(!inverseKinematicsAbs(all_space_points[i],q,conf))
				return false;
			all_joints_value.push_back(q);
			conf=getCurrentConfiguration(q);
		}
		q = all_joints_value[0];
		index_pos = 0;
	}
	else if (path_type==DEFAULT)
		if(!inverseKinematicsAbs(t,q))
			return false;

	goTo(q);
	return true;
}

bool RobotSim::goTo(Transformation3D t)
{
	unsigned char conf=getCurrentConfiguration();
	vector<double> q;
	
	if (path_type==LINEAR)
	{
		linearPath (t);
		int size = (int)all_space_points.size();
		if (!size)
			return false;
		for (int i=0;i<size;i++)
		{
			q.clear();
			if(!inverseKinematics(all_space_points[i],q,conf))
				return false;
			all_joints_value.push_back(q);
			conf=getCurrentConfiguration(q);
		}
		q = all_joints_value[0];
		index_pos = 0;
	}
	else if (path_type==DEFAULT)
		if(!inverseKinematics(t,q,conf))
			return false;

	goTo(q);
	return true;
}


/*
	Simplified method to resolve the tridiagonal matrix of velocities in TVP trajectory
*/

vector<double> RobotSim::TDMA (vector<double> a, vector<double> b, vector<double> c,vector<double> d, int nIterations)
{
	/*It's the Thomas algorithm to resolve tridiagonal matrix. It's useful 
	to calculate the intermediate values of the velocity in the points to 
	interpolate (vk).

	c'1=c1/b1
	c'i= ci/(bi-c'[i-1]*ai)		i=2...n-1

	d'1=d1/b1
	d'i=(di-d'[i-1]*ai)/(bi-c'[i-1]*ai)	i=2...n

	xn=d'n
	xi=d'i-c'i*x[i+1]	i=n-1....1
	*/

	double denom;
	vector<double> gamma,betta,x;
	gamma.push_back(0);
	betta.push_back(0);
	
	int i=0;

	//a=2*(Tk+Tk)=4*Tk; b=Tk; c=Tk;
	//dk = (3/(Tk*Tk))*((Tk*Tk*(path))+(Tk*Tk*(path)));
	//dk = (3*(2*path))
	
	//dk = 6*path; //more simplificate

	for (i=1;i<=nIterations;i++)
	{
		denom=a[i]*gamma[i-1]+b[i];
		if(i!=nIterations)
			gamma.push_back((-c[i])/denom);
		betta.push_back((d[i]-(a[i]*betta[i-1]))/denom);
	}
	
	//calculate the solutions
	x.push_back(betta[nIterations]);
	int index=0;

	for (i=nIterations-1;i>=1;i--)
	{
		x.push_back(gamma[i]*x[index++]+betta[i]);
	}

	//reordenate the vector  to return all the velocities in order
	vector<double> veloc;

	veloc.push_back(0.0);//veloc init

	for(i=0;i<(int)x.size();i++)
	{
		veloc.push_back(x[i]);
	}
	veloc.push_back(0.0);//veloc final

	return veloc;

}



};//Namespace mr