#ifndef INDYDUALARM_SETUP_H
#define INDYDUALARM_SETUP_H

#include "../ModernRobotics/modern_robotics.h"
#include <vector>
#include <Eigen/Dense>
#include <iostream>
using namespace Eigen;
using namespace std;
using namespace mr;

//----------------------pybullet----------------------------------
#include "SharedMemory/b3RobotSimulatorClientAPI_NoDirect.h"
#include "Bullet3Common/b3HashMap.h"

//----------------------------------------------------------------


class IndyDualArm
{
	    
	int robotId;
	int actuated_joint_num;
	int eef_num;
	int right_eef_num;
	int left_eef_num;
	b3RobotSimulatorClientAPI_NoDirect* sim;	

	vector<int> actuated_joint_id;
	vector<string> actuated_joint_name;

	vector<int> right_actuated_joint_id;
	vector<string> right_actuated_joint_name;

	vector<int> left_actuated_joint_id;
	vector<string> left_actuated_joint_name;

	RelativeScrewList relSlist;
	RelativeScrewList relBlist;

	ScrewList Slist;
	ScrewList Blist;
	SE3 M;

	vector<Matrix6d> Glist;	
	vector<SE3> Mlist;		
	SE3 relM;				
	SE3 Tbr;				
	SE3 Tbl;				
public:

	IndyDualArm(class b3RobotSimulatorClientAPI_NoDirect* sim,int robotId);
	void initialize_joint();
	void setRightTorques(JVec  torques ,JVec  max_torques);
	void setLeftTorques(JVec  torques ,JVec  max_torques);
	void resetRightPositions(JVec  positions);
	void resetLeftPositions(JVec  positions);
	void setRightPositions(JVec  positions ,JVec  max_torques, JVec Kp ,JVec Kd);
	void setLeftPositions(JVec  positions ,JVec  max_torques, JVec Kp ,JVec Kd);
	JVec getRightPositions();
	JVec getLeftPositions();
	JVec getRightVelocities();
	JVec getLeftVelocities();
	SE3 getRightEEFPose();
	SE3 getLeftEEFPose();




//---------------------------------MR SETUP-------------------------
	IndyDualArm();
	void MRSetup();
	int getActuatedJointNum(){
		return this->actuated_joint_num;
	};

	RelativeScrewList getRelativeSlist(){
		return this->relSlist;
	};
	ScrewList getSlist(){
		return this->Slist;
	};
	ScrewList getBlist(){
		return this->Blist;
	};
	RelativeScrewList getRelativeBlist(){
		return this->relBlist;
	};	
	SE3 getRelativeM(){
		return this->relM;
	};	

	SE3 getM(){
		return this->M;
	};	
	SE3 getTbr(){
		return this->Tbr;
	};	
	SE3 getTbl(){
		return this->Tbl;
	};		
	vector<SE3> getMlist(){
		return this->Mlist;
	}
	vector<Matrix6d> getGlist(){
		return this->Glist;
	}
	virtual ~IndyDualArm();

};
#endif  //INDYDUALARM_SETUP_H
