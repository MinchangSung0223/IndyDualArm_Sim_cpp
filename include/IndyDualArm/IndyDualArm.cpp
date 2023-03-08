



#include "IndyDualArm.h"
#include "../ModernRobotics/modern_robotics.h"
#include <json/json.h>
#pragma comment(lib, "jsoncpp.lib")

#include <Eigen/Dense>
#include <iostream>
#include <iomanip>      // std::setprecision

using namespace std;
using namespace Eigen;
using namespace mr;

IndyDualArm::IndyDualArm(){

	cout<<"INDYDUALARM LOAD"<<endl;
	this->actuated_joint_num = 12;
	this->MRSetup();
	
}

IndyDualArm::IndyDualArm(class b3RobotSimulatorClientAPI_NoDirect* sim,int robotId){

	cout<<"INDYDUALARM LOAD"<<endl;
	this->actuated_joint_num = 12;
	this->MRSetup();
	this->sim= sim;
	this->robotId = robotId;
	this->initialize_joint();

}

void IndyDualArm::initialize_joint(){
	int actuated_joint_num = 0;
	int numJoints = sim->getNumJoints(this->robotId);

	for (int i = 0; i < numJoints; i++)
	{
		b3JointInfo jointInfo;
		sim->getJointInfo(this->robotId, i, &jointInfo);
		if (jointInfo.m_jointName[0] && jointInfo.m_jointType!=eFixedType)
		{
			this->actuated_joint_name.push_back(jointInfo.m_jointName);
			this->actuated_joint_id.push_back(i);		
			if(actuated_joint_num<6){
				this->right_actuated_joint_name.push_back(jointInfo.m_jointName);
				this->right_actuated_joint_id.push_back(i);
			}
			else{
				this->left_actuated_joint_name.push_back(jointInfo.m_jointName);
				this->left_actuated_joint_id.push_back(i);
			}
			// initialize motor
			b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_VELOCITY);
			controlArgs.m_maxTorqueValue  = 0.0;
			this->sim->setJointMotorControl(this->robotId,i,controlArgs);				
			b3RobotSimulatorJointMotorArgs controlArgs2(CONTROL_MODE_TORQUE);
			controlArgs2.m_maxTorqueValue  = 0.0;			
			this->sim->setJointMotorControl(this->robotId,i,controlArgs2);
			actuated_joint_num++;	
		}
		
	}
	
	this->eef_num = numJoints-1;
	this->left_eef_num =16;
	this->right_eef_num =8;
	this->actuated_joint_num = actuated_joint_num;	
}











VectorXd saturate(VectorXd val, VectorXd max_val){
	VectorXd retVal = Map<VectorXd>(val.data(), val.rows(), val.cols());
	for (int i = 0; i<val.size();i++){
		retVal[i] = min(max(val[i],-max_val[i]),max_val[i]);
	}
	return retVal;
}
void IndyDualArm::setRightTorques(JVec  torques ,JVec  max_torques){
	b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_TORQUE);
	VectorXd saturated_torques = saturate(torques,max_torques);
	for (int i = 0; i<torques.size();i++){
		controlArgs.m_maxTorqueValue  =saturated_torques[i];
		this->sim->setJointMotorControl(this->robotId,this->right_actuated_joint_id.at(i),controlArgs);
	}	
}

void IndyDualArm::setLeftTorques(JVec  torques ,JVec  max_torques){
	b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_TORQUE);
	VectorXd saturated_torques = saturate(torques,max_torques);
	for (int i = 0; i<torques.size();i++){
		controlArgs.m_maxTorqueValue  =saturated_torques[i];
		this->sim->setJointMotorControl(this->robotId,this->right_actuated_joint_id.at(i),controlArgs);
	}	
}
void IndyDualArm::resetRightPositions(JVec  positions){
	for (int i = 0; i<positions.size();i++){
		sim->resetJointState(this->robotId,this->right_actuated_joint_id.at(i),positions[i]);
	}
}
void IndyDualArm::resetLeftPositions(JVec  positions){
	for (int i = 0; i<positions.size();i++){
		sim->resetJointState(this->robotId,this->left_actuated_joint_id.at(i),positions[i]);
	}
}
void IndyDualArm::setRightPositions(JVec  positions ,JVec  max_torques, JVec Kp ,JVec Kd){
	b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_PD);
	for (int i = 0; i<positions.size();i++){
		controlArgs.m_maxTorqueValue  =max_torques[i];
		controlArgs.m_targetPosition = positions[i];
		controlArgs.m_kd = Kd(i);
		controlArgs.m_kp = Kp(i);
		sim->setJointMotorControl(this->robotId,this->right_actuated_joint_id.at(i),controlArgs);
	}	
}
void IndyDualArm::setLeftPositions(JVec  positions ,JVec  max_torques, JVec Kp ,JVec Kd){
	b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_PD);
	for (int i = 0; i<positions.size();i++){
		controlArgs.m_maxTorqueValue  =max_torques[i];
		controlArgs.m_targetPosition = positions[i];
		controlArgs.m_kd = Kd(i);
		controlArgs.m_kp = Kp(i);
		sim->setJointMotorControl(this->robotId,this->left_actuated_joint_id.at(i),controlArgs);
	}	
}
JVec IndyDualArm::getRightPositions(){
	JVec q = JVec::Zero();
	b3JointSensorState jointStates;
	for (int i = 0; i < this->right_actuated_joint_id.size(); i++)
	{
		if(sim->getJointState(this->robotId,this->right_actuated_joint_id.at(i), &jointStates)){
			q[i] = jointStates.m_jointPosition;
		}
	}
	return q;
}	
JVec IndyDualArm::getLeftPositions(){
	JVec q = JVec::Zero();
	b3JointSensorState jointStates;
	for (int i = 0; i < this->left_actuated_joint_id.size(); i++)
	{
		if(sim->getJointState(this->robotId,this->left_actuated_joint_id.at(i), &jointStates)){
			q[i] = jointStates.m_jointPosition;
		}
	}
	return q;
}	

JVec IndyDualArm::getRightVelocities(){
	JVec qdot = JVec::Zero();
	b3JointSensorState jointStates;
	for (int i = 0; i < this->right_actuated_joint_id.size(); i++)
	{
		if(sim->getJointState(this->robotId,this->right_actuated_joint_id.at(i), &jointStates)){
			qdot[i] =  jointStates.m_jointVelocity;
		}
	}
	return qdot;
}	
JVec IndyDualArm::getLeftVelocities(){
	JVec qdot = JVec::Zero();
	b3JointSensorState jointStates;
	for (int i = 0; i < this->left_actuated_joint_id.size(); i++)
	{
		if(sim->getJointState(this->robotId,this->left_actuated_joint_id.at(i), &jointStates)){
			qdot[i] =  jointStates.m_jointVelocity;
		}
	}
	return qdot;
}	


SE3 IndyDualArm::getRightEEFPose(){
	SE3 pose = MatrixXd::Identity(4,4);
	b3LinkState linkState;
	bool computeVelocity = true;
	bool computeForwardKinematics = true;
	sim->getLinkState(this->robotId, this->right_eef_num, computeVelocity, computeForwardKinematics, &linkState);
	VectorXd pos(3,1);
	pos<< linkState.m_worldLinkFramePosition[0], linkState.m_worldLinkFramePosition[1] , linkState.m_worldLinkFramePosition[2];
	btQuaternion orn = btQuaternion(linkState.m_worldLinkFrameOrientation[0],linkState.m_worldLinkFrameOrientation[1],linkState.m_worldLinkFrameOrientation[2],linkState.m_worldLinkFrameOrientation[3]);
	btMatrix3x3 R = btMatrix3x3(orn);
	pose(0,3) = pos[0];
	pose(1,3) = pos[1];
	pose(2,3) = pos[2];		
	for(int i =0;i<3;i++){
		btVector3 r = R[i];
		for(int j =0;j<3;j++){		
			pose(i,j) = r[j];
		}
	}
	return pose;
}
SE3 IndyDualArm::getLeftEEFPose(){
	SE3 pose = MatrixXd::Identity(4,4);
	b3LinkState linkState;
	bool computeVelocity = true;
	bool computeForwardKinematics = true;
	sim->getLinkState(this->robotId, this->left_eef_num, computeVelocity, computeForwardKinematics, &linkState);
	VectorXd pos(3,1);
	pos<< linkState.m_worldLinkFramePosition[0], linkState.m_worldLinkFramePosition[1] , linkState.m_worldLinkFramePosition[2];
	btQuaternion orn = btQuaternion(linkState.m_worldLinkFrameOrientation[0],linkState.m_worldLinkFrameOrientation[1],linkState.m_worldLinkFrameOrientation[2],linkState.m_worldLinkFrameOrientation[3]);
	btMatrix3x3 R = btMatrix3x3(orn);

	pose(0,3) = pos[0];
	pose(1,3) = pos[1];
	pose(2,3) = pos[2];		

	for(int i =0;i<3;i++){
		btVector3 r = R[i];
		for(int j =0;j<3;j++){		
			pose(i,j) = r[j];
		}
	}
	return pose;
}








//--------------------------------MR SETUP--------------------------------//




bool ReadFromFile(const char* filename, char* buffer, int len){
  FILE* r = fopen(filename,"rb");
  if (NULL == r)
       return false;
  size_t fileSize = fread(buffer, 1, len, r);
  fclose(r);
  return true;

}
bool ReadMRData(const char* filename,Json::Value &rootr){
	//cout<<"START ReadMRData"<<endl;
	const int BufferLength = 102400;
	char readBuffer[BufferLength] = {0,};
	if (false == ReadFromFile(filename, readBuffer, BufferLength)) {
		cout<<"Failed"<<endl;
		return -1;
	}
	std::string config_doc = readBuffer;

	Json::Reader reader;
	bool parsingSuccessful = reader.parse(config_doc,rootr);
	if ( !parsingSuccessful ) { 
		std::cout << "Failed to parse configuration\n" << reader.getFormatedErrorMessages(); 
		return -1;
		
	}
}
void IndyDualArm::MRSetup(){
	Json::Value rootr;
	bool ret = ReadMRData("MR_info.json",rootr);
	for(int i =0;i<6 ; i++){
		for(int j =0;j<this->actuated_joint_num;j++){
			this->relSlist(i,j) = rootr["relSlist"][i][j].asDouble();
			this->relBlist(i,j) = rootr["relBlist"][i][j].asDouble();
		}
	}
	//cout<<"MR Setup 2"<<endl;
	for(int i =0;i<6 ; i++){
		for(int j =0;j<this->actuated_joint_num/2;j++){
			this->Slist(i,j) = rootr["Slist"][i][j].asDouble();
			this->Blist(i,j) = rootr["Blist"][i][j].asDouble();
		}
	}	
	for(int i = 0;i< rootr["Mlist"].size(); i++){
		SE3 M;
		for(int j = 0;j< rootr["Mlist"][0].size(); j++){
			for(int k = 0;k< rootr["Mlist"][0][0].size(); k++){
				M(j,k) = rootr["Mlist"][i][j][k].asDouble();
			}
		}
		char str[50];		
		this->Mlist.push_back(M);
	}
	for(int i = 0;i< rootr["Glist"].size(); i++){
		Matrix6d G;
		for(int j = 0;j< rootr["Glist"][0].size(); j++){
			for(int k = 0;k< rootr["Glist"][0][0].size(); k++){
				G(j,k) = rootr["Glist"][i][j][k].asDouble();
			}
		}
		char str[50];
		this->Glist.push_back(G);	
	}	

	for (int i = 0;i<4;i++){
		for (int j = 0;j<4;j++){
			this->relM(i,j) = rootr["relM"][i][j].asDouble();
		}
	}
	for (int i = 0;i<4;i++){
		for (int j = 0;j<4;j++){
			this->M(i,j) = rootr["M"][i][j].asDouble();
		}
	}	
	for (int i = 0;i<4;i++){
		for (int j = 0;j<4;j++){
			this->Tbr(i,j) = rootr["Tbr"][i][j].asDouble();
		}
	}		
	for (int i = 0;i<4;i++){
		for (int j = 0;j<4;j++){
			this->Tbl(i,j) = rootr["Tbl"][i][j].asDouble();
		}
	}		
	cout<<"END MRSetup"<<endl;

}

IndyDualArm::~IndyDualArm(){
	
}
