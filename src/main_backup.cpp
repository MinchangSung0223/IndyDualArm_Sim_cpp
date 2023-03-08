#include "SharedMemory/b3RobotSimulatorClientAPI_NoDirect.h"
#include "SharedMemory/PhysicsClientSharedMemory_C_API.h"
#include "SharedMemory/b3RobotSimulatorClientAPI_InternalData.h"
#include "Bullet3Common/b3Vector3.h"
#include "Bullet3Common/b3Quaternion.h"
#include "Bullet3Common/b3HashMap.h"
#include <Eigen/Dense>
#include "Utils/b3Clock.h"
#include "../include/ModernRobotics/modern_robotics.h"
#include "../include/IndyDualArm/IndyDualArm.h"
#include <vector>
#include <iostream>


using namespace std;
using namespace Eigen;
using namespace mr;

extern const int CONTROL_RATE;
const int CONTROL_RATE = 2000;
const b3Scalar FIXED_TIMESTEP = 1.0 / ((b3Scalar)CONTROL_RATE);
b3SharedMemoryCommandHandle command;
int statusType, ret;
void printVector_(VectorXd vec,string str){
    cout<<str<<":";
    for(int i = 0;i<vec.rows()-1;i++)
        cout<<vec[i]<<",";
    cout<<vec[vec.rows()-1];
    cout<<""<<endl;
        
}
void printMatrix_(MatrixXd mat,string str){

int strlen = str.length();  
const char* sep ="-";
    for(int i = 0;i<(80-strlen)/2;i++)
        cout<<sep;
    cout<<str;
    if(strlen% 2 == 0){
        for(int i = 0;i<(80-strlen)/2;i++)
            cout<<sep;
    }else{
        for(int i = 0;i<(80-strlen)/2+1;i++)
            cout<<sep;
    }
    
    cout<<""<<endl;
        
    IOFormat OctaveFmt(StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
    
    std::cout << mat.format(OctaveFmt)<<endl;
    
cout<<"--------------------------------------------------------------------------------"<<endl;
}
int main()
{

    b3PhysicsClientHandle client = b3ConnectSharedMemory(SHARED_MEMORY_KEY);
    if (!b3CanSubmitCommand(client))
    {
        printf("Not connected, start a PyBullet server first, using python -m pybullet_utils.runServer\n");
        exit(0);
    }
    b3RobotSimulatorClientAPI_InternalData data;
    data.m_physicsClientHandle = client;
    data.m_guiHelper = 0;
    b3RobotSimulatorClientAPI_NoDirect sim;
    sim.setInternalData(&data);

    sim.resetSimulation();
    sim.setGravity( btVector3(0 , 0 ,-9.8));
    //int plane = sim.loadURDF("plane.urdf");
    //int robotId = sim.loadURDF("model/indy7.urdf");    
    int robotId = sim.loadURDF("model/IndyDualArm.urdf"); 
    sim.setTimeStep(FIXED_TIMESTEP);
    IndyDualArm IndyDualArm(&sim,robotId);
    double t = 0;   
    
    /*
    Indy7 indy7(&sim,robotId);
    
    Eigen::VectorXd torques(6,1);
    Eigen::VectorXd MAX_TORQUES(6,1);    
    torques<<0.0,0.0,0.0,0.0,0.0,0.0;
    MAX_TORQUES<<431.97,431.97,197.23,79.79,79.79,79.79;

    double jointAccelerations[6]={0,0,0,0,0,0};
    double jointForcesOutput[6]={0,0,0,0,0,0};
    MatrixXd M = indy7.getM();
    MatrixXd Slist = indy7.getSlist();
    MatrixXd Blist = indy7.getBlist();
    vector<MatrixXd> Mlist = indy7.getMlist();
    vector<MatrixXd> Glist = indy7.getGlist();
    VectorXd g(3,1);
	g<<0.0,0.0,-9.8;
    VectorXd Ftip(6,1);
	Ftip<<0,0,0,0,0,0;
    int m = indy7.getActuatedJointNum();    	
    VectorXd qddot=  VectorXd::Zero(m);
    VectorXd eint=  VectorXd::Zero(m);
        
    VectorXd dq=  VectorXd::Zero(m);
    VectorXd dqdot=  VectorXd::Zero(m);  
    VectorXd e =        VectorXd::Zero(m);  
    double dt= FIXED_TIMESTEP;
    double scale = 100;
    VectorXd Kp=  VectorXd::Zero(m);
    Kp<<70,70,40,25,25,18;

    VectorXd Ki=  VectorXd::Zero(m);
    Ki<<1,1,0.5,0.2,0.2,0.1;    
    VectorXd Kd=  VectorXd::Zero(m);    
    Kd<<55,55,30,15,15,3;
    
    MatrixXd matKp = scale*Kp.asDiagonal();
    MatrixXd matKi = scale*Ki.asDiagonal();
    MatrixXd matKd = scale*Kd.asDiagonal();        
    */
    double jointAccelerations[12]={0,0,0,0,0,0,0,0,0,0,0,0};
    double jointForcesOutput[12]={0,0,0,0,0,0,0,0,0,0,0,0};
    Eigen::VectorXd torques(12,1);
    Eigen::VectorXd MAX_TORQUES(12,1); 
    VectorXd g(3,1);
    g<<-0.000000 ,8.495709 ,-4.905000;
    VectorXd Ftip(6,1);
    Ftip<<0,0,0,0,0,0;    
    MatrixXd M = IndyDualArm.getM();
    MatrixXd Slist = IndyDualArm.getSlist();
    MatrixXd Blist = IndyDualArm.getBlist();
    printMatrix_(Slist,"=============Slist============");
b3Clock::usleep(1000. * 1000000000. * FIXED_TIMESTEP);
    MatrixXd relM = IndyDualArm.getrelM();
    MatrixXd relSlist = IndyDualArm.getrelSlist();
    MatrixXd relBlist = IndyDualArm.getrelBlist();
    

    vector<MatrixXd> Mlist = IndyDualArm.getMlist();
    vector<MatrixXd> Glist = IndyDualArm.getGlist();   
    VectorXd qddot=  VectorXd::Zero(6);
    Eigen::VectorXd HOME_POS(12,1); 
    HOME_POS  << -0.186, -0.886, -1.298 ,-0.643, -0.960, 0.070, 0.186 ,0.886 ,1.298, 0.643 ,0.960 ,-0.070;
    MAX_TORQUES<<431.97,431.97,197.23,79.79,79.79,79.79,431.97,431.97,197.23,79.79,79.79,79.79;
    IndyDualArm.resetPositions(&sim,HOME_POS);
    while(1){
        VectorXd right_q = IndyDualArm.getRightQ( &sim);
        VectorXd left_q = IndyDualArm.getLeftQ( &sim);
        VectorXd right_qdot = IndyDualArm.getRightQdot( &sim);
        VectorXd left_qdot = IndyDualArm.getLeftQdot( &sim);        
        VectorXd q = IndyDualArm.getQ( &sim);
        VectorXd qdot = IndyDualArm.getQdot( &sim);
        VectorXd relq(12);
        relq<<-right_q[5],-right_q[4],-right_q[3],-right_q[2],-right_q[1],-right_q[0],left_q[0],left_q[1],left_q[2],left_q[3],left_q[4],left_q[5];
        MatrixXd relT = FKinSpace(relM,relSlist,relq);
        printMatrix_(relT,"relT");
        bool ret =  sim.calculateInverseDynamics(robotId, q.data(), qdot.data(), jointAccelerations, torques.data());
        //IndyDualArm.setPositions(&sim,HOME_POS,MAX_TORQUES);
         IndyDualArm.setTorques(&sim,torques,MAX_TORQUES);
    	sim.stepSimulation();	
    	b3Clock::usleep(1000. * 1000. * FIXED_TIMESTEP);
    	t = t+FIXED_TIMESTEP;	
    }
    
    
    
}
