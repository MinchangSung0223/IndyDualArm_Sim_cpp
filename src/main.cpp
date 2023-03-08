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
const int CONTROL_RATE = 4000;
const b3Scalar FIXED_TIMESTEP = 1.0 / ((b3Scalar)CONTROL_RATE);
b3SharedMemoryCommandHandle command;
b3RobotSimulatorAddUserDebugLineArgs args;
int statusType, ret;
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
    b3ConfigureDebugVisualizerEnum flag=COV_ENABLE_GUI ;
    sim.configureDebugVisualizer(flag, false);
    double camDistance = 1.5;
    double camPitch = -10;
    double camYaw = 90;
    btVector3 targetPos(0,0,0.5);
    sim.resetDebugVisualizerCamera(camDistance, camPitch, camYaw, targetPos);

    int robotId = sim.loadURDF("model/IndyDualArm.urdf"); 
    int planeId = sim.loadURDF("plane.urdf"); 
    sim.setTimeStep(FIXED_TIMESTEP);

    IndyDualArm *indyDualArm = new IndyDualArm(&sim,robotId);

    RelativeScrewList   relSlist = indyDualArm->getRelativeSlist();
    RelativeScrewList   relBlist = indyDualArm->getRelativeBlist();
    ScrewList   Slist = indyDualArm->getSlist();
    ScrewList   Blist = indyDualArm->getBlist();
    std::vector<SE3> Mlist = indyDualArm->getMlist();
    std::vector<Matrix6d> Glist = indyDualArm->getGlist();
    SE3 relM = indyDualArm->getRelativeM();
    SE3 M = indyDualArm->getM();
    SE3 Tbr = indyDualArm->getTbr();
    SE3 Tbl = indyDualArm->getTbl();
    
    JVec thetalist_r;
    JVec thetalist_l;
    JVec dthetalist = JVec::Zero();
    JVec ddthetalist = JVec::Zero();
    thetalist_r<<   -0.186, -0.886, -1.298 ,-0.643, -0.960, 0.070;
    thetalist_l<<   0.186 ,0.886 ,1.298, 0.643 ,0.960 ,-0.070;
    JVec MAX_TORQUES;
    MAX_TORQUES<< 431.97,431.97,197.23,79.79,79.79,79.79;
    RelativeJVec relative_thetalist=getRelativeJVec(thetalist_r,thetalist_l);
    Vector3d g(0,0,-9.8);
    Vector6d Ftip=Vector6d::Zero();
    SE3 Xd;
    Xd<<-1,0,0,0,
         0,1,0,0,
         0,0,-1,0.4,
         0,0,0,1;
    double t = 0;   
    double dt = FIXED_TIMESTEP;
    indyDualArm->resetRightPositions(thetalist_r);
    indyDualArm->resetLeftPositions(thetalist_l);
    SE3 T_right = Tbr*FKinSpace(M, Slist,thetalist_r);
    RelativeJVec thetalist_rel = getRelativeJVec(thetalist_r,thetalist_l);
    SE3 T_rel = RelativeFKinSpace(relM, relSlist,thetalist_rel);    
    SE3 T_left = T_right*T_rel;

    double fromRightXYZ[3]={T_right(0,3),T_right(1,3),T_right(2,3)};    
    double toRightXYZ[3]={T_right(0,3),T_right(1,3),T_right(2,3)};
    double fromLeftXYZ[3]={T_left(0,3),T_left(1,3),T_left(2,3)};    
    double toLeftXYZ[3]={T_left(0,3),T_left(1,3),T_left(2,3)};
    while(1){
        JVec q_r = indyDualArm-> getRightPositions();
        JVec q_l = indyDualArm-> getLeftPositions();
        RelativeJVec q_rel = getRelativeJVec(q_r,q_l);
        SE3 T_right = Tbr*FKinSpace(M, Slist,q_r);
        SE3 T_rel = RelativeFKinSpace(relM, relSlist,q_rel);
        RelativeJacobian Jb_rel = RelativeJacobianBody(relBlist,q_rel);
        RelativeJacobian Js_rel = RelativeJacobianSpace(relSlist,q_rel);
        double Jb_mu1 = RelativeManipulability_mu1(Jb_rel);
        double Js_mu1 = RelativeManipulability_mu1(Js_rel);
        double Jb_mu2 = RelativeManipulability_mu2(Jb_rel);
        double Js_mu2 = RelativeManipulability_mu2(Js_rel);
        double Jb_mu3 = RelativeManipulability_mu3(Jb_rel);
        double Js_mu3 = RelativeManipulability_mu3(Js_rel);


    
        Vector6d Jb_u1 = getRelativeU1(Jb_rel);
        Vector6d Js_u1 = getRelativeU1(Js_rel);
        Matrix6d Jb_P1 =getRelativeP1(Jb_rel);
        Matrix6d Js_P1 =getRelativeP1(Js_rel);
        Matrix6d Jb_Pm =getRelativePm(Jb_rel);
        Matrix6d Js_Pm =getRelativePm(Js_rel);
        
        RelativeJacobian Pm_P1Js = (Js_Pm-Js_P1)*Js_rel;
        RelativeJacobian Pm_P1Jb = (Jb_Pm-Jb_P1)*Jb_rel;
        RelativeJacobianVec vecPm_P1Js = VectorizeRelativeJacobian(Pm_P1Js);
        RelativeJacobianVec vecPm_P1Jb = VectorizeRelativeJacobian(Pm_P1Jb);

        
        DerivativeRelativeJacobianVec vecdJs = DerivativeVectorizeRelativeJacobianSpace(relSlist, q_rel);
        DerivativeRelativeJacobianVec vecdJb = DerivativeVectorizeRelativeJacobianBody(relBlist, q_rel);
        RelativeJVec Js_dmu1 = Js_mu1* vecPm_P1Js.transpose()*vecdJs;
        RelativeJVec Jb_dmu1 = Jb_mu1* vecPm_P1Jb.transpose()*vecdJb;
        RelativePinvJacobian pinvJb_rel = RelativePinvJacobianBody(relBlist,q_rel);        
        RelativePinvJacobian pinvJs_rel = RelativePinvJacobianSpace(relSlist,q_rel);
        RelativeJacobianVec vecpinvJT = VectorizeRelativeJacobian(pinvJb_rel.transpose());
        RelativeJVec Js_dmu3 = Js_mu3* vecpinvJT.transpose()*vecdJs;
        RelativeJVec Jb_dmu3 = Jb_mu3* vecpinvJT.transpose()*vecdJb;
        
        
        SE3 T_left = T_right*T_rel;
        Xd(0,3) = 0.2*cos(2*M_PI*t*20);
        Xd(1,3) = 0.2*sin(2*M_PI*t*20);
        Vector6d Xe_rel = TwistSE3toSE3(Xd,T_rel);
        Matrix<double,RELATIVEJOINTNUM,RELATIVEJOINTNUM> Identity12x12 =  Matrix<double,RELATIVEJOINTNUM,RELATIVEJOINTNUM> ::Identity();
       // RelativeJVec qdot_rel =pinvJb_rel*Xe_rel + (Identity12x12- pinvJs_rel*Js_rel)*(-Js_dmu1);
        //RelativeJVec qdot_rel =20.0*(Identity12x12- pinvJb_rel*Jb_rel)*(-Jb_dmu1);
        RelativeJVec qdot_rel =200*pinvJb_rel*Xe_rel+(Identity12x12- pinvJb_rel*Jb_rel)*(-Jb_dmu1);
        RelativeJVec qddot_rel = RelativeJVec::Zero();

        EulerStep(q_rel, qdot_rel, qddot_rel, dt);

        JVec thetalist_r;
        thetalist_r<<-q_rel[5],-q_rel[4],-q_rel[3],-q_rel[2],-q_rel[1],-q_rel[0];
        JVec thetalist_l;
        thetalist_l<<q_rel[6],q_rel[7],q_rel[8],q_rel[9],q_rel[10],q_rel[11];


/*
        static unsigned int print_cnt = 0;
        double lifetime = 0.1;
        if (++print_cnt > int(lifetime*CONTROL_RATE))
        {
            toRightXYZ[0] = T_right(0,3);
            toRightXYZ[1] = T_right(1,3);
            toRightXYZ[2] = T_right(2,3);
            toLeftXYZ[0] = T_left(0,3);
            toLeftXYZ[1] = T_left(1,3);
            toLeftXYZ[2] = T_left(2,3);
            args.m_lineWidth=1;
            args.m_lifeTime=lifetime*1000;
            args.m_colorRGB[0] = 0;
            args.m_colorRGB[1] = 1;
            args.m_colorRGB[2] = 0;
            sim.addUserDebugLine(fromLeftXYZ, toLeftXYZ, args);
            sim.addUserDebugLine(fromRightXYZ, toRightXYZ, args);
            args.m_lineWidth=2;
            args.m_colorRGB[0] = 1;
            args.m_colorRGB[1] = 0;
            args.m_colorRGB[2] = 0;
            args.m_lifeTime=lifetime*10;
            sim.addUserDebugLine(toRightXYZ, toLeftXYZ, args);
            fromRightXYZ[0] = toRightXYZ[0];
            fromRightXYZ[1] = toRightXYZ[1];
            fromRightXYZ[2] = toRightXYZ[2];
            fromLeftXYZ[0] = toLeftXYZ[0];
            fromLeftXYZ[1] = toLeftXYZ[1];
            fromLeftXYZ[2] = toLeftXYZ[2];        
            print_cnt = 0;
        }
*/
        indyDualArm->resetRightPositions(thetalist_r);
        indyDualArm->resetLeftPositions(thetalist_l);
        sim.stepSimulation();   
        b3Clock::usleep(1000. * 1000. * FIXED_TIMESTEP);
        t = t+FIXED_TIMESTEP;   
    }

}
