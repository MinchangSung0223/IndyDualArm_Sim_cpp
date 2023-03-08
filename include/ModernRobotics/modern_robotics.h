#pragma once

#include <Eigen/Dense>
#include <vector>
#define JOINTNUM 6
#define RELATIVEJOINTNUM 12
/**
 * \defgroup ModernRobotics
 * @brief Short doc of mr
 *
 * Long doc of mr.
 */

/**
 * @brief Modern Robotics Library
 *
 * \ingroup mr
 */
namespace mr {
    //TYPE DEFINE
    typedef Eigen::Matrix<double, 6, 1> Vector6d;   
    typedef Eigen::Vector3d Vector3d;   
    typedef Eigen::Vector4d Vector4d;  
    typedef Eigen::Matrix<double, 6, 6> Matrix6d;  
    typedef Eigen::Matrix<double, 6, 6> AdjMap;  
    typedef Eigen::Matrix<double, 6, 6> adjMap;  
    
    
    typedef Eigen::Matrix3d Matrix3d;  
    typedef Eigen::Matrix<double, 4, 4> SE3;
    typedef Eigen::Matrix<double, 3, 3> SO3;
    typedef Eigen::Matrix<double, 4, 4> se3;
    typedef Eigen::Matrix<double, 3, 3> so3;
    typedef Eigen::Matrix<double, JOINTNUM, 1> JVec;
    typedef Eigen::Matrix<double, 6, JOINTNUM> ScrewList;
    typedef Eigen::Matrix<double, 6, JOINTNUM> Jacobian;
    typedef Eigen::Matrix<double, JOINTNUM,6 > PinvJacobian;
    typedef Eigen::Matrix<double, 6*JOINTNUM, 1> JacobianVec;
    typedef Eigen::Matrix<double, 6*JOINTNUM, 1> PinvJacobianVec;
    typedef Eigen::Matrix<double, 6*JOINTNUM, JOINTNUM> DerivativeJacobianVec;
    typedef Eigen::Matrix<double, JOINTNUM, JOINTNUM> MassMat;
    typedef Eigen::Matrix<double, JOINTNUM, JOINTNUM> CoriolisMat;  
    typedef Eigen::Matrix<double, RELATIVEJOINTNUM,1 > RelativeJVec;
    typedef Eigen::Matrix<double, 6, RELATIVEJOINTNUM> RelativeScrewList;
    typedef Eigen::Matrix<double, 6, RELATIVEJOINTNUM> RelativeJacobian;
    typedef Eigen::Matrix<double, RELATIVEJOINTNUM,6 > RelativePinvJacobian; 
    typedef Eigen::Matrix<double, 6*RELATIVEJOINTNUM, 1> RelativeJacobianVec;
    typedef Eigen::Matrix<double, 6*RELATIVEJOINTNUM, 1> RelativePinvJacobianVec;
    typedef Eigen::Matrix<double, 6*RELATIVEJOINTNUM, RELATIVEJOINTNUM> DerivativeRelativeJacobianVec;


    //FUNCTIONS
    
    SE3 MatrixExp6(const se3& se3mat);
    se3 VecTose3(const Vector6d& V);
    so3 VecToso3(const Vector3d& omg);
    Vector3d so3ToVec(const so3& so3mat);
    Vector6d se3ToVec(const se3& T);
    Vector4d AxisAng3(const Vector3d& expc3);
    bool NearZero(const double val);
    SO3 MatrixExp3(const so3& so3mat);
    SE3 TransInv(const SE3& transform);
    Vector3d Normalize(Vector3d V);
    SE3 FKinSpace(const SE3& M, const ScrewList& Slist, const JVec& thetaList);
    SE3 FKinBody(const SE3& M, const ScrewList& Blist, const JVec& thetaList);
    Jacobian JacobianBody(const ScrewList& Blist, const JVec& thetaList);
    PinvJacobian PinvJacobianBody(const ScrewList& Blist, const JVec& thetaList);
    PinvJacobian PinvJacobianSpace(const ScrewList& Slist, const JVec& thetaList);
    Jacobian JacobianSpace(const ScrewList& Slist, const JVec& thetaList);
    Vector6d getU1(const Jacobian& J);
    Matrix6d getP1(const Jacobian& J);
    Vector6d getUm(const Jacobian& J);
    Matrix6d getPm(const Jacobian& J);
    Vector6d getRelativeU1(const RelativeJacobian& J);
    Matrix6d getRelativeP1(const RelativeJacobian& J);
    Vector6d getRelativeUm(const RelativeJacobian& J);
    Matrix6d getRelativePm(const RelativeJacobian& J);
    unsigned int getMaxCoeff(const Vector6d& vec);
    unsigned int getMinCoeff(const Vector6d& vec);
    double Manipulability_mu1(const Jacobian& J);
    double Manipulability_mu2(const Jacobian& J);
    double Manipulability_mu3(const Jacobian& J);
    double RelativeManipulability_mu1(const RelativeJacobian& J);
    double RelativeManipulability_mu2(const RelativeJacobian& J);
    double RelativeManipulability_mu3(const RelativeJacobian& J);
    JacobianVec VectorizeJacobian(const Jacobian& J);
    DerivativeJacobianVec DerivativeVectorizeJacobianSpace(const ScrewList& Slist, const JVec& thetaList);
    DerivativeJacobianVec DerivativeVectorizeJacobianBody(const ScrewList& Blist, const JVec& thetaList);
    DerivativeRelativeJacobianVec DerivativeVectorizeRelativeJacobianSpace(const RelativeScrewList& relSlist, const RelativeJVec& thetaList);
    DerivativeRelativeJacobianVec DerivativeVectorizeRelativeJacobianBody(const RelativeScrewList& relBlist, const RelativeJVec& thetaList);
    PinvJacobianVec VectorizePinvJacobian(const PinvJacobian& pinvJ);
    RelativeJacobianVec VectorizeRelativeJacobian(const RelativeJacobian& J);
    RelativePinvJacobianVec VectorizeRelativePinvJacobian(const RelativePinvJacobian& pinvJ);
    SE3 RelativeFKinBody(const SE3& M, const RelativeScrewList& relBlist, const RelativeJVec& thetaList);
    SE3 RelativeFKinSpace(const SE3& M, const RelativeScrewList& relSlist, const RelativeJVec& thetaList);
    RelativeJacobian RelativeJacobianBody(const RelativeScrewList& relBlist, const RelativeJVec& thetaList);
    RelativePinvJacobian RelativePinvJacobianBody(const RelativeScrewList& relBlist, const RelativeJVec& thetaList);
    RelativePinvJacobian RelativePinvJacobianSpace(const RelativeScrewList& relSlist, const RelativeJVec& thetaList);
    RelativeJacobian RelativeJacobianSpace(const RelativeScrewList& relSlist, const RelativeJVec& thetaList);
    RelativeJVec getRelativeJVec(const JVec& J1, const JVec& J2);
    Matrix6d Adjoint(const SE3& T);
    Vector6d TwistSE3toSE3(const SE3 &Xd,const SE3 &X);
    so3 MatrixLog3(const SO3& R);
    se3 MatrixLog6(const SE3& T);
    SE3 RpToTrans(const SO3& R, const Vector3d& p);
    void TransToRp( const SE3& T, SO3& R, Vector3d& p);
    SO3 RotInv(const SO3& rotMatrix);
    Vector6d ScrewToAxis(Vector3d q, Vector3d s, double h);
    Vector6d AxisAng6(const Eigen::VectorXd& expc6);
    SO3 ProjectToSO3(const SO3& M);
    SE3 ProjectToSE3(const SE3& M);
    double DistanceToSO3(const SO3& M);
    double DistanceToSE3(const Eigen::Matrix4d& T);
    bool IKinBody(const ScrewList& Blist, const SE3& M, const SE3& T,JVec& thetalist, double eomg, double ev);
    bool RelativeIKinBody(const RelativeScrewList& relBlist, const SE3& relM, const SE3& relT,RelativeJVec& thetalist, double eomg, double ev);
    bool IKinSpace(const ScrewList& Slist, const SE3& M, const SE3& T,JVec& thetalist, double eomg, double ev);
    bool RelativeIKinSpace(const RelativeScrewList& relSlist, const SE3& relM, const SE3& relT, RelativeJVec& thetalist, double eomg, double ev);
    Matrix6d ad(const Vector6d& V);
    JVec InverseDynamics(const JVec& thetalist, const JVec& dthetalist, const JVec& ddthetalist,const Vector3d& g, const Vector6d& Ftip, const std::vector<SE3>& Mlist,const std::vector<Matrix6d>& Glist, const ScrewList& Slist);
    JVec GravityForces(const JVec& thetalist, const Vector3d& g,const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const ScrewList& Slist);
    MassMat MassMatrix(const JVec& thetalist, const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const ScrewList& Slist);
    JVec VelQuadraticForces(const JVec& thetalist, const JVec& dthetalist, const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const ScrewList& Slist);
    Vector6d  EndEffectorForces(const JVec& thetalist, const Vector6d& Ftip,const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const ScrewList& Slist);
    void EulerStep(JVec& thetalist, JVec& dthetalist, const JVec& ddthetalist, double dt);
    void EulerStep(RelativeJVec& thetalist, RelativeJVec& dthetalist, const RelativeJVec& ddthetalist, double dt);
    double CubicTimeScaling(double Tf, double t);
    double QuinticTimeScaling(double Tf, double t);
    std::vector<SE3> ScrewTrajectory(const SE3& Xstart, const SE3& Xend, double Tf, int N, int method);
    std::vector<SE3> CartesianTrajectory(const SE3& Xstart, const SE3& Xend, double Tf, int N, int method);
    void printMatrixJVec (const JVec & Mat);
    void printMatrixRelativeJVec (const RelativeJVec & Mat);
    void printMatrixSE3 (const SE3 & Mat);
    void printMatrixSO3 (const SO3 & Mat);
    void printMatrixse3 (const se3 & Mat);
    void printMatrixso3 (const so3 & Mat);
    void printMatrixScrewList (const ScrewList & Mat);
    void printMatrixJacobian (const Jacobian & Mat);
    void printMatrixPinvJacobian (const PinvJacobian & Mat);
    void printMatrixJacobianVec (const JacobianVec & Mat);
    void printMatrixPinvJacobianVec (const PinvJacobianVec & Mat);
    void printMatrixDerivativeJacobianVec (const DerivativeJacobianVec & Mat);
    void printMatrixRelativeScrewList (const RelativeScrewList & Mat);
    void printMatrixRelativeJacobian (const RelativeJacobian & Mat);
    void printMatrixRelativePinvJacobian (const RelativePinvJacobian & Mat);
    void printMatrixRelativeJacobianVec (const RelativeJacobianVec & Mat);
    void printMatrixRelativePinvJacobianVec (const RelativePinvJacobianVec & Mat);
    void printMatrixDerivativeRelativeJacobianVec (const DerivativeRelativeJacobianVec & Mat);
    void printMatrixMassMat (const MassMat & Mat);
    void printMatrixCoriolisMat (const CoriolisMat & Mat);
    void printMatrixVector6d (const Vector6d & Mat);
    void printMatrixVector3d (const Vector3d & Mat);
    void printMatrixVector4d (const Vector4d & Mat);
    void printMatrixMatrix6d (const Matrix6d & Mat);
    void printMatrixMatrix3d (const Matrix3d & Mat);
}/* namespace mr */

