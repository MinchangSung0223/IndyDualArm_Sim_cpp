#include "modern_robotics.h"

/*
 * modernRobotics.cpp
 * Adapted from modern_robotics.py provided by modernrobotics.org
 * Provides useful Jacobian and frame representation functions
 */
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <iostream>
# define M_PI           3.14159265358979323846  /* pi */
using namespace std;

namespace mr {
    so3 Identity3x3=Eigen::Matrix3d::Identity();
    SE3 Identity4x4=Eigen::Matrix4d::Identity();
    so3 Zero3x3=Eigen::Matrix3d::Zero();
    Matrix6d Zero6x6 = Matrix6d::Zero();
    Vector3d Zero3d = Vector3d::Zero();
    Vector6d Zero6d = Vector6d::Zero();
	SE3 MatrixExp6(const se3& se3mat) {
		// Extract the angular velocity vector from the transformation matrix
		so3 se3mat_cut = se3mat.block<3, 3>(0, 0);
		Vector3d omgtheta = so3ToVec(se3mat_cut);
		SE3 m_ret(4, 4);
		// If negligible rotation, m_Ret = [[Identity, angular velocty ]]
		//									[	0	 ,		1		   ]]
		if (NearZero(omgtheta.norm())) {
			// Reuse previous variables that have our required size
			se3mat_cut<<1, 0 , 0 , 0 ,1 ,0 , 0 , 0 ,1;
			omgtheta << se3mat(0, 3), se3mat(1, 3), se3mat(2, 3);
			m_ret << se3mat_cut, omgtheta,
				0, 0, 0, 1;
			return m_ret;
		}
		// If not negligible, MR page 105
		else {
			double theta = (AxisAng3(omgtheta))(3);
			Eigen::Matrix3d omgmat = se3mat.block<3, 3>(0, 0) / theta;
			Eigen::Matrix3d expExpand = Identity3x3* theta + (1 - std::cos(theta)) * omgmat + ((theta - std::sin(theta)) * (omgmat * omgmat));
			Eigen::Vector3d linear(se3mat(0, 3), se3mat(1, 3), se3mat(2, 3));
			Eigen::Vector3d GThetaV = (expExpand*linear) / theta;
			m_ret << MatrixExp3(se3mat_cut), GThetaV,
				0, 0, 0, 1;
			return m_ret;
		}
	}
	se3 VecTose3(const Vector6d& V) {
		// Separate angular (exponential representation) and linear velocities
		Eigen::Vector3d exp(V(0), V(1), V(2));
		Eigen::Vector3d linear(V(3), V(4), V(5));
		// Fill in values to the appropriate parts of the transformation matrix
		SE3 m_ret(4, 4);
		m_ret << VecToso3(exp), linear,
			0, 0, 0, 0;
		return m_ret;
	}
	so3 VecToso3(const Vector3d& omg) {
		so3 m_ret;
		m_ret << 0, -omg(2), omg(1),
			omg(2), 0, -omg(0),
			-omg(1), omg(0), 0;
		return m_ret;
	}
	Vector3d so3ToVec(const so3& so3mat) {
		Vector3d v_ret;
		v_ret << so3mat(2, 1), so3mat(0, 2), so3mat(1, 0);
		return v_ret;
	}
	Vector6d se3ToVec(const se3& T) {
		Vector6d m_ret(6);
		m_ret << T(2, 1), T(0, 2), T(1, 0), T(0, 3), T(1, 3), T(2, 3);

		return m_ret;
	}
	Vector4d AxisAng3(const Vector3d& expc3) {
		Vector4d v_ret;
		v_ret << Normalize(expc3), expc3.norm();
		return v_ret;
	}
	bool NearZero(const double val) {
		return (std::abs(val) < .000001);
	}
	SO3 MatrixExp3(const so3& so3mat) {
		Vector3d omgtheta = so3ToVec(so3mat);

		SO3 m_ret = Identity3x3;
		if (NearZero(so3mat.norm())) {
			return m_ret;
		}
		else {
			double theta = (AxisAng3(omgtheta))(3);
			Eigen::Matrix3d omgmat = so3mat * (1 / theta);
			return m_ret + std::sin(theta) * omgmat + ((1 - std::cos(theta)) * (omgmat * omgmat));
		}
	}	
	SE3 TransInv(const SE3& transform) {
		SO3 R;
		Vector3d p;
		TransToRp(transform,R,p);
		SO3 Rt = R.transpose();
		Vector3d t = -(Rt * p);
		SE3 inv;
		inv <<0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
		inv.block(0, 0, 3, 3) = Rt;
		inv.block(0, 3, 3, 1) = t;
		inv(3, 3) = 1;
		return inv;
	}
	Vector3d Normalize(Vector3d V) {
		V.normalize();
		return V;
	}
	SE3 FKinSpace(const SE3& M, const ScrewList& Slist, const JVec& thetaList) {
		SE3 T = M;
		for (int i = (thetaList.size() - 1); i > -1; i--) {
			T = MatrixExp6(VecTose3(Slist.col(i)*thetaList(i))) * T;
		}
		return T;
	}	
	SE3 FKinBody(const SE3& M, const ScrewList& Blist, const JVec& thetaList) {
		SE3 T = M;
		for (int i = 0; i < thetaList.size(); i++) {
			T = T * MatrixExp6(VecTose3(Blist.col(i)*thetaList(i)));
		}
		return T;
	}	
	Jacobian JacobianBody(const ScrewList& Blist, const JVec& thetaList) {
		Jacobian Jb = Blist;
		SE3 T = Identity4x4;
		Vector6d bListTemp;
		for (int i = thetaList.size() - 2; i >= 0; i--) {
			bListTemp << Blist.col(i + 1) * thetaList(i + 1);
			T = T * MatrixExp6(VecTose3(-1 * bListTemp));
			// std::cout << "array: " << sListTemp << std::endl;
			Jb.col(i) = Adjoint(T) * Blist.col(i);
		}
		return Jb;
	}
	PinvJacobian PinvJacobianBody(const ScrewList& Blist, const JVec& thetaList) {
		Jacobian Jb = JacobianBody(Blist, thetaList);
		
		double epsilon = 0.0000001;
		Eigen::JacobiSVD<Jacobian> svd(Jb ,Eigen::ComputeFullU | Eigen::ComputeFullV);
		double tolerance = epsilon * std::max(Jb.cols(), Jb.rows()) *svd.singularValues().array().abs()(0);
 		PinvJacobian pinv_relJb =  svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
		return pinv_relJb;
	}
	PinvJacobian PinvJacobianSpace(const ScrewList& Slist, const JVec& thetaList) {
		Jacobian Js = JacobianSpace(Slist, thetaList);
		
		double epsilon = 0.0000001;
		Eigen::JacobiSVD<Jacobian> svd(Js ,Eigen::ComputeFullU | Eigen::ComputeFullV);
		double tolerance = epsilon * std::max(Js.cols(), Js.rows()) *svd.singularValues().array().abs()(0);
 		PinvJacobian pinv_relJs =  svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
		return pinv_relJs;
	}	
	Jacobian JacobianSpace(const ScrewList& Slist, const JVec& thetaList) {
		Jacobian Js = Slist;	
		SE3 T = Identity4x4;
		Vector6d sListTemp;
		for (int i = 1; i < thetaList.size(); i++) {
			sListTemp << Slist.col(i - 1) * thetaList(i - 1);
			T = T * MatrixExp6(VecTose3(sListTemp));
			// std::cout << "array: " << sListTemp << std::endl;
			Js.col(i) = Adjoint(T) * Slist.col(i);
		}
		return Js;
	}	

	Vector6d getU1(const Jacobian& J){
		Matrix6d  JJT = J*J.transpose();
		Eigen::EigenSolver<Matrix6d> eigensolver;
		eigensolver.compute(JJT);	
	    Vector6d eigen_values = eigensolver.eigenvalues().real();
		Vector6d ret=eigensolver.eigenvectors().col(getMinCoeff(eigen_values)).real();
		return ret;

	}
	Matrix6d getP1(const Jacobian& J){
		Matrix6d ret;
		return ret;
	}
	Vector6d getUm(const Jacobian& J){
		Matrix6d  JJT = J*J.transpose();
		Eigen::EigenSolver<Matrix6d> eigensolver;
		eigensolver.compute(JJT);	
	    Vector6d eigen_values = eigensolver.eigenvalues().real();
		Vector6d ret=eigensolver.eigenvectors().col(getMaxCoeff(eigen_values)).real();
		return ret;			
	}
	Matrix6d getPm(const Jacobian& J){
		Matrix6d ret;
		return ret;
	}
    unsigned int getMaxCoeff(const Vector6d& vec){
		int idx = 0;
		double maxValue = -99999999;
		for(int i = 0 ;i<vec.size();i++){
			if(vec(i) > maxValue){
				maxValue = vec(i);
				idx = i;
			}
		}
		return idx;
	}
    unsigned int getMinCoeff(const Vector6d& vec){
		int idx = 0;
		double minValue = 99999999;
		for(int i = 0 ;i<vec.size();i++){
			if(vec(i) < minValue){
				minValue = vec(i);
				idx = i;
			}
		}
		return idx;
	}	
	Vector6d getRelativeU1(const RelativeJacobian& J){
		Matrix6d  JJT = J*J.transpose();
		Eigen::EigenSolver<Matrix6d> eigensolver;
		eigensolver.compute(JJT);	
	    Vector6d eigen_values = eigensolver.eigenvalues().real();
		Vector6d ret=eigensolver.eigenvectors().col(getMinCoeff(eigen_values)).real();
		return ret;

	}
	Matrix6d getRelativeP1(const RelativeJacobian& J){
		Matrix6d  JJT = J*J.transpose();
		Eigen::EigenSolver<Matrix6d> eigensolver;
		eigensolver.compute(JJT);	
	    Vector6d eigen_values = eigensolver.eigenvalues().real();
	    double lambda_1 =eigen_values(getMinCoeff(eigen_values));

		Vector6d u1 = getRelativeU1(J);
		Matrix6d ret=1.0/lambda_1*u1*u1.transpose();
		return ret;
	}
	Vector6d getRelativeUm(const RelativeJacobian& J){
		Matrix6d  JJT = J*J.transpose();
		Eigen::EigenSolver<Matrix6d> eigensolver;
		eigensolver.compute(JJT);	
	    Vector6d eigen_values = eigensolver.eigenvalues().real();
		Vector6d ret=eigensolver.eigenvectors().col(getMaxCoeff(eigen_values)).real();
		return ret;		
	}
	Matrix6d getRelativePm(const RelativeJacobian& J){
		Matrix6d  JJT = J*J.transpose();
		Eigen::EigenSolver<Matrix6d> eigensolver;
		eigensolver.compute(JJT);	
	    Vector6d eigen_values = eigensolver.eigenvalues().real();
	    double lambda_m =eigen_values(getMaxCoeff(eigen_values));


		Vector6d um = getRelativeUm(J);
		Matrix6d ret=1.0/lambda_m*um*um.transpose();
		return ret;
	}	


	double Manipulability_mu1(const Jacobian& J){
	   	double mu1 = sqrt(Manipulability_mu2(J));
		return mu1;
	}
	double Manipulability_mu2(const Jacobian& J){
		double mu2=0.0;
		Matrix6d  JJT = J*J.transpose();
		Eigen::EigenSolver<Matrix6d> eigensolver;
		eigensolver.compute(JJT);		
	    Vector6d eigen_values = eigensolver.eigenvalues().real();
	    double Lambda_m = eigen_values.maxCoeff(); //max_eigen_value
	    double Lambda_1 = eigen_values.minCoeff(); //min_eigen_value
	    mu2 = (Lambda_m/Lambda_1);
		return mu2;
	}
	double Manipulability_mu3(const Jacobian& J){
		Matrix6d  JJT = J*J.transpose();
		double mu3=sqrt(JJT.determinant());
		return mu3;
	}	
	RelativeJVec RelativeSpaceManipulability_dmu1(const Jacobian& J){
		RelativeJVec dmu1;
		
		return dmu1;
	}
	RelativeJVec RelativeSpaceManipulability_dmu2(const Jacobian& J){
		RelativeJVec dmu2;
		return dmu2;	
	}
	RelativeJVec RelativeSpaceManipulability_dmu3(const Jacobian& J){
		RelativeJVec dmu3;
		return dmu3;
	}
	RelativeJVec RelativeBodyManipulability_dmu1(const Jacobian& J){
		RelativeJVec dmu1;
		return dmu1;
	}
	RelativeJVec RelativeBodyManipulability_dmu2(const Jacobian& J){
		RelativeJVec dmu2;
		return dmu2;	
	}
	RelativeJVec RelativeBodyManipulability_dmu3(const Jacobian& J){
		RelativeJVec dmu3;
		return dmu3;
	}
	double RelativeManipulability_mu1(const RelativeJacobian& J){
	   	double mu1 = sqrt(RelativeManipulability_mu2(J));
		return mu1;
	}
	double RelativeManipulability_mu2(const RelativeJacobian& J){
		double mu2=0.0;
		Matrix6d  JJT = J*J.transpose();
		Eigen::EigenSolver<Matrix6d> eigensolver;
		eigensolver.compute(JJT);		
	    Vector6d eigen_values = eigensolver.eigenvalues().real();
	    double Lambda_m = eigen_values.maxCoeff(); //max_eigen_value
	    double Lambda_1 = eigen_values.minCoeff(); //min_eigen_value
	    mu2 = (Lambda_m/Lambda_1);
		return mu2;
	}
	double RelativeManipulability_mu3(const RelativeJacobian& J){
		Matrix6d  JJT = J*J.transpose();
		double mu3=sqrt(JJT.determinant());
		return mu3;
	}	
	JacobianVec VectorizeJacobian(const Jacobian& J){
		JacobianVec vecJ;
		for(int i = 0;i<JOINTNUM;i++){
			for(int j = 0;j<6;j++){
				vecJ(i*6 + j) = J(j,i);
			}
		}
		return vecJ;
	}
	DerivativeJacobianVec DerivativeVectorizeJacobianSpace(const ScrewList& Slist, const JVec& thetaList){
		DerivativeJacobianVec dvecJ = DerivativeJacobianVec::Zero();
		Jacobian Js = JacobianSpace(Slist,thetaList);
		for(int i = 1;i<JOINTNUM;i++){
			Vector6d Js_i = Js.col(i-1) ;
			for(int j= 0;j<i;j++){
				Vector6d Js_j = Js.col(j+1) ;
				Vector6d adJs_iJs_j = ad(Js_i)*Js_j;
				dvecJ.block(i*6,j,6,1) = adJs_iJs_j;
			}
		}

		return dvecJ;
	}
	DerivativeJacobianVec DerivativeVectorizeJacobianBody(const ScrewList& Blist, const JVec& thetaList){
		DerivativeJacobianVec dvecJ = DerivativeJacobianVec::Zero();
		Jacobian Jb = JacobianBody(Blist,thetaList);
		for(int i = 0;i<JOINTNUM-1;i++){
			Vector6d Jb_i = Jb.col(i) ;
			for(int j= i+1;j<JOINTNUM;j++){
				Vector6d Jb_j = Jb.col(j) ;
				Vector6d adJb_iJb_j = ad(Jb_i)*Jb_j;
				dvecJ.block(i*6,j,6,1) = adJb_iJb_j;
			}
		}
		return dvecJ;
	}
	DerivativeRelativeJacobianVec DerivativeVectorizeRelativeJacobianSpace(const RelativeScrewList& relSlist, const RelativeJVec& thetaList){
		DerivativeRelativeJacobianVec dvecJ = DerivativeRelativeJacobianVec::Zero();
		RelativeJacobian Js = RelativeJacobianSpace(relSlist,thetaList);
		for(int i = 0;i<RELATIVEJOINTNUM;i++){
			for(int j= 0;j<RELATIVEJOINTNUM;j++){
				if(i <j ){
					Vector6d Js_j = Js.col(j) ;
					Vector6d Js_i = Js.col(i) ;
					Vector6d adJs_iJs_j = ad(Js_i)*Js_j;
					dvecJ.block(6*j,i,6,1) = adJs_iJs_j;
				}else{
					dvecJ.block(6*j,i,6,1) = Vector6d::Zero();
				}
			}
		}

		return dvecJ;
	}
	DerivativeRelativeJacobianVec DerivativeVectorizeRelativeJacobianBody(const RelativeScrewList& relBlist, const RelativeJVec& thetaList){
		DerivativeRelativeJacobianVec dvecJ = DerivativeRelativeJacobianVec::Zero();
		RelativeJacobian Jb = RelativeJacobianBody(relBlist,thetaList);
		for(int i = 0;i<RELATIVEJOINTNUM;i++){
			for(int j= 0;j<RELATIVEJOINTNUM;j++){
				if(i >j ){
					Vector6d Jb_j = Jb.col(j) ;
					Vector6d Jb_i = Jb.col(i) ;
					Vector6d adJb_iJb_j = ad(Jb_i)*Jb_j;
					dvecJ.block(6*j,i,6,1) = adJb_iJb_j;
				}else{
					dvecJ.block(6*j,i,6,1) = Vector6d::Zero();
				}
			}
		}

		return dvecJ;
	}
	PinvJacobianVec VectorizePinvJacobian(const PinvJacobian& pinvJ){
		PinvJacobianVec vecJ;
		for(int i = 0;i<6;i++){
			for(int j = 0;j<JOINTNUM;j++){
				vecJ(i*6 + j) = pinvJ(j,i);
			}
		}
		return vecJ;
	}	
	RelativeJacobianVec VectorizeRelativeJacobian(const RelativeJacobian& J){
		RelativeJacobianVec vecJ;
		for(int i = 0;i<RELATIVEJOINTNUM;i++){
			for(int j = 0;j<6;j++){
				vecJ(i*6 + j) = J(j,i);
			}
		}
		return vecJ;
	}
	RelativePinvJacobianVec VectorizeRelativePinvJacobian(const RelativePinvJacobian& pinvJ){
		RelativePinvJacobianVec vecJ;
		for(int i = 0;i<6;i++){
			for(int j = 0;j<RELATIVEJOINTNUM;j++){
				vecJ(i*RELATIVEJOINTNUM + j) = pinvJ(j,i);
			}
		}
		return vecJ;
	}

	SE3 RelativeFKinBody(const SE3& M, const RelativeScrewList& relBlist, const RelativeJVec& thetaList) {
		SE3 T = M;
		for (int i = 0; i < thetaList.size(); i++) {
			T = T * MatrixExp6(VecTose3(relBlist.col(i)*thetaList(i)));
		}
		return T;
	}	
	SE3 RelativeFKinSpace(const SE3& M, const RelativeScrewList& relSlist, const RelativeJVec& thetaList) {
		SE3 T = M;
		for (int i = (thetaList.size() - 1); i > -1; i--) {
			T = MatrixExp6(VecTose3(relSlist.col(i)*thetaList(i))) * T;
		}
		return T;
	}	
	RelativeJacobian RelativeJacobianBody(const RelativeScrewList& relBlist, const RelativeJVec& thetaList) {
		RelativeJacobian Jb = relBlist;
		SE3 T = Identity4x4;
		Vector6d bListTemp;
		for (int i = thetaList.size() - 2; i >= 0; i--) {
			bListTemp << relBlist.col(i + 1) * thetaList(i + 1);
			T = T * MatrixExp6(VecTose3(-1 * bListTemp));
			Jb.col(i) = Adjoint(T) * relBlist.col(i);
		}
		return Jb;
	}
	RelativePinvJacobian RelativePinvJacobianBody(const RelativeScrewList& relBlist, const RelativeJVec& thetaList) {
		//https://gist.github.com/pshriwise/67c2ae78e5db3831da38390a8b2a209f

		RelativeJacobian relJb = RelativeJacobianBody(relBlist, thetaList);
		
		double epsilon = 0.0000001;
 		RelativePinvJacobian pinv_relJb = relJb.transpose()*(relJb*relJb.transpose()).inverse();
		return pinv_relJb;
	}
	RelativePinvJacobian RelativePinvJacobianSpace(const RelativeScrewList& relSlist, const RelativeJVec& thetaList) {
		RelativeJacobian relJs = RelativeJacobianSpace(relSlist, thetaList);
 		RelativePinvJacobian pinv_relJs = relJs.transpose()*(relJs*relJs.transpose()).inverse();
		return pinv_relJs;
	}	
	RelativeJacobian RelativeJacobianSpace(const RelativeScrewList& relSlist, const RelativeJVec& thetaList) {
		RelativeJacobian Js = relSlist;	
		SE3 T = Identity4x4;
		Vector6d sListTemp;
		for (int i = 1; i < thetaList.size(); i++) {
			sListTemp << relSlist.col(i - 1) * thetaList(i - 1);
			T = T * MatrixExp6(VecTose3(sListTemp));
			Js.col(i) = Adjoint(T) * relSlist.col(i);
		}
		return Js;
	}	
	RelativeJVec getRelativeJVec(const JVec& J1, const JVec& J2){
		RelativeJVec RelJ;
		RelJ(0) = -J1(5);
		RelJ(1) = -J1(4);
		RelJ(2) = -J1(3);
		RelJ(3) = -J1(2);
		RelJ(4) = -J1(1);
		RelJ(5) = -J1(0);

		RelJ(6) = J2(0);
		RelJ(7) = J2(1);
		RelJ(8) = J2(2);
		RelJ(9) = J2(3);
		RelJ(10)= J2(4);
		RelJ(11)= J2(5);
		return RelJ;
	}
	AdjMap Adjoint(const SE3& T) {
		SO3 R;
		Vector3d p;
		TransToRp(T,R,p);
		AdjMap ad_ret(6, 6);
		ad_ret = Zero6x6;
		ad_ret << R, Zero3x3,
			VecToso3(p) * R, R;
		return ad_ret;
	}
	Vector6d TwistSE3toSE3(const SE3 &Xd,const SE3 &X){
		Vector6d vec=se3ToVec(MatrixLog6(TransInv(X)*Xd));
		return vec;
	}
	so3 MatrixLog3(const SO3& R) {
		double acosinput = (R.trace() - 1) / 2.0;
		so3 m_ret =Zero3x3;	
		if (acosinput >= 1)
			return m_ret;
		else if (acosinput <= -1) {
			Eigen::Vector3d omg;
			if (!NearZero(1 + R(2, 2)))
				omg = (1.0 / std::sqrt(2 * (1 + R(2, 2))))*Eigen::Vector3d(R(0, 2), R(1, 2), 1 + R(2, 2));
			else if (!NearZero(1 + R(1, 1)))
				omg = (1.0 / std::sqrt(2 * (1 + R(1, 1))))*Eigen::Vector3d(R(0, 1), 1 + R(1, 1), R(2, 1));
			else
				omg = (1.0 / std::sqrt(2 * (1 + R(0, 0))))*Eigen::Vector3d(1 + R(0, 0), R(1, 0), R(2, 0));
			m_ret = VecToso3(M_PI * omg);
			return m_ret;
		}
		else {
			double theta = std::acos(acosinput);
			m_ret = theta / 2.0 / sin(theta)*(R - R.transpose());
			return m_ret;
		}
	}	
	se3 MatrixLog6(const SE3& T) {
		se3 m_ret(4, 4);
		SO3 R;
		Vector3d p;
		TransToRp(T,R,p);
		so3 omgmat = MatrixLog3(R);

		if (NearZero(omgmat.norm())) {
			m_ret << Zero3x3, p,
				0, 0, 0, 0;
		}
		else {
			double theta = std::acos((R.trace() - 1) / 2.0);
			Eigen::Matrix3d logExpand1 = Identity3x3 - omgmat / 2.0;
			Eigen::Matrix3d logExpand2 = (1.0 / theta - 1.0 / std::tan(theta / 2.0) / 2)*omgmat*omgmat / theta;
			Eigen::Matrix3d logExpand = logExpand1 + logExpand2;
			m_ret << omgmat, logExpand*p,
				0, 0, 0, 0;
		}
		return m_ret;
	}	
	SE3 RpToTrans(const SO3& R, const Vector3d& p) {
		SE3 m_ret(4, 4);
		m_ret << R, p,
			0, 0, 0, 1;
		return m_ret;
	}
	void TransToRp( const SE3& T, SO3& R, Vector3d& p) {
		R = T.block<3, 3>(0, 0);
		p = Vector3d(T(0, 3), T(1, 3), T(2, 3));
	}
	SO3 RotInv(const SO3& rotMatrix) {
		return rotMatrix.transpose();
	}
	Vector6d ScrewToAxis(Vector3d q, Vector3d s, double h) {
		Vector6d axis(6);
		axis.segment(0, 3) = s;
		axis.segment(3, 3) = q.cross(s) + (h * s);
		return axis;
	}
	Vector6d AxisAng6(const Eigen::VectorXd& expc6) {
		Vector6d v_ret(6);
		double theta = Eigen::Vector3d(expc6(0), expc6(1), expc6(2)).norm();
		if (NearZero(theta))
			theta = Eigen::Vector3d(expc6(3), expc6(4), expc6(5)).norm();
		v_ret << expc6 / theta, theta;
		return v_ret;
	}
	SO3 ProjectToSO3(const SO3& M) {
		Eigen::JacobiSVD<SO3> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
		SO3 R = svd.matrixU() * svd.matrixV().transpose();
		if (R.determinant() < 0)
			// In this case the result may be far from M; reverse sign of 3rd column
			R.col(2) *= -1;
		return R;
	}
	SE3 ProjectToSE3(const SE3& M) {
		SO3 R = M.block<3, 3>(0, 0);
		Vector3d t = M.block<3, 1>(0, 3);
		SE3 T = RpToTrans(ProjectToSO3(R), t);
		return T;
	}
	double DistanceToSO3(const SO3& M) {
		if (M.determinant() > 0)
			return (M.transpose() * M - Identity3x3).norm();
		else
			return 1.0e9;
	}
	double DistanceToSE3(const Eigen::Matrix4d& T) {
		Matrix3d matR = T.block<3, 3>(0, 0);
		if (matR.determinant() > 0) {
			Eigen::Matrix4d m_ret;
			m_ret << matR.transpose()*matR, Eigen::Vector3d::Zero(3),
				T.row(3);
			m_ret = m_ret - Identity4x4;
			return m_ret.norm();
		}
		else
			return 1.0e9;
	}
	bool IKinBody(const ScrewList& Blist, const SE3& M, const SE3& T,
		JVec& thetalist, double eomg, double ev) {
		int i = 0;
		int maxiterations = 20;
		SE3 Tfk = FKinBody(M, Blist, thetalist);
		SE3 Tdiff = TransInv(Tfk)*T;
		Vector6d Vb = se3ToVec(MatrixLog6(Tdiff));
		Vector3d angular(Vb(0), Vb(1), Vb(2));
		Vector3d linear(Vb(3), Vb(4), Vb(5));

		bool err = (angular.norm() > eomg || linear.norm() > ev);
		Jacobian Jb;
		while (err && i < maxiterations) {
			Jb = JacobianBody(Blist, thetalist);
			thetalist += Jb.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Vb);
			i += 1;
			// iterate
			Tfk = FKinBody(M, Blist, thetalist);
			Tdiff = TransInv(Tfk)*T;
			Vb = se3ToVec(MatrixLog6(Tdiff));
			angular = Eigen::Vector3d(Vb(0), Vb(1), Vb(2));
			linear = Eigen::Vector3d(Vb(3), Vb(4), Vb(5));
			err = (angular.norm() > eomg || linear.norm() > ev);
		}
		return !err;
	}
	bool RelativeIKinBody(const RelativeScrewList& relBlist, const SE3& relM, const SE3& relT,
		RelativeJVec& thetalist, double eomg, double ev) {
		int i = 0;
		int maxiterations = 20;
		SE3 relTfk = RelativeFKinBody(relM, relBlist, thetalist);
		SE3 relTdiff = TransInv(relTfk)*relT;
		Vector6d Vb = se3ToVec(MatrixLog6(relTdiff));
		Vector3d angular(Vb(0), Vb(1), Vb(2));
		Vector3d linear(Vb(3), Vb(4), Vb(5));

		bool err = (angular.norm() > eomg || linear.norm() > ev);
		RelativeJacobian Jb;
		while (err && i < maxiterations) {
			Jb = RelativeJacobianBody(relBlist, thetalist);
			thetalist += Jb.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Vb);
			i += 1;
			// iterate
			relTfk = RelativeFKinBody(relM, relBlist, thetalist);
			relTdiff = TransInv(relTfk)*relT;
			Vb = se3ToVec(MatrixLog6(relTdiff));
			angular = Eigen::Vector3d(Vb(0), Vb(1), Vb(2));
			linear = Eigen::Vector3d(Vb(3), Vb(4), Vb(5));
			err = (angular.norm() > eomg || linear.norm() > ev);
		}
		return !err;
	}	
	bool IKinSpace(const ScrewList& Slist, const SE3& M, const SE3& T,
		JVec& thetalist, double eomg, double ev) {
		int i = 0;
		int maxiterations = 20;
		SE3 Tfk = FKinSpace(M, Slist, thetalist);
		SE3 Tdiff = TransInv(Tfk)*T;
		Vector6d Vs = Adjoint(Tfk)*se3ToVec(MatrixLog6(Tdiff));
		Vector3d angular(Vs(0), Vs(1), Vs(2));
		Vector3d linear(Vs(3), Vs(4), Vs(5));

		bool err = (angular.norm() > eomg || linear.norm() > ev);
		Jacobian Js;
		while (err && i < maxiterations) {
			Js = JacobianSpace(Slist, thetalist);
			thetalist += Js.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Vs);
			i += 1;
			// iterate
			Tfk = FKinSpace(M, Slist, thetalist);
			Tdiff = TransInv(Tfk)*T;
			Vs = Adjoint(Tfk)*se3ToVec(MatrixLog6(Tdiff));
			angular = Eigen::Vector3d(Vs(0), Vs(1), Vs(2));
			linear = Eigen::Vector3d(Vs(3), Vs(4), Vs(5));
			err = (angular.norm() > eomg || linear.norm() > ev);
		}
		return !err;
	}	
	bool RelativeIKinSpace(const RelativeScrewList& relSlist, const SE3& relM, const SE3& relT,	RelativeJVec& thetalist, double eomg, double ev) {
		int i = 0;
		int maxiterations = 20;
		SE3 relTfk = RelativeFKinSpace(relM, relSlist, thetalist);
		SE3 relTdiff = TransInv(relTfk)*relT;
		Vector6d Vs = Adjoint(relTfk)*se3ToVec(MatrixLog6(relTdiff));
		Vector3d angular(Vs(0), Vs(1), Vs(2));
		Vector3d linear(Vs(3), Vs(4), Vs(5));

		bool err = (angular.norm() > eomg || linear.norm() > ev);
		RelativeJacobian Js;
		while (err && i < maxiterations) {
			Js = RelativeJacobianSpace(relSlist, thetalist);
			thetalist += Js.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Vs);
			i += 1;
			// iterate
			relTfk = RelativeFKinSpace(relM, relSlist, thetalist);
			relTdiff = TransInv(relTfk)*relT;
			Vs = Adjoint(relTfk)*se3ToVec(MatrixLog6(relTdiff));
			angular = Eigen::Vector3d(Vs(0), Vs(1), Vs(2));
			linear = Eigen::Vector3d(Vs(3), Vs(4), Vs(5));
			err = (angular.norm() > eomg || linear.norm() > ev);
		}
		return !err;
	}		

	adjMap ad(const Vector6d& V) {
		so3 omgmat = VecToso3(Eigen::Vector3d(V(0), V(1), V(2)));

		adjMap result(6, 6);
		result.topLeftCorner<3, 3>() = omgmat;
		result.topRightCorner<3, 3>() = Zero3x3;
		result.bottomLeftCorner<3, 3>() = VecToso3(Eigen::Vector3d(V(3), V(4), V(5)));
		result.bottomRightCorner<3, 3>() = omgmat;
		return result;
	}
	JVec InverseDynamics(const JVec& thetalist, const JVec& dthetalist, const JVec& ddthetalist,const Vector3d& g, const Vector6d& Ftip, const std::vector<SE3>& Mlist,const std::vector<Matrix6d>& Glist, const ScrewList& Slist) {
	    // the size of the lists
		int n = thetalist.size();

		SE3 Mi = Identity4x4;
		ScrewList Ai;
		std::vector<AdjMap> AdTi;
		for (int i = 0; i < n+1; i++) {
			AdTi.push_back(Zero6x6);
		}
		Eigen::Matrix<double, 6, JOINTNUM+1> Vi;    // velocity
		Eigen::Matrix<double, 6, JOINTNUM+1> Vdi;   // acceleration

		Vdi.block(3, 0, 3, 1) = - g;
		AdTi[n] =Adjoint(TransInv(Mlist[n]));
		Vector6d Fi = Ftip;

		JVec taulist;

		// forward pass
		for (int i = 0; i < n; i++) {
			Mi = Mi * Mlist[i];
			Ai.col(i) = Adjoint(TransInv(Mi))*Slist.col(i);

			AdTi[i] = Adjoint(MatrixExp6(VecTose3(Ai.col(i)*-thetalist(i)))
			          * TransInv(Mlist[i]));

			Vi.col(i+1) = AdTi[i] * Vi.col(i) + Ai.col(i) * dthetalist(i);
			Vdi.col(i+1) = AdTi[i] * Vdi.col(i) + Ai.col(i) * ddthetalist(i)
						   + ad(Vi.col(i+1)) * Ai.col(i) * dthetalist(i); // this index is different from book!
		}

		// backward pass
		for (int i = n-1; i >= 0; i--) {
			Fi = AdTi[i+1].transpose() * Fi + Glist[i] * Vdi.col(i+1)
			     - ad(Vi.col(i+1)).transpose() * (Glist[i] * Vi.col(i+1));
			taulist(i) = Fi.transpose() * Ai.col(i);
		}
		return taulist;	}
	JVec GravityForces(const JVec& thetalist, const Vector3d& g,
									const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const ScrewList& Slist) {
		JVec dummylist;
		Vector6d dummyForce;
		for(int i =0;i<JOINTNUM;i++){
			dummylist(i) = 0;
		}
		for(int i = 0;i<6;i++){
			dummyForce(i) = 0;
		}
		JVec grav = InverseDynamics(thetalist, dummylist, dummylist, g, dummyForce, Mlist, Glist, Slist);
		return grav;
	}
	MassMat MassMatrix(const JVec& thetalist, const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const ScrewList& Slist) {
		
		JVec dummylist;
		Vector3d dummyg=Zero3d;
		Vector6d dummyForce;
		for(int i =0;i<JOINTNUM;i++){
			dummylist(i) = 0;
		}
		for(int i = 0;i<6;i++){
			dummyForce(i) = 0;
		}
		MassMat M;
		for (int i = 0; i < JOINTNUM; i++) {
			JVec ddthetalist;
			for(int j =0;j<JOINTNUM;j++){
				ddthetalist(j) = 0;
			}
			ddthetalist(i) = 1;
			M.col(i) = InverseDynamics(thetalist, dummylist, ddthetalist, dummyg, dummyForce, Mlist, Glist, Slist);
		}
		return M;
	}
	JVec VelQuadraticForces(const JVec& thetalist, const JVec& dthetalist, const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const ScrewList& Slist) {
		JVec dummylist;
		Vector3d dummyg=Zero3d;
		Vector6d dummyForce;
		for(int i =0;i<JOINTNUM;i++){
			dummylist(i) = 0;
		}
		for(int i = 0;i<6;i++){
			dummyForce(i) = 0;
		}
		JVec c = InverseDynamics(thetalist, dthetalist, dummylist, dummyg, dummyForce, Mlist, Glist, Slist);
		return c;
	}
	Vector6d  EndEffectorForces(const JVec& thetalist, const Vector6d& Ftip,const std::vector<SE3>& Mlist, const std::vector<Matrix6d>& Glist, const ScrewList& Slist) {
		JVec dummylist;
		Vector3d dummyg=Zero3d;
		for(int i =0;i<JOINTNUM;i++){
			dummylist(i) = 0;
		}

		Vector6d JTFtip = InverseDynamics(thetalist, dummylist, dummylist, dummyg, Ftip, Mlist, Glist, Slist);
		return JTFtip;
	}
	JVec ForwardDynamics(const JVec& thetalist, const JVec& dthetalist, const JVec& taulist,
									const Vector3d& g, const Vector6d& Ftip, const std::vector<SE3>& Mlist,
									const std::vector<Matrix6d>& Glist, const ScrewList& Slist) {

		JVec totalForce = taulist - VelQuadraticForces(thetalist, dthetalist, Mlist, Glist, Slist)
                 							 - GravityForces(thetalist, g, Mlist, Glist, Slist)
                                             - EndEffectorForces(thetalist, Ftip, Mlist, Glist, Slist);

		MassMat M = MassMatrix(thetalist, Mlist, Glist, Slist);

		// Use LDLT since M is positive definite
        JVec ddthetalist = M.ldlt().solve(totalForce);

		return ddthetalist;
	}
	void EulerStep(JVec& thetalist, JVec& dthetalist, const JVec& ddthetalist, double dt) {
		thetalist += dthetalist * dt;
		dthetalist += ddthetalist * dt;
		return;
	}
	void EulerStep(RelativeJVec& thetalist, RelativeJVec& dthetalist, const RelativeJVec& ddthetalist, double dt) {
		thetalist += dthetalist * dt;
		dthetalist += ddthetalist * dt;
		return;
	}
	double CubicTimeScaling(double Tf, double t) {
		double timeratio = 1.0*t / Tf;
		double st = 3 * pow(timeratio, 2) - 2 * pow(timeratio, 3);
		return st;
	}
	double QuinticTimeScaling(double Tf, double t) {
		double timeratio = 1.0*t / Tf;
		double st = 10 * pow(timeratio, 3) - 15 * pow(timeratio, 4) + 6 * pow(timeratio, 5);
		return st;
	}
	std::vector<SE3> ScrewTrajectory(const SE3& Xstart, const SE3& Xend, double Tf, int N, int method) {
		double timegap = Tf / (N - 1);
		std::vector<SE3> traj(N);
		double st;
		for (int i = 0; i < N; ++i) {
			if (method == 3)
				st = CubicTimeScaling(Tf, timegap*i);
			else
				st = QuinticTimeScaling(Tf, timegap*i);
			se3 Ttemp = MatrixLog6(TransInv(Xstart)*Xend);
			traj.at(i) = Xstart * MatrixExp6(Ttemp*st);
		}
		return traj;
	}
	std::vector<SE3> CartesianTrajectory(const SE3& Xstart, const SE3& Xend, double Tf, int N, int method) {
		double timegap = Tf / (N - 1);
		std::vector<SE3> traj(N);
		SO3 Rstart;
		Vector3d pstart;
		TransToRp(Xstart,Rstart,pstart);
		SO3 Rend;
		Vector3d pend;
		TransToRp(Xend,Rend,pend);
		double st;
		for (int i = 0; i < N; ++i) {
			if (method == 3)
				st = CubicTimeScaling(Tf, timegap*i);
			else
				st = QuinticTimeScaling(Tf, timegap*i);
			SO3 Ri = Rstart * MatrixExp3(MatrixLog3(Rstart.transpose() * Rend)*st);
			Vector3d pi = st*pend + (1 - st)*pstart;
			SE3 traji(4, 4);
			traji << Ri, pi,
				0, 0, 0, 1;
			traj.at(i) = traji;
		}
		return traj;
	}

     void printMatrixJVec (const JVec & Mat){ Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
     	 cout<<Mat.format(OctaveFmt)<<endl;}
     void printMatrixRelativeJVec (const RelativeJVec & Mat){ Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
     	 cout<<Mat.format(OctaveFmt)<<endl;}
     void printMatrixSE3 (const SE3 & Mat){ Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
     	 cout<<Mat.format(OctaveFmt)<<endl;}
     void printMatrixSO3 (const SO3 & Mat){ Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
     	 cout<<Mat.format(OctaveFmt)<<endl;}
     void printMatrixse3 (const se3 & Mat){ Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
     	 cout<<Mat.format(OctaveFmt)<<endl;}
     void printMatrixso3 (const so3 & Mat){ Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
     	 cout<<Mat.format(OctaveFmt)<<endl;}
     void printMatrixScrewList (const ScrewList & Mat){ Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
     	 cout<<Mat.format(OctaveFmt)<<endl;}
     void printMatrixJacobian (const Jacobian & Mat){ Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
     	 cout<<Mat.format(OctaveFmt)<<endl;}
     void printMatrixPinvJacobian (const PinvJacobian & Mat){ Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
     	 cout<<Mat.format(OctaveFmt)<<endl;}
     void printMatrixJacobianVec (const JacobianVec & Mat){ Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
     	 cout<<Mat.format(OctaveFmt)<<endl;}
     void printMatrixPinvJacobianVec (const PinvJacobianVec & Mat){ Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
     	 cout<<Mat.format(OctaveFmt)<<endl;}
     void printMatrixDerivativeJacobianVec (const DerivativeJacobianVec & Mat){ Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
     	 cout<<Mat.format(OctaveFmt)<<endl;}
     void printMatrixRelativeScrewList (const RelativeScrewList & Mat){ Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
     	 cout<<Mat.format(OctaveFmt)<<endl;}
     void printMatrixRelativeJacobian (const RelativeJacobian & Mat){ Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
     	 cout<<Mat.format(OctaveFmt)<<endl;}
     void printMatrixRelativePinvJacobian (const RelativePinvJacobian & Mat){ Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
     	 cout<<Mat.format(OctaveFmt)<<endl;}
     void printMatrixRelativeJacobianVec (const RelativeJacobianVec & Mat){ Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
     	 cout<<Mat.format(OctaveFmt)<<endl;}
     void printMatrixRelativePinvJacobianVec (const RelativePinvJacobianVec & Mat){ Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
     	 cout<<Mat.format(OctaveFmt)<<endl;}
     void printMatrixDerivativeRelativeJacobianVec (const DerivativeRelativeJacobianVec & Mat){ Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
     	 cout<<Mat.format(OctaveFmt)<<endl;}
     void printMatrixMassMat (const MassMat & Mat){ Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
     	 cout<<Mat.format(OctaveFmt)<<endl;}
     void printMatrixCoriolisMat (const CoriolisMat & Mat){ Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
     	 cout<<Mat.format(OctaveFmt)<<endl;}
     void printMatrixVector6d (const Vector6d & Mat){ Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
     	 cout<<Mat.format(OctaveFmt)<<endl;}
     void printMatrixVector3d (const Vector3d & Mat){ Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
     	 cout<<Mat.format(OctaveFmt)<<endl;}
     void printMatrixVector4d (const Vector4d & Mat){ Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
     	 cout<<Mat.format(OctaveFmt)<<endl;}
     void printMatrixMatrix6d (const Matrix6d & Mat){ Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
     	 cout<<Mat.format(OctaveFmt)<<endl;}
     void printMatrixMatrix3d (const Matrix3d & Mat){ Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
     	 cout<<Mat.format(OctaveFmt)<<endl;}

}
