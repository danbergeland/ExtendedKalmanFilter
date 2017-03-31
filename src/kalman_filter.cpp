#include "kalman_filter.h"
#include <iostream>
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

MatrixXd KalmanFilter::CalculateJacobian() {
    
    MatrixXd Hj(3,4);
  //recover state parameters
	float px = x_(0);
	float py = x_(1);
	float vx = x_(2);
	float vy = x_(3);

    float px2 = px*px;
    float py2 = py*py;
    float hyp = sqrt(px2+py2);
	//check division by zero
	if(hyp > .0001)
	{
	  	//compute the Jacobian matrix  
	  	Hj <<  px/hyp,          py/hyp,     0,      0,
	  	       -py/(px2+py2), px/(px2+py2), 0,      0,
	  	       py*(py*vx-px*vy)/pow(px2+py2,1.5),px*(vy*px-vx*py)/pow(px2+py2,1.5),px/hyp,py/hyp;
	}
	return Hj;
}

void KalmanFilter::Predict() {
  /**
  */
    //assume x = Fx + u, where u = 0,0,0,0 (u is omitted)
    x_ = F_*x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_*P_*Ft+Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
    * update the state by using Kalman Filter equations
  */
    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

	//new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
    VectorXd hx = VectorXd(3);
    float px = x_(0);
    float py = x_(1);
    float vx = x_(2);
    float vy = x_(3);
    float hyp = sqrt(px*px+py*py);
    //avoid divide by 0 in atan function
    if(px>.0001||px<-.0001)
    {
        hx << hyp, atan2(py,px), (px*vx+py*vy)/hyp;
    }

    MatrixXd Hj = CalculateJacobian();
    //VectorXd z_pred = Hj * x_;
    VectorXd y = z - hx;
    MatrixXd Ht = Hj.transpose();
    MatrixXd S = Hj * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

	//new estimate
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * Hj) * P_;

}
