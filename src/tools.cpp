#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    VectorXd rmse(4);
    rmse << 0,0,0,0;

    if(estimations.size()>0 && estimations.size()==ground_truth.size())
    {
        //accumulate squared residuals
            for(int i=0; i < estimations.size(); ++i){
                // ... your code here
                VectorXd res = estimations[i]-ground_truth[i];
                res = res.array()*res.array();
                rmse += res;
            
            }
        //calculate the mean
        rmse = rmse/estimations.size();
        //calculate the squared root
        rmse = rmse.array().sqrt();

    }
	//return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {

      MatrixXd Hj(3,4);
  //recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//TODO: YOUR CODE HERE 
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
