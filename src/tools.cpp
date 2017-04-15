#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}


VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
		const vector<VectorXd> &ground_truth){

  /**
  DONE:
  * Calculate the RMSE here.
  Using the RMSE calculation from lectures...
  */

	VectorXd rmse(4); 
	rmse << 0,0,0,0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if(estimations.size() != ground_truth.size()
			|| estimations.size() == 0){
		std::cout << "Invalid estimation or ground_truth data" << std::endl;
		return rmse;
	}

	//accumulate squared residuals
	for(unsigned int i=0; i < estimations.size(); ++i){

		VectorXd residual = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	//calculate the mean
	rmse = rmse/estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}



MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  DONE:
    * Calculate a Jacobian here.
    Use code provided from the lectures...
    For RADAR measurements the Jacobian must be calculated
    For LIDAR measurements the Jacobian isn't required and this function won't be called
  */

	MatrixXd Hj = MatrixXd::Zero(3,4);
	// Recover the state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//pre-compute a set of terms to avoid repeated calculation
	float c1 = px*px+py*py;
	float c2 = sqrt(c1);
	float c3 = (c1*c2);       // So c3 is power 3/2

	//check division by zero
	if(fabs(c1) > 0.00001){		
	  //compute the Jacobian matrix
	  Hj << 	(px/c2), (py/c2), 0, 0,
		  		-(py/c1), (px/c1), 0, 0,
		  		py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

	  return Hj;
	}

  // If we get here the Jacobian can't be created due to potential division by zero
	std::cout << "CalculateJacobian() - division by zero. Returning empty Jacobian" << std::endl;
	return Hj;
}
