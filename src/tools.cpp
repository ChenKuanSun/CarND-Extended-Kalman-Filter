#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

//initialize paramater
Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;
	if (estimations.size() == 0) {
		cout << "ERROR - The estimations vector is empty!" << endl;
		return rmse;

	if (estimations.size() == 0) 
		cout << "ERROR - The estimations vector is empty!" << endl;
		return rmse;
	}

	if (ground_truth.size() == 0) {
		cout << "ERROR - The ground-truth vector is empty!" << endl;
		return rmse;
	}

	float t = estimations.size();
	if (t != ground_truth.size()) {
		cout << "ERROR - The ground-truth and estimations vectors must have the same size!" << endl;
		return rmse;
	}

	for (float i = 0; i < estimations.size(); ++i) {
		VectorXd dif = estimations[i] - ground_truth[i];
		dif = dif.array() * dif.array();
		rmse += dif;
	}
	rmse /= t;
	rmse = rmse.array().sqrt();;
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
	MatrixXd Hj(3, 4);

	if (x_state.size() != 4) {
		cout << "ERROR - The state vector must size 4!" << endl;
		return Hj;
	}

	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);


	//Calculate the Jacobian 
	float rho = px * px + py * py;
	float rho2 = sqrt(rho); //sqrtrho
	float rho3 = rho2 * rho;//sqrt rho * rho
	//4*3
	Hj <<		px / rho2,						 py / rho2,				 0,					0,
					-py / rho,						px / rho,				 0,					0,
	py*(vx*py - vy * px) / rho3, px*(vy*px - vx * py) / rho3,		px / rho2,			 py / rho2;
	
	return Hj;
}
