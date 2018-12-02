#include <iostream>
#include "tools.h"
#include <math.h>

using namespace std;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
  VectorXd rmse(4);
  rmse<< 0, 0, 0, 0;
  
  if(estimations.size() == 0 || estimations.size() != ground_truth.size()) 
  {
    cout << "Invalid estimation or ground truth data" << endl;
    return rmse;
  }
  
  // accumulate squared residuals
  
  for (unsigned int i = 0; i<estimations.size(); ++i)
  {
    cout <<estimations[i] << endl;
    VectorXd residual = estimations[i] -ground_truth[i];
    residual = residual.array()*residual.array();
    
    rmse += residual;
  }
  
  // calculate the mean
  rmse = rmse/estimations.size();
  
  // calculate the squared root
  rmse = rmse.array().sqrt();
  
  // return the result
  
  return rmse;
  
  
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3,4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  
  // pre-compute a set of terms to avoid repeated calculations
  float radius_2 = px*px + py*py;
  float radius = sqrt(radius_2);
  float radius_3 = radius_2*radius;
  
  // check division by zero 
  
  if(fabs(radius_2)<0.0001) 
  {
    cout << "CalculateJacobian() - Error - Division by Zero" << endl;
    return Hj;
  }
  
  // compute the Jacobian matrix
  Hj <<(px/radius), (py/radius), 0, 0,
  	   -(py/radius_2), (px/radius_2), 0, 0,
  	   py*(vx*py-vy*px)/radius_3, px*(px*vy-py*vx)/radius_3, px/radius, py/radius;
  
  return Hj;
  
  
}
