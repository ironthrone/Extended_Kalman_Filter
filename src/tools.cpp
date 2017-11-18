#include <iostream>
#include "tools.h"
#include <cmath>

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
  rmse << 0, 0, 0, 0;
  VectorXd se(4);
  se << 0, 0, 0, 0;
  for (int i = 0; i < estimations.size(); ++i) {
    VectorXd different = (estimations[i] - ground_truth[i]);
    different = different.array() * different.array();
    se += different;
  }

  rmse = (se / estimations.size());
  rmse = rmse.array().sqrt();

  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd &x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3, 4);
  //recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //check division by zero
  double squareSum = px * px + py * py;
  if (squareSum < 0.00001) {
    //TODO how to deal with this situation
    cout << "can not divide by zero";
    Hj << 0, 0, 0, 0,
      0, 0, 0, 0,
      0, 0, 0, 0;
  } else {
    Hj << px / sqrt(squareSum), py / sqrt(squareSum), 0, 0,
      -1 * py / squareSum, px / squareSum, 0, 0,
      py * (vx * py - vy * px) / pow(squareSum, 1.5), px * (vy * px - vx * py) / pow(squareSum, 1.5)
      , px / sqrt(squareSum), py / sqrt(squareSum);
  }
  std::cout << Hj << endl;
  return Hj;

}

VectorXd Tools::ConvertRadarMeasure(const VectorXd &radar_measure) {
  VectorXd x(4);
  double rho = radar_measure(0);
  double phi = radar_measure(1);
  x(0) = rho * cos(phi);
  x(1) = rho * sin(phi);
  x(2) = 0;
  x(3) = 0;
  return x;
}
