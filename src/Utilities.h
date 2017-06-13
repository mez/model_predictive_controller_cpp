//
// Created by Mez Gebre on 6/2/17.
//

#ifndef MPC_UTILITIES_H
#define MPC_UTILITIES_H

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include <string>

namespace utilities {

  // Checks if the SocketIO event has JSON data.
  // If there is data the JSON object in string format will be returned,
  // else the empty string "" will be returned.
  std::string hasData(std::string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.rfind("}]");
    if (found_null != std::string::npos) {
      return "";
    } else if (b1 != std::string::npos && b2 != std::string::npos) {
      return s.substr(b1, b2 - b1 + 2);
    }
    return "";
  }

  // For converting back and forth between radians and degrees.
  constexpr double pi() { return M_PI; }
  double deg2rad(double x) { return x * pi() / 180; }
  double rad2deg(double x) { return x * 180 / pi(); }

  // Evaluate a polynomial.
  double polyeval(Eigen::VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
      result += coeffs[i] * pow(x, i);
    }
    return result;
  }

  // Fit a polynomial.
  // Adapted from
  // https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
  Eigen::VectorXd polyfit(Eigen::VectorXd &xvals, Eigen::VectorXd &yvals, int order) {
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++) {
      A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++) {
      for (int i = 0; i < order; i++) {
        A(j, i + 1) = A(j, i) * xvals(j);
      }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
  }

  /**
   *  https://forum.kde.org/viewtopic.php?f=74&t=94839#p194926 shows how to use Eigen::Map
   * @param px
   * @param py
   * @param psi
   * @param ptsx
   * @param ptsy
   * @param x_way_points
   * @param y_way_points
   */
  void transformGlobalWaypointsToCarCoord(double px,
                           double py,
                           double psi,
                           std::vector<double> &ptsx,
                           std::vector<double> &ptsy,
                           Eigen::VectorXd &x_way_points,
                           Eigen::VectorXd &y_way_points) {

    for (int i = 0; i < ptsx.size(); ++i) {
      const double dx = ptsx[i]-px;
      const double dy = ptsy[i]-py;

      ptsx[i] =  dx*cos(psi) + dy*sin(psi);
      ptsy[i] = -dx*sin(psi) + dy*cos(psi);
    }

    double *ptsx_pointer = &ptsx[0];
    x_way_points = Eigen::Map<Eigen::VectorXd>(ptsx_pointer, ptsx.size());

    double *ptsy_pointer = &ptsy[0];
    y_way_points = Eigen::Map<Eigen::VectorXd>(ptsy_pointer, ptsy.size());
  }
}
#endif //MPC_UTILITIES_H
