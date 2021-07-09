#include <random>
#include <vector>
#include "ptg.h"
#include "Eigen-3.3/Eigen/Dense"


/**
 num_particles = 100;  // TODO: Set the number of particles
  std::default_random_engine gen;
  std::normal_distribution<double> dist_x(x, std[0]);
  std::normal_distribution<double> dist_y(y, std[1]);
  std::normal_distribution<double> dist_theta(theta, std[2]);

  for (int i = 0; i < num_particles; ++i) {
    Particle p;
    p.id = i;
    p.x =dist_x(gen);
    p.y = dist_y(gen);*/
using Eigen::MatrixXd;
using Eigen::VectorXd;
vector<vector<double>> ptg(vector<double> &start_s,vector<double> &start_d,vector<double> &target_s,vector<double> &target_d, double &T,map<int ,vector<Vehicle>> &predictions)
{
  vector<vector<double>> frenet_points;
  vector<double> alpha_s;
  vector<double> alpha_d;
  alpha_s=JMT(start_s, target_s, T);
  alpha_d=JMT(start_d, target_d, T);
  double t = 0.0;

  for(double t=0;t<=T;t++)
  {
    vector<double> coord;
    double s=0.0;
    double d=0.0;
    s=evaluate_equation(alpha_s,t);
    d=evaluate_equation(alpha_d,t);
    /*if(s > target_s[0])
    {
    return frenet_points;
    }*/

    coord.push_back(s);
    coord.push_back(d);
    frenet_points.push_back(coord);
  }

  return frenet_points;

}

double evaluate_equation(vector<double> &coefficients, double &t)
{
  double total = 0.0;
  for(int i = 0;i<coefficients.size();i++)
  {
    total += coefficients[i]*pow(t,i);
  }
  return total;
}


vector<double> JMT(vector<double> &start, vector<double> &end, double T) {
  /**
   *
   * @param start - the vehicles start location given as a length three array
   *   corresponding to initial values of [s, s_dot, s_double_dot]
   * @param end - the desired end state for vehicle. Like "start" this is a
   *   length three array.
   * @param T - The duration, in seconds, over which this maneuver should occur.
   *
   * @output an array of length 6, each value corresponding to a coefficent in
   *   the polynomial:
   *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   *
   * EXAMPLE
   *   > JMT([0, 10, 0], [10, 10, 0], 1)
   *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */
  MatrixXd t(3,3) ;
  MatrixXd s(3,1) ;
  MatrixXd alpha(3,1);
  t << pow(T,3),pow(T,4),pow(T,5),
       3*pow(T,2),4*pow(T,3),5*pow(T,4),
       6*T,12*pow(T,2),20*pow(T,3);
  s << end[0]-(start[0]+start[1]*T+start[2]*T*T/2),
       end[1]-(start[1]+start[2]*T),
       end[2]-start[2];
  alpha = t.inverse()*s;
  return {start[0],start[1],0.5*start[2],alpha.data()[0],alpha.data()[1],alpha.data()[2]};
}
