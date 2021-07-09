#include <vector>
#include "vehicle.h"


using std::vector;
vector<vector<double>> ptg(vector<double> &start_s,vector<double> &start_d,vector<double> &target_s,vector<double> &target_d, double &T,map<int ,vector<Vehicle>> &predictions);

vector<double> JMT(vector<double> &start,vector<double> &end,double T);

double evaluate_equation(vector<double> &coefficients, double &t);
