#include "cost.h"
#include <cmath>
#include <functional>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include "vehicle.h"
#include <iostream>

using std::string;
using std::vector;

/**
 * TODO: change weights for cost functions.
 */
const float REACH_GOAL = pow(10,3);
const float EFFICIENCY = pow(10,3);
const float ACC_COST = 0;//pow(10,3);
const float LANE_TR_COST = 2*pow(10,3);

// Here we have provided two possible suggestions for cost functions, but feel
//   free to use your own! The weighted cost over all cost functions is computed
//   in calculate_cost. The data from get_helper_data will be very useful in
//   your implementation of the cost functions below. Please see get_helper_data
//   for details on how the helper data is computed.

float goal_distance_cost(const Vehicle &vehicle,
                         const vector<Vehicle> &trajectory,
                         const map<int, vector<Vehicle>> &predictions,
                         map<string, float> &data) {
  // Cost increases based on distance of intended lane (for planning a lane
  //   change) and final lane of trajectory.
  // Cost of being out of goal lane also becomes larger as vehicle approaches
  //   goal distance.
  // This function is very similar to what you have already implemented in the
  //   "Implement a Cost Function in C++" quiz.
  float cost;
  float distance = data["distance_to_goal"];
  if (distance > 0 && data["intended_lane"] >= 0 && data["intended_lane"] <= 2) {
    cost = 2 - 2*exp(-(abs(2.0*vehicle.goal_lane - data["intended_lane"]
         - data["final_lane"]) / distance));
  } else {
    cost = 2;
  }

  return cost;
}

float inefficiency_cost(const Vehicle &vehicle,
                        const vector<Vehicle> &trajectory,
                        const map<int, vector<Vehicle>> &predictions,
                        map<string, float> &data) {
  // Cost becomes higher for trajectories with intended lane and final lane
  //   that have traffic slower than vehicle's target speed.
  // You can use the lane_speed function to determine the speed for a lane.
  // This function is very similar to what you have already implemented in
  //   the "Implement a Second Cost Function in C++" quiz.
  float proposed_speed_intended = lane_speed(predictions, data["intended_lane"],vehicle);
  if (proposed_speed_intended < 0) {
    //std::cout<<trajectory[1].state<<"========================================no veh\n";
    proposed_speed_intended = vehicle.target_speed;
    //std::cout<<"Target speed "<<vehicle.target_speed;
  }

  float proposed_speed_final = lane_speed(predictions, data["final_lane"],vehicle);
  if (proposed_speed_final < 0) {
    proposed_speed_final = vehicle.target_speed;
  }

  float cost;
  if (data["intended_lane"] >= 0 && data["intended_lane"] <= 2) {
  cost = (2.0*vehicle.target_speed - proposed_speed_intended
             - proposed_speed_final)/vehicle.target_speed;}
  else
 {cost = 2.0;
 }

  return cost;
}


float acceleration_cost(const Vehicle &vehicle,
                         const vector<Vehicle> &trajectory,
                         const map<int, vector<Vehicle>> &predictions,
                         map<string, float> &data)
{
/*
float proposed_speed_intended = lane_speed(predictions, data["intended_lane"]);
if (proposed_speed_intended < 0) {
    proposed_speed_intended = vehicle.target_speed;
    //std::cout<<"Target speed "<<vehicle.target_speed;
  }

  float proposed_speed_final = lane_speed(predictions, data["final_lane"]);
  if (proposed_speed_final < 0) {
    proposed_speed_final = vehicle.target_speed;
  }
 float cost = 0;
 cost = (abs(proposed_speed_intended-vehicle.v)/9.0+abs(proposed_speed_final-vehicle.v)/9)/2;
*/
float cost = 2.0;
 if (data["intended_lane"] >= 0 && data["intended_lane"] <= 2)
 {
 cost = 2*abs(trajectory[1].v - vehicle.v)/vehicle.max_acceleration;
 }



  return cost;

}




float lane_traffic_cost(const Vehicle &vehicle,
                         const vector<Vehicle> &trajectory,
                         const map<int, vector<Vehicle>> &predictions,
                         map<string, float> &data)
{
  float cost = 2.0;
  double dist_intended = lane_free_dist(predictions, data["intended_lane"], vehicle);
  double dist_final = lane_free_dist(predictions, data["final_lane"], vehicle);

 if (data["intended_lane"] >= 0 && data["intended_lane"] <= 2)
 {
  cost = float((2.0*200 - dist_intended - dist_final)/200);
  }

  return cost;
  }

float lane_speed(const map<int, vector<Vehicle>> &predictions, int lane, const Vehicle &vehicle) {
  // All non ego vehicles in a lane have the same speed, so to get the speed
  //   limit for a lane, we can just find one vehicle in that lane.
 int min_s = vehicle.goal_s;
  bool found_vehicle = false;
  Vehicle temp_vehicle;
  double v = 0.0;
  for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin();
       it != predictions.end(); ++it) {
    temp_vehicle = it->second[0];
    int key = it->first;
    if (temp_vehicle.lane == lane && temp_vehicle.s< vehicle.s
        && temp_vehicle.s < min_s&&key != -1) {
      min_s = temp_vehicle.s;
      v = temp_vehicle.v;
      found_vehicle = true;
    }
  }
  // Found no vehicle in the lane
  if(found_vehicle == false)
  {
  return -1.0;
  }
  else
  {
  return v;
  }
}

float lane_free_dist(const map<int, vector<Vehicle>> &predictions, int lane, const Vehicle &vehicle)
{
  int max_s = -1;
  bool found_vehicle = false;
  Vehicle temp_vehicle;
  double dist = 0.0;
  for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin();
       it != predictions.end(); ++it) {
    temp_vehicle = it->second[0];
    int key = it->first;
    if (temp_vehicle.lane == lane && temp_vehicle.s > vehicle.s
        && temp_vehicle.s > max_s && key != -1) {
      max_s = temp_vehicle.s;
      dist = temp_vehicle.s - vehicle.s ;
      found_vehicle = true;
    }
  }
  // Found no vehicle in the lane
  if(found_vehicle == false)
  {
  return 200;
  }
  else
  {
  //std::cout<<lane << " "<<"Dist"<<dist<<"\n";
  return dist;
  }

}

float calculate_cost(const Vehicle &vehicle,
                     const map<int, vector<Vehicle>> &predictions,
                     const vector<Vehicle> &trajectory) {
  // Sum weighted cost functions to get total cost for trajectory.
  map<string, float> trajectory_data = get_helper_data(vehicle, trajectory,
                                                       predictions);
  float cost = 0.0;
//std::cout<<"intended_lane "<<trajectory_data["intended_lane"]<<" final_lane "<<trajectory_data["final_lane"]<<" distance_to_goal "<<trajectory_data["distance_to_goal"]<<"\n";
  // Add additional cost functions here.
  vector<std::function<float(const Vehicle &, const vector<Vehicle> &,
                             const map<int, vector<Vehicle>> &,
                             map<string, float> &)
    >> cf_list = {goal_distance_cost, inefficiency_cost, acceleration_cost,lane_traffic_cost};
  vector<float> weight_list = {REACH_GOAL, EFFICIENCY,ACC_COST,LANE_TR_COST};

  for (int i = 0; i < cf_list.size(); ++i) {
    float new_cost = weight_list[i]*cf_list[i](vehicle, trajectory, predictions,
                                               trajectory_data);
    //std::cout<<i<<" "<<new_cost<<"\n";
    cost += new_cost;
  }

  return cost;
}

map<string, float> get_helper_data(const Vehicle &vehicle,
                                   const vector<Vehicle> &trajectory,
                                   const map<int, vector<Vehicle>> &predictions) {
  // Generate helper data to use in cost functions:
  // intended_lane: the current lane +/- 1 if vehicle is planning or
  //   executing a lane change.
  // final_lane: the lane of the vehicle at the end of the trajectory.
  // distance_to_goal: the distance of the vehicle to the goal.

  // Note that intended_lane and final_lane are both included to help
  //   differentiate between planning and executing a lane change in the
  //   cost functions.
  map<string, float> trajectory_data;
  Vehicle trajectory_last = trajectory[1];
  float intended_lane;

  if (trajectory_last.state.compare("PLCL") == 0) {
    intended_lane = trajectory_last.lane - 1;
  } else if (trajectory_last.state.compare("PLCR") == 0) {
    intended_lane = trajectory_last.lane + 1;
  } else {
    intended_lane = trajectory_last.lane;
  }

  float distance_to_goal = vehicle.goal_s - trajectory_last.s;
  float final_lane = trajectory_last.lane;
  trajectory_data["intended_lane"] = intended_lane;
  trajectory_data["final_lane"] = final_lane;
  trajectory_data["distance_to_goal"] = distance_to_goal;

  return trajectory_data;
}
