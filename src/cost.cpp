/*
 * cost.cpp
 *
 *  Created on: Apr 6, 2020
 *      Author: ahmadsv
 */
#include "cost.h"
#include <cmath>
#include <functional>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include "vehicle.h"
#include "parameters.h"
#include "iostream"

using std::string;
using std::vector;


/**
 * TODO: change weights for cost functions.
 */
const double EFFICIENCY = pow(10.0,5.0);

// Here we have provided two possible suggestions for cost functions, but feel
//   free to use your own! The weighted cost over all cost functions is computed
//   in calculate_cost. The data from get_helper_data will be very useful in
//   your implementation of the cost functions below. Please see get_helper_data
//   for details on how the helper data is computed.

/*float goal_distance_cost(const Vehicle &vehicle,
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
  if (distance > 0) {
    cost = 1 - 2*exp(-(abs(2.0*vehicle.goal_lane - data["intended_lane"]
         - data["final_lane"]) / distance));
  } else {
    cost = 1;
  }

  return cost;
}*/

double inefficiency_cost(const Vehicle &vehicle,
                        const vector<Vehicle> &trajectory,
                        const map<int, Vehicle> &otherCars,
                        map<string, double> &data) {
  // Cost becomes higher for trajectories with intended lane and final lane
  //   that have traffic slower than vehicle's target speed.
  // You can use the lane_speed function to determine the speed for a lane.
  // This function is very similar to what you have already implemented in
  //   the "Implement a Second Cost Function in C++" quiz.
  double proposed_speed_intended = data["intended_speed"];
  if (proposed_speed_intended < 0) {
    proposed_speed_intended = MAX_SPEED_MS;
  }

  double proposed_speed_final = data["final_speed"];
  if (proposed_speed_final < 0) {
    proposed_speed_final = MAX_SPEED_MS;
  }

  std::cout << "intended_speed " << proposed_speed_intended << " final speed " << proposed_speed_final << std::endl;

  double cost = (2.0*MAX_SPEED_MS - proposed_speed_intended
             - proposed_speed_final)/MAX_SPEED_MS;

  return cost;
}



double calculate_cost(const Vehicle &egoVehicle,
                     const map<int, Vehicle> &otherCars,
                     const vector<Vehicle> &trajectory) {
  // Sum weighted cost functions to get total cost for trajectory.
  map<string, double> trajectory_data = get_helper_data(egoVehicle, trajectory,
		  otherCars);
  double cost = 0.0;

  // Add additional cost functions here.
  vector<std::function<double(const Vehicle &, const vector<Vehicle> &,
                             const map<int, Vehicle> &,
                             map<string, double> &)
    >> cf_list = { inefficiency_cost};//goal_distance_cost,
  vector<double> weight_list = {EFFICIENCY};

  for (int i = 0; i < cf_list.size(); ++i) {
    double new_cost = weight_list[i]*cf_list[i](egoVehicle, trajectory, otherCars,
                                               trajectory_data);
    cost += new_cost;
  }

  return cost;
}

map<string, double> get_helper_data(const Vehicle &vehicle,
                                   const vector<Vehicle> &trajectory,
                                   const map<int, Vehicle> &otherCars) {
  // Generate helper data to use in cost functions:
  // intended_lane: the current lane +/- 1 if vehicle is planning or
  //   executing a lane change.
  // final_lane: the lane of the vehicle at the end of the trajectory.
  // distance_to_goal: the distance of the vehicle to the goal.

  // Note that intended_lane and final_lane are both included to help
  //   differentiate between planning and executing a lane change in the
  //   cost functions.
  map<string, double> trajectory_data;
  Vehicle trajectory_last = trajectory[1];
  double intended_speed;

  if ((trajectory_last.state.compare("PLCL") == 0) || (trajectory_last.state.compare("PLCR") == 0)) {
	  intended_speed = trajectory[2].v;
  } else {
	  intended_speed = trajectory_last.v;
  }

  //float distance_to_goal = vehicle.goal_s - trajectory_last.s;
  double final_speed = trajectory[1].v;
  trajectory_data["intended_speed"] = intended_speed;
  trajectory_data["final_speed"] = final_speed;
  //trajectory_data["distance_to_goal"] = distance_to_goal;

  return trajectory_data;
}



