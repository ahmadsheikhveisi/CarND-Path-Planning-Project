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
#include <algorithm>
#include <map>
#include <string>
#include <vector>
#include "vehicle.h"
#include "parameters.h"
#include "iostream"
#include <limits>

using std::string;
using std::vector;


/**
 * weights for cost functions.
 */
const double EFFICIENCY = pow(10.0,3.0);
const double TRAFIC_AVOIDANCE = pow(10.0,1.0);//0.0;//
const double FASTEST_LANE_DISTANCE = pow(10.0,2.0);//0.0;//


double fastest_lane_cost(const Vehicle &vehicle,
                        const vector<Vehicle> &trajectory,
                        const map<int, Vehicle> &otherCars,
                        map<string, double> &data)
{
	//finding the fastest going lane
	vector<double> lane_speeds(NUMBER_OF_LANES, MAX_SPEED_MS);
	vector<double> min_s(NUMBER_OF_LANES, std::numeric_limits<double>::max());

	double intended_s = data["intended_s"];
	int intended_lane = static_cast<int>(data["intended_lane"]);

	//std::cout << "intended_s " << intended_s << " intended lane " << intended_lane << std::endl;

	for (auto t_vehicle : otherCars)
	{
		Vehicle &p_vehicle = t_vehicle.second.prediction[1];

		if (p_vehicle.lane < NUMBER_OF_LANES && p_vehicle.lane >= 0)
		{
			if ((p_vehicle.s > intended_s) &&
					(p_vehicle.s < min_s[p_vehicle.lane]))
			{
				min_s[p_vehicle.lane] = p_vehicle.s;
				lane_speeds[p_vehicle.lane] = std::min(p_vehicle.v,MAX_SPEED_MS);
			}
		}
	}

  double cost = 0.0;

  vector<double> lane_cost;

  for (int cnt = 0; cnt < NUMBER_OF_LANES; ++cnt)
  {
	  lane_speeds[cnt] = (MAX_SPEED_MS - lane_speeds[cnt]);
	  min_s[cnt] = (min_s[cnt] - intended_s);

	  lane_cost.push_back(lane_speeds[cnt]/min_s[cnt]);
  }

  int fastest_lane = std::min_element(lane_cost.begin(),lane_cost.end()) - lane_cost.begin();

  double t_cost = fabs(lane_cost[fastest_lane] - lane_cost[intended_lane]) * fabs(fastest_lane - intended_lane);

  if (t_cost > 0.0)
  {
	  //std::cout << "fastest lane " << fastest_lane << " " << lane_cost[0] << " " << lane_cost[1] << " " << lane_cost[2] << std::endl;

	  //std::cout << "cost for lane " << intended_lane << " is " << t_cost << " " << lane_speeds[intended_lane] << " " << min_s[intended_lane] << std::endl;
  }
  cost += t_cost;

  if (cost < 0.05)
  {
	  cost = 0.0;
  }

	//auto iter = std::min_element(begin(lane_speeds),end(lane_speeds));

	//int fastest_lane = iter - lane_speeds.begin();
    //cost = 1 - exp(-(abs(2.0*fastest_lane - data["intended_lane"]
    //     - data["final_lane"])));

    //std::cout << "cost for fastest lane is " << cost << " " << fastest_lane << " " << max_speed << std::endl;

  return cost;
}

double traffic_avoidance_cost(const Vehicle &vehicle,
                        const vector<Vehicle> &trajectory,
                        const map<int, Vehicle> &otherCars,
                        map<string, double> &data)
{
	double res_cost = 0.0;

	double min_s = std::numeric_limits<double>::max();

	double lane_speed = MAX_SPEED_MS;

	double intended_s = data["intended_s"];

	for (auto t_vehicle : otherCars)
	{
		Vehicle &p_vehicle = t_vehicle.second.prediction[1];
		int t_lane = p_vehicle.lane;
		int t_s = p_vehicle.s;
		int t_v = p_vehicle.v;
		if ((t_lane  == static_cast<int>(data["intended_lane"])) &&
				(t_s > intended_s) &&
				(t_s < min_s))
		{
			min_s = t_s;
			lane_speed = t_v;

			res_cost += 0.001;
		}
	}

	double distance = (min_s - intended_s);

	res_cost += std::max((MAX_SPEED_MS - lane_speed),0.0)/(distance);

	if (res_cost < 0.05)
	{
		//std::cout << "cost is so small " << distance << " " << (MAX_SPEED_MS - lane_speed) << std::endl;
		res_cost = 0.0;
	}

	return res_cost;
}

double inefficiency_cost(const Vehicle &vehicle,
                        const vector<Vehicle> &trajectory,
                        const map<int, Vehicle> &otherCars,
                        map<string, double> &data) {
  // Cost becomes higher for trajectories with intended lane and final lane
  //   that have traffic slower than vehicle's target speed.
  double proposed_speed_intended = data["intended_speed"];
  if (proposed_speed_intended < 0) {
    proposed_speed_intended = MAX_SPEED_MS;
  }

  double proposed_speed_final = data["final_speed"];
  if (proposed_speed_final < 0) {
    proposed_speed_final = MAX_SPEED_MS;
  }

  //std::cout << "intended_speed " << proposed_speed_intended << " final speed " << proposed_speed_final << std::endl;

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
    >> cf_list = { inefficiency_cost,traffic_avoidance_cost,fastest_lane_cost};
  vector<double> weight_list = {EFFICIENCY,TRAFIC_AVOIDANCE,FASTEST_LANE_DISTANCE};

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
  double intended_speed = trajectory_last.v;
  int intended_lane = trajectory_last.lane;
  double intended_s = trajectory_last.s;

  if ((trajectory_last.state.compare("PLCL") == 0) || (trajectory_last.state.compare("PLCR") == 0))
  {
	  intended_speed = trajectory[2].v;
	  intended_lane = trajectory[2].lane;
	  intended_s = trajectory[2].s;
  }

  double final_s = trajectory_last.s;
  double final_speed = trajectory_last.v;
  double final_lane = trajectory_last.lane;

  trajectory_data["intended_speed"] = intended_speed;
  trajectory_data["final_speed"] = final_speed;

  trajectory_data["intended_lane"] = intended_lane;
  trajectory_data["final_lane"] = final_lane;

  trajectory_data["intended_s"] = intended_s;
  trajectory_data["final_s"] = final_s;

  return trajectory_data;
}



