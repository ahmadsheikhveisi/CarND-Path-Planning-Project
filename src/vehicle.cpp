/*
 * vehicle.cpp
 *
 *  Created on: Apr 6, 2020
 *      Author: ahmadsv
 */
#include "vehicle.h"
#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include "cost.h"
#include "parameters.h"
#include "iostream"

using std::string;
using std::vector;

// Initializes Vehicle
Vehicle::Vehicle(){}

Vehicle::Vehicle(int id, int lane, double s, double v, double a, string state) {
  this->lane = lane;
  this->s = s;
  this->v = v;
  this->a = a;
  this->state = state;
  this->idx = id;

  this->pred_horizon = PREDICTION_HORIZON_SEC;
}

Vehicle::~Vehicle() {}

void Vehicle::init(int t_lane, double t_s, double t_v, double t_a)
{
	if (this->initialized == false)
	{
		this->lane = t_lane;
		this->s = t_s;
		this->v = t_v;
		this->a = t_a;
		this->initialized = true;
	}
}

vector<Vehicle> Vehicle::choose_next_state(map<int, Vehicle> &otherCars) {
  /**
   * Here you can implement the transition_function code from the Behavior
   *   Planning Pseudocode classroom concept.
   *
   * @param A predictions map. This is a map of vehicle id keys with predicted
   *   vehicle trajectories as values. Trajectories are a vector of Vehicle
   *   objects representing the vehicle at the current timestep and one timestep
   *   in the future.
   * @output The best (lowest cost) trajectory corresponding to the next ego
   *   vehicle state.
   *
   * Functions that will be useful:
   * 1. successor_states - Uses the current state to return a vector of possible
   *    successor states for the finite state machine.
   * 2. generate_trajectory - Returns a vector of Vehicle objects representing
   *    a vehicle trajectory, given a state and predictions. Note that
   *    trajectory vectors might have size 0 if no possible trajectory exists
   *    for the state.
   * 3. calculate_cost - Included from cost.cpp, computes the cost for a trajectory.
   *
   */

	vector<string> possible_successors = successor_states();

	vector<double> cost;

	for (auto state : possible_successors)
	{
		vector<Vehicle> trajectory = generate_trajectory(state,otherCars);

		if (trajectory.size() > 0)
		{
			double t_cost = calculate_cost(*this,otherCars,trajectory);
			cost.push_back(t_cost);
			std::cout << "cost for " << state << " " << t_cost << std::endl;
		}
	}

	auto iter = std::min_element(begin(cost), end(cost));

	string next_best_state = possible_successors[iter - begin(cost)];
	if (next_best_state.compare(this->state) != 0)
	{
		std::cout << "State change from " << this->state << " to " << next_best_state << " with cost " << *iter << std::endl;
	}


  return generate_trajectory(next_best_state,otherCars);
}

vector<string> Vehicle::successor_states() {
  // Provides the possible next states given the current state for the FSM
  //   discussed in the course, with the exception that lane changes happen
  //   instantaneously, so LCL and LCR can only transition back to KL.
  vector<string> states;
  states.push_back("KL");
  /*string state = this->state;
  if(state.compare("KL") == 0) {
	  if (lane != (NUMBER_OF_LANES - 1))
	  {
		  states.push_back("PLCR");
	  }
	  if (lane != 0)
	  {
		  states.push_back("PLCL");
	  }
  } else if (state.compare("PLCL") == 0) {
    if (lane != NUMBER_OF_LANES - 1) {
      states.push_back("PLCL");
      states.push_back("LCL");
    }
  } else if (state.compare("PLCR") == 0) {
    if (lane != 0) {
      states.push_back("PLCR");
      states.push_back("LCR");
    }
  }*/

  // If state is "LCL" or "LCR", then just return "KL"
  return states;
}

vector<Vehicle> Vehicle::generate_trajectory(string state,
                                             map<int, Vehicle> &otherCars) {
  // Given a possible next state, generate the appropriate trajectory to realize
  //   the next state.
  vector<Vehicle> trajectory;
  if (state.compare("CS") == 0) {
    trajectory = constant_speed_trajectory();
  } else if (state.compare("KL") == 0) {
    trajectory = keep_lane_trajectory(otherCars);
  } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
    trajectory = lane_change_trajectory(state, otherCars);
  } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
    trajectory = prep_lane_change_trajectory(state, otherCars);
  }

  return trajectory;
}

vector<double> Vehicle::get_kinematics(map<int, Vehicle> &otherCars,
                                      int t_lane) {
  // Gets next timestep kinematics (position, velocity, acceleration)
  //   for a given lane. Tries to choose the maximum velocity and acceleration,
  //   given other vehicle positions and accel/velocity constraints.
  double max_velocity_accel_limit = (MAX_ACC*this->pred_horizon) + this->v;
  double new_position;
  double new_velocity;
  double new_accel;
  Vehicle vehicle_ahead;
  Vehicle vehicle_behind;

  if (get_vehicle_ahead(otherCars, t_lane, vehicle_ahead)) {
    /*if (get_vehicle_behind(otherCars, t_lane, vehicle_behind)) {
      // must travel at the speed of traffic, regardless of preferred buffer
      new_velocity = vehicle_ahead.v;
    } else*/ {
    	Vehicle vehicle_ahead_t1 = vehicle_ahead.prediction[0];
    	new_accel = 2 / (this->pred_horizon * this->pred_horizon) * (vehicle_ahead.prediction[1].s - this->s - SAFE_GAP - this->v * this->pred_horizon);
    	double max_velocity_in_front = new_accel * this->pred_horizon + this->v;
      /*double max_velocity_in_front = (double)(vehicle_ahead_t1.s - this->s
                                  - SAFE_GAP)/this->pred_horizon + vehicle_ahead_t1.v
                                  - 0.5 * (this->a) * this->pred_horizon;*/
      std::cout << "max velocity in front " << max_velocity_in_front << " max velocity acc " << max_velocity_accel_limit << std::endl;
      new_velocity = std::min(std::min(max_velocity_in_front,
                                       max_velocity_accel_limit),(double)MAX_SPEED_MS);
    }
  } else {
    new_velocity = std::min(max_velocity_accel_limit, (double)MAX_SPEED_MS);
  }

  new_accel = (new_velocity - this->v)/this->pred_horizon; //  (v_1 - v_0)/t = acceleration
  new_position = this->s + new_velocity*this->pred_horizon + this->pred_horizon*this->pred_horizon*new_accel / 2.0; //

  std::cout << " now accel " << new_accel << " new position " << new_position << std::endl;

  return{new_position, new_velocity, new_accel};
}

vector<Vehicle> Vehicle::constant_speed_trajectory() {
  // Generate a constant speed trajectory.
  double next_pos = position_at(this->pred_horizon);
  vector<Vehicle> trajectory = {Vehicle(this->idx,this->lane,this->s,this->v,this->a,this->state),
                                Vehicle(this->idx,this->lane,next_pos,this->v,0,"CS")};
  return trajectory;
}

vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, Vehicle> &otherCars) {
  // Generate a keep lane trajectory.
  vector<Vehicle> trajectory = {Vehicle(this->idx,lane, this->s, this->v, this->a, this->state)};
  vector<double> kinematics = get_kinematics(otherCars, this->lane);
  double new_s = kinematics[0];
  double new_v = kinematics[1];
  double new_a = kinematics[2];
  trajectory.push_back(Vehicle(this->idx,this->lane, new_s, new_v, new_a, "KL"));

  return trajectory;
}

vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state,
                                                     map<int, Vehicle> &otherCars) {
  // Generate a trajectory preparing for a lane change.
  double new_s;
  double new_v;
  double new_a;
  int new_lane = this->lane + lane_direction[state];
  vector<Vehicle> trajectory = {Vehicle(this->idx,this->lane, this->s, this->v, this->a,
                                        this->state)};
  vector<double> curr_lane_new_kinematics = get_kinematics(otherCars, this->lane);
  vector<double> next_lane_new_kinematics = get_kinematics(otherCars, new_lane);

  Vehicle vehicle_behind;
  bool collision_danger = false;
  if (get_vehicle_behind(otherCars, this->lane, vehicle_behind)) {
    // if there is car behind, is it safe to get to the next lane speed?
	  if ((vehicle_behind.prediction[1].s + SAFE_GAP) >= next_lane_new_kinematics[0])
	  {
		  collision_danger = true;
		  new_s = curr_lane_new_kinematics[0];
		  new_v = curr_lane_new_kinematics[1];
		  new_a = curr_lane_new_kinematics[2];
	  }
  }

  if (collision_danger == false){
    vector<double> best_kinematics;
    // Choose kinematics with lowest velocity.so we don't hit the car in front
    if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
      best_kinematics = next_lane_new_kinematics;
    } else {
      best_kinematics = curr_lane_new_kinematics;
    }
    new_s = best_kinematics[0];
    new_v = best_kinematics[1];
    new_a = best_kinematics[2];
  }

  trajectory.push_back(Vehicle(this->idx,this->lane, new_s, new_v, new_a, state));

  return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string state,
                                                map<int, Vehicle> &otherCars) {
  // Generate a lane change trajectory.
  int new_lane = this->lane + lane_direction[state];
  vector<Vehicle> trajectory;
  Vehicle next_lane_vehicle;
  // Check if a lane change is possible (check if another vehicle occupies
  //   that spot).
  for (map<int, Vehicle>::iterator it = otherCars.begin();
       it != otherCars.end(); ++it) {
    next_lane_vehicle = it->second;
    if ((next_lane_vehicle.lane == new_lane) &&
    		((next_lane_vehicle.s < (this->s + SAFE_GAP)) || (next_lane_vehicle.s > (this->s - SAFE_GAP)))) {
      // If lane change is not possible, return empty trajectory.
      return trajectory;
    }
  }
  trajectory.push_back(Vehicle(this->idx,this->lane, this->s, this->v, this->a,
                               this->state));
  vector<double> kinematics = get_kinematics(otherCars, new_lane);
  trajectory.push_back(Vehicle(this->idx,new_lane, kinematics[0], kinematics[1],
                               kinematics[2], state));
  return trajectory;
}

void Vehicle::increment(int dt = 1) {
  this->s = position_at(dt);
}

double Vehicle::position_at(double t) {
  return this->s + (this->v*t) + (this->a*t*t/2.0);
}

bool Vehicle::get_vehicle_behind(map<int, Vehicle> &otherCars,
                                 int t_lane, Vehicle &rVehicle) {
  // Returns a true if a vehicle is found behind the current vehicle, false
  //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
  int max_s = -1;
  bool found_vehicle = false;
  Vehicle temp_vehicle;
  for (map<int, Vehicle>::iterator it = otherCars.begin();
       it != otherCars.end(); ++it) {
    temp_vehicle = it->second;
    Vehicle temp_vehicle_t1 = temp_vehicle.prediction[0];
    if (temp_vehicle_t1.lane == t_lane && temp_vehicle_t1.s < this->s //
        && temp_vehicle_t1.s > max_s) {
      max_s = temp_vehicle_t1.s;
      rVehicle = temp_vehicle;
      found_vehicle = true;
    }
  }
  if (found_vehicle)
  {
	  std::cout << "found vehicle behind " << rVehicle.s << " " << rVehicle.v << " " << rVehicle.lane <<  std::endl;
  }
  else
  {
	  std::cout << "Not found behind " << t_lane << std::endl;
  }

  return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(map<int, Vehicle> &otherCars,
                                int t_lane, Vehicle &rVehicle) {
  // Returns a true if a vehicle is found ahead of the current vehicle, false
  //   otherwise. The passed reference rVehicle is updated if a vehicle is found.
  int min_s = MAX_S;
  bool found_vehicle = false;
  Vehicle temp_vehicle;
  for (map<int, Vehicle>::iterator it = otherCars.begin();
       it != otherCars.end(); ++it) {
    temp_vehicle = it->second;
    Vehicle temp_vehicle_t1 = temp_vehicle.prediction[0];
    if (temp_vehicle_t1.lane == t_lane && temp_vehicle_t1.s > this->s //
        && temp_vehicle_t1.s < min_s) {
      min_s = temp_vehicle_t1.s;
      rVehicle = temp_vehicle;
      found_vehicle = true;
    }
  }
  if (found_vehicle)
  {
	  std::cout << "found vehicle ahead " << rVehicle.s << " " << rVehicle.v << " " << rVehicle.lane <<  std::endl;
  }
  else
  {
	  std::cout << "Not found ahead " << t_lane << std::endl;
  }

  return found_vehicle;
}

void Vehicle::generate_predictions(double start_time, double horizon) {
  // Generates predictions for non-ego vehicles to be used in trajectory
  //   generation for the ego vehicle.
  prediction.clear();

  double s1 = position_at(start_time);
  double v1 = this->v + (start_time * this->a);
  prediction.push_back(Vehicle(this->idx,this->lane,s1,v1,this->a));

  double t2 = start_time + horizon;
  double s2 = position_at(t2);
  double v2 = this->v + (t2 * this->a);
  prediction.push_back(Vehicle(this->idx,this->lane,s2,v2,this->a));

  this->pred_horizon = horizon;

}

void Vehicle::realize_next_state(vector<Vehicle> &trajectory) {
  // Sets state and kinematics for ego vehicle using the last state of the trajectory.
  Vehicle next_state = trajectory[1];
  this->state = next_state.state;
  this->lane = next_state.lane;
  this->s = next_state.s;
  this->v = next_state.v;
  this->a = next_state.a;
}





