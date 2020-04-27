/*
 * vehicle.h
 *
 *  Created on: Apr 6, 2020
 *      Author: ahmadsv
 */

#ifndef VEHICLE_H
#define VEHICLE_H

#include <map>
#include <string>
#include <vector>

using std::map;
using std::string;
using std::vector;

class Vehicle {
 public:
  // Constructors
  Vehicle();
  Vehicle(int id, int lane, double s, double v, double a, string state="CS");

  // De-structor
  virtual ~Vehicle();

  // Vehicle functions
  vector<Vehicle> choose_next_state(map<int, Vehicle> &otherCars);

  vector<string> successor_states();

  vector<Vehicle> generate_trajectory(string state, map<int, Vehicle> &otherCars);

  vector<double> get_kinematics(map<int, Vehicle> &otherCars, int t_lane);

  vector<Vehicle> constant_speed_trajectory();

  vector<Vehicle> keep_lane_trajectory(map<int, Vehicle> &otherCars);

  vector<Vehicle> lane_change_trajectory(string state,
                                         map<int, Vehicle> &otherCars);

  vector<Vehicle> prep_lane_change_trajectory(string state,
                                              map<int, Vehicle> &otherCars);

  void increment(int dt);

  double position_at(double t);

  bool get_vehicle_behind(map<int, Vehicle> &otherCars, int t_lane,
                          Vehicle &rVehicle);

  bool get_vehicle_ahead(map<int, Vehicle> &otherCars, int t_lane,
                         Vehicle &rVehicle);

  void generate_predictions(double start_time, double horizon);

  void realize_next_state(vector<Vehicle> &trajectory);


  // public Vehicle variables
  struct collider{
    bool collision; // is there a collision?
    int  time; // time collision happens
  };

  map<string, int> lane_direction = {{"PLCL", -1}, {"LCL", -1},
		  	  	  	  	  	  	  	  {"LCR", 1}, {"PLCR", 1}};

  int L = 1;

  int lane;

  double v, a, s;

  string state;

  int idx;

  vector<Vehicle> prediction;

  double pred_horizon;

  bool initialized = false;

  void init(int t_lane, double t_s, double t_v, double t_a);

};

#endif  // VEHICLE_H
