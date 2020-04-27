/*
 * cost.h
 *
 *  Created on: Apr 6, 2020
 *      Author: ahmadsv
 */

#ifndef COST_H
#define COST_H

#include "vehicle.h"

using std::map;
using std::string;
using std::vector;

double calculate_cost(const Vehicle &egoVehicle,
                     const map<int, Vehicle> &otherCars,
                     const vector<Vehicle> &trajectory);

/*float goal_distance_cost(const Vehicle &vehicle,
                         const vector<Vehicle> &trajectory,
                         const map<int, vector<Vehicle>> &predictions,
                         map<string, float> &data);*/

double inefficiency_cost(const Vehicle &vehicle,
                        const vector<Vehicle> &trajectory,
                        const map<int, Vehicle> &otherCars,
                        map<string, double> &data);



map<string, double> get_helper_data(const Vehicle &vehicle,
                                   const vector<Vehicle> &trajectory,
                                   const map<int, Vehicle> &otherCars);

#endif  // COST_H
