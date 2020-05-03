/*
 * parameters.h
 *
 *  Created on: Apr 14, 2020
 *      Author: ahmadsv
 */

#ifndef PARAMETERS_H
#define PARAMETERS_H

#define MPS2MPH(_VAL) ((_VAL)*(2.2369))
#define MPH2MPS(_VAL) ((_VAL)/(2.2369))
#define MAX_SPEED 49.5
#define MAX_SPEED_MS MPH2MPS(MAX_SPEED)
#define LANE_INIT 1
#define SPEED_INIT 0.0
#define MAX_ACC 5.0//8.0//10.0//
#define MAX_JERK 10.0
#define TIME_STEP 0.02
#define LANE_WIDTH 4
#define POINT_FORWARD_STEP 30
#define SAFE_GAP 15//10//20//
#define NUMBER_OF_POINTS 30
#define MAX_S 6945.554
#define PREDICTION_HORIZON_SEC 4
#define SPEED_ADJUST_MPS MPH2MPS((double)0.1)

#define NUMBER_OF_LANES 3
#define D2LANE(_VAL) (static_cast<int>((_VAL)/(double)LANE_WIDTH))//NUMBER_OF_LANES -

#endif // PARAMETERS_H
