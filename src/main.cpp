#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "vehicle.h"
#include "parameters.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = MAX_S;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  //this holds the last behavioral planning output
  Vehicle ego = Vehicle(-1,LANE_INIT,0,SPEED_INIT,0,"KL");

  double ref_vel = SPEED_INIT;

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&ego,&ref_vel]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          int prev_size = previous_path_x.size();
          int points_needed = NUMBER_OF_POINTS - prev_size;

           /*if(prev_size > 0)
           {
        	   car_s = end_path_s;
           }*/
          double pred_start_time = (double)prev_size * (double)TIME_STEP;
          double pred_duration_time = PREDICTION_HORIZON_SEC;//(double)(NUMBER_OF_POINTS - prev_size) * (double)TIME_STEP;
          std::cout << "##########" << std::endl;
          std::cout << "points needed " << points_needed << std::endl;
          std::cout << "car s " << car_s << " car d " << car_d << " car speed " << car_speed << std::endl;


           map<int, Vehicle> otherCars;

           for(int car_cnt = 0; car_cnt < sensor_fusion.size(); ++car_cnt)
           {
        	  float t_car_d = sensor_fusion[car_cnt][6];
        	  int t_car_lane = D2LANE(t_car_d);
			  double vx = sensor_fusion[car_cnt][3];
			  double vy = sensor_fusion[car_cnt][4];
			  double check_speed = sqrt((vx*vx)+(vy*vy));
			  double check_car_s = sensor_fusion[car_cnt][5];

        	  Vehicle t_car = Vehicle(sensor_fusion[car_cnt][0],t_car_lane,check_car_s,check_speed,0);

        	  t_car.generate_predictions(pred_start_time,pred_duration_time);
        	  otherCars[t_car.idx] = t_car;

        	  std::cout << "sensor fusion cars " << t_car.idx << " " << t_car.lane << " " << t_car.s << " " << t_car.v << std::endl;

        	  //if (car_d < (LANE_WIDTH*(lane+1)) && car_d > (LANE_WIDTH*lane))
        	  /*if (car_lane == lane)
        	  {
        		  check_car_s += ((double)prev_size*TIME_STEP*check_speed);
        		  if ((check_car_s > car_s) && ((check_car_s - car_s) < SAFE_GAP))
        		  {
        			  too_close = true;
        		  }
        	  }*/
           }

		   /*ego.s = car_s;
		   ego.a = (MPH2MPS(car_speed) - ego.v)/(float)PREDICTION_HORIZON_SEC;
		   ego.v = MPH2MPS(car_speed);*/

		   std::cout << "********" << std::endl;
		   ego.init(D2LANE(car_d), car_s, MPH2MPS(car_speed),0);
		   vector<Vehicle> traj;
		   if (points_needed > 0)
		   {
			   if(prev_size > 0)
			   {
				   ego.s = end_path_s;
			   }
			   ego.v = ref_vel;

			   std::cout << "State " << ego.state << " lane " << ego.lane << " s " << ego.s << " speed " << MPS2MPH(ego.v) << " ego acc " << ego.a << std::endl;

			   //ego.generate_predictions(0.0,pred_duration_time);
			   traj = ego.choose_next_state(otherCars);


			   std::cout << "State " << traj[1].state << " lane " << traj[1].lane << " s " << traj[1].s << " speed " << traj[1].v << " ego acc " << ego.a << std::endl;

			   ego.realize_next_state(traj);

		   }

           /*if (ref_vel > MPS2MPH(ego.v))//(too_close)
           {
        	   ref_vel -= MPS2MPH(MAX_ACC*TIME_STEP);
           }
           else if (ref_vel < MPS2MPH(ego.v))
           {
        	   ref_vel += MPS2MPH(MAX_ACC*TIME_STEP);;
           }*/

           vector<double> ptsx;
           vector<double> ptsy;

           double ref_x = car_x;
           double ref_y = car_y;
           double ref_yaw = deg2rad(car_yaw);

           // get the last two points
           if (prev_size < 2)
           {
        	   double prev_car_x = car_x - cos(car_yaw);
        	   double prev_car_y = car_y - sin(car_yaw);

        	   ptsx.push_back(prev_car_x);
        	   ptsx.push_back(car_x);

        	   ptsy.push_back(prev_car_y);
        	   ptsy.push_back(car_y);
           }
           else
           {
        	   ref_x = previous_path_x[prev_size - 1];
        	   ref_y = previous_path_y[prev_size - 1];

        	   double ref_x_prev = previous_path_x[prev_size - 2];
        	   double ref_y_prev = previous_path_y[prev_size - 2];
        	   ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

        	   ptsx.push_back(ref_x_prev);
        	   ptsy.push_back(ref_y_prev);

        	   ptsx.push_back(ref_x);
        	   ptsy.push_back(ref_y);
           }


           /*if (traj.size() > 0)
           {
			   vector<double> next_wp1 = getXY(traj[1].s,(((double)LANE_WIDTH/(double)2.0) + (LANE_WIDTH*traj[1].lane)),map_waypoints_s,map_waypoints_x,map_waypoints_y);

			   ptsx.push_back(next_wp1[0]);
			   ptsy.push_back(next_wp1[1]);
           }
           else*/
           {
               vector<double> next_wp0 = getXY(ego.s + POINT_FORWARD_STEP,(((double)LANE_WIDTH/(double)2.0) + (LANE_WIDTH*ego.lane)),map_waypoints_s,map_waypoints_x,map_waypoints_y);
               vector<double> next_wp1 = getXY(ego.s + POINT_FORWARD_STEP*2,(((double)LANE_WIDTH/(double)2.0) + (LANE_WIDTH*ego.lane)),map_waypoints_s,map_waypoints_x,map_waypoints_y);
               vector<double> next_wp2 = getXY(ego.s + POINT_FORWARD_STEP*3,(((double)LANE_WIDTH/(double)2.0) + (LANE_WIDTH*ego.lane)),map_waypoints_s,map_waypoints_x,map_waypoints_y);

               ptsx.push_back(next_wp0[0]);
               ptsy.push_back(next_wp0[1]);

			   ptsx.push_back(next_wp1[0]);
			   ptsy.push_back(next_wp1[1]);

			   ptsx.push_back(next_wp2[0]);
			   ptsy.push_back(next_wp2[1]);
           }

           // move the reference points to the car coordinates
          for (int cnt = 0; cnt < ptsx.size(); cnt++)
          {
        	  double shift_x = ptsx[cnt] - ref_x;
        	  double shift_y = ptsy[cnt] - ref_y;

        	  ptsx[cnt] = (shift_x * cos(0 - ref_yaw)) - (shift_y * sin(0 - ref_yaw));
        	  ptsy[cnt] = (shift_x * sin(0 - ref_yaw)) + (shift_y * cos(0 - ref_yaw));

        	  std::cout << " ptsx " << ptsx[cnt] << std::endl;
          }

          tk::spline spln;

          spln.set_points(ptsx,ptsy);
          // add the previous points
          for (int cnt = 0; cnt < previous_path_x.size(); ++cnt)
          {
        	  next_x_vals.push_back(previous_path_x[cnt]);
        	  next_y_vals.push_back(previous_path_y[cnt]);
          }

          double target_x = ptsx[ptsx.size() - 1];
          double target_y = spln(target_x);
          double target_dist = sqrt((target_x*target_x)+(target_y*target_y));

          std::cout << " target_x " << target_x << " target_y " << target_y << " target_dist " << target_dist << std::endl;

          /*vector<double> pst;
          vector<double> ped;
          vector<double> poly;

          if (traj.size() > 0)
          {
			  pst = {ego.s,ego.v,ego.a};
			  ped = {traj[1].s,traj[1].v,traj[1].a};
			  poly = JMT(pst,ped,pred_duration_time);
          }*/

          double x_add_on = 0;

          // generate new points
          for (int cnt = 0; cnt < NUMBER_OF_POINTS - previous_path_x.size(); ++cnt)
          {
        	  if (ref_vel < ego.v)
        	  {
        		  ref_vel += MAX_ACC*TIME_STEP;
        	  }
        	  else if (ref_vel > ego.v)
        	  {
        		  ref_vel -= MAX_ACC*TIME_STEP;
        	  }
        	  double dist_seg = ref_vel * TIME_STEP;//((traj[1].v + ego.v) * TIME_STEP / 2.0);//evaluate_poly(poly,(cnt + 1)*TIME_STEP) - evaluate_poly(poly,cnt*TIME_STEP);//
        	  double N = target_dist/(dist_seg);
        	  double x_point = x_add_on + target_x/N;
        	  double y_point = spln(x_point);

        	  std::cout << " dist seg " << dist_seg << " N " << N << " xpoint " << x_point << " y_point " << y_point << std::endl;

        	  x_add_on = x_point;

        	  double x_ref = x_point;
        	  double y_ref = y_point;

        	  x_point = x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw);
        	  y_point = x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw);

        	  x_point += ref_x;
        	  y_point += ref_y;

        	  next_x_vals.push_back(x_point);
        	  next_y_vals.push_back(y_point);
          }

          std::cout << " points generated " << next_x_vals.size() << std::endl;


          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}
