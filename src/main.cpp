#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "vehicle.h"
#include "constants.h"
#include "vehicle.cpp"
#include "ptg.h"
#include "ptg.cpp"
#include "spline.h"


// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::map;

string state = "CS";
Vehicle prev_traj_target(0,0,0,0);
double speed = 0;
double acc = 0;
Vehicle ego =Vehicle(0,0,0,0);
double prev_ego_speed =0.0;
vector<double> GOAL = {6945.554, 1};
double goal_s = std::floor(GOAL[0]);
double goal_lane = GOAL[1];
double num_lanes = LANE_SPEEDS.size();
vector<double> ego_config = {SPEED_LIMIT,num_lanes,goal_s,goal_lane,MAX_ACCEL};
double v  = 0.0;

int main() {


  uWS::Hub h;
  ego.configure(ego_config);


  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

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


  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
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
          car_speed = car_speed/2.24 ;
          //std::cout<<"car yaw deg"<<car_yaw<<"\n";
          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];



          /*add here*/

     




          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */

         map<int, Vehicle> vehicles;
         for(int i = 0;i<sensor_fusion.size();i++)
          {
           int id  =  sensor_fusion[i][0];
           float d = sensor_fusion[i][6];
           float s = sensor_fusion[i][5];
           float vx = sensor_fusion[i][3];
           float vy = sensor_fusion[i][4];
           float v = sqrt(vx*vx + vy*vy);
           int l = 0;
           if(d>0 && d <=4) l = 0;
           else if(d>4 && d<=8) l = 1;
           else if(d>8 && d<=12) l = 2;
           else l = -1;

           Vehicle vehicle = Vehicle(l,s,v,0);
           vehicle.state = "CS";
           vehicles.insert(std::pair<int,Vehicle>(id,vehicle));
          }
          int prev_size = previous_path_x.size();

          double V_end_path_s =0;
          double a =0;

         if(prev_size > 2)
          {

          double d = end_path_s - car_s;
          a = 2*(d - car_speed*0.02*(prev_size-1))/((0.02*0.02)*(prev_size-1)*(prev_size-1));
       

          }
   


          if(prev_size < 2)
          {
          int lane_num =0;
          if(car_d>0 && car_d <=4) lane_num = 0;
          else if(car_d>4 && car_d<=8) lane_num = 1;
          else if(car_d>8 && car_d<=12) lane_num = 2;
          else lane_num = -1;
          ego.lane = lane_num;
          }
          ego.s = car_s;
          ego.v = car_speed;
          ego.a = a;
     
          std::cout<<car_speed<<"  "<<prev_ego_speed<<" covered points "<<50-prev_size<<"\n";

          prev_ego_speed = car_speed;

          if((abs(car_d - double(prev_traj_target.lane*4+2)) <1.5) && (ego.state.compare("PLCL")||ego.state.compare("PLCR")))
          {
           ego.state = prev_traj_target.state;
           ego.lane = prev_traj_target.lane;
          }

          if((abs(car_d - double(prev_traj_target.lane*4+2)) < 1.5) && ego.state.compare("KL"))
          {
          ego.state = prev_traj_target.state;
          ego.lane = prev_traj_target.lane;
          }
        
          if((abs(car_d - double(prev_traj_target.lane*4+2)) < 1.5) && ego.state.compare("CS"))
          {
          
          ego.state = prev_traj_target.state;
          ego.lane = prev_traj_target.lane;
          }




          std::cout<<"Ego car para: ";ego.print_vehicle_para();
          std::cout<<"Prev traj: ";prev_traj_target.print_vehicle_para();

       ;

          map<int ,vector<Vehicle>> predictions;
		  map<int, Vehicle>::iterator it = vehicles.begin();

          while (it != vehicles.end())
          {
   			 int v_id = it->first;
             vector<Vehicle> preds = it->second.generate_predictions();
    		predictions[v_id] = preds;
    		
    		++it;
         }

    

          vector<Vehicle> trajectory;
          trajectory = ego.choose_next_state(predictions);
      

          std::cout<<"Car position :"<<car_s<<" car speed "<<car_speed<<" d " <<car_d<<" final goal lane "<<ego.goal_lane<<" end_path_s "<<end_path_s<<" end_path_d "<<end_path_d<<"\n";
        



          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
          vector<double> fr;
          if(prev_size<2)
          {
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
       
            fr = getFrenet(ref_x, ref_y, ref_yaw, map_waypoints_x,map_waypoints_y);

          }
          else
          {
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
            fr = {end_path_s,end_path_d};
            
          }

        
         std::cout<<"Trajectory: ";
         trajectory[1].print_vehicle_para();
     

        vector<double> xy;
        xy = getXY(trajectory[1].s+40, trajectory[1].lane*4+2, map_waypoints_s,map_waypoints_x,map_waypoints_y);
        ptsx.push_back(xy[0]);
        ptsy.push_back(xy[1]);
        xy = getXY(trajectory[1].s+60, trajectory[1].lane*4+2, map_waypoints_s,map_waypoints_x,map_waypoints_y);
        ptsx.push_back(xy[0]);
        ptsy.push_back(xy[1]);
        xy = getXY(trajectory[1].s+90, trajectory[1].lane*4+2, map_waypoints_s,map_waypoints_x,map_waypoints_y);
        ptsx.push_back(xy[0]);
        ptsy.push_back(xy[1]);

   
       
  

          for(int i =0;i<ptsx.size();i++)
          {
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;
            ptsx[i] = (shift_x * cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
            ptsy[i] = (shift_x * sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
         
          }

  

          tk::spline s;
          s.set_points(ptsx,ptsy);

          vector<double> next_x_vals;
          vector<double> next_y_vals;

         for(int i =0;i< previous_path_x.size();i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          double target_x = 30;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x)*(target_x) + (target_y)*(target_y));

          double x_add_on = 0 ;
    
          double x_point = 0;
          std::cout<< " target_x "<<target_x;

      


          for(int i =0;i < 50-prev_size;i++)
          {
            if(abs(v-trajectory[1].v) > .1)
            {
            v= v+trajectory[1].a *.02;
            }
            else
            {
              v = trajectory[1].v;
            }
            



            double N = (target_dist/(0.02*v));
            
            x_point =  x_add_on+(target_x)/N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            x_point = (x_ref * cos(ref_yaw)-y_ref*sin(ref_yaw));
            y_point = (x_ref * sin(ref_yaw)+y_ref*cos(ref_yaw));

            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);

          }
          prev_traj_target = trajectory[1];


          json msgJson;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;
         
          std::cout<<"\n\n";
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
