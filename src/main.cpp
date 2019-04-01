#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "MPC.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::cout;
using std::endl;



int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    // std::cout << sdata << std::endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          /**
           * Calculate steering angle and throttle using MPC.
           * Both are in between [-1, 1].
           */

          // translate to car coordinate system
          MatrixXd car_coor = transGlobalToLocal(px, py, psi, ptsx, ptsy);       
          VectorXd Ptsx = car_coor.row(0);
          VectorXd Ptsy = car_coor.row(1);
          
          // polynomial fit
          VectorXd coeffs = polyfit(Ptsx, Ptsy, 3);

          // get the cross track error
          double cte = polyeval(coeffs, 0);

          // get the orientation error
          double epsi = -atan(coeffs[1]);

          // inital state: x, y, orientation are 0
          VectorXd state(6);
          //cout << " input states v " << v << endl;
          //cout << " input states cte " << cte << endl;
          //cout << " input states epsi " << epsi << endl;   
    
          state << 0, 0, 0, v, cte, epsi;

          // compute the optimal trajectory
          Solution sol = mpc.Solve(state, coeffs);


          double steer_value = sol.Delta.at(2);
          double throttle_value = sol.A.at(2);

          // update values for previous step in mpc
          mpc.delta_prev = steer_value;
          mpc.a_prev     = throttle_value;

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the 
          //   steering value back. Otherwise the values will be in between 
          //   [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          // angles are revsersed in the car coordinate in the simulator
          msgJson["steering_angle"] = - steer_value/0.436332;
          msgJson["throttle"] = throttle_value;

          // add (x,y) points to list here, points are in reference to the
          // vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          // cout << " x           " << px << endl;
          // cout << " y           " << py << endl;
          // cout << " psi         " << psi << endl;
          // cout << " v           " << v << endl;
          // cout << " cte         " << cte << endl;
          // cout << " epsi        " << epsi << endl;
          // cout << " steer_value " << steer_value << endl ;
          // cout << " throttle    " << throttle_value << endl ;


          // Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          /**
           * add (x,y) points to list here, points are in reference to 
           *   the vehicle's coordinate system the points in the simulator are 
           *   connected by a Green line
           */

          //mpc_x_vals = sol.X;
          //mpc_y_vals = sol.Y;
          for (int i = 0; i < sol.X.size() - 2; i++) {
            mpc_x_vals.push_back(sol.X.at(i + 2));
            mpc_y_vals.push_back(sol.Y.at(i + 2));
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * add (x,y) points to list here, points are in reference to 
           *   the vehicle's coordinate system the points in the simulator are 
           *   connected by a Yellow line
           */
          for (int i = 0; i < ptsx.size(); i++) {
            next_x_vals.push_back(Ptsx(i));
            next_y_vals.push_back(Ptsy(i));
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          //   the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          //   around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE SUBMITTING.
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
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
