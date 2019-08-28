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

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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
    std::cout << sdata << std::endl;
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
          double psi = j[1]["psi"]; // radians
          double v = j[1]["speed"]; // mph
          double delta = j[1]["steering_angle"];
          double throttle = j[1]["throttle"];
          /**
           * TODO: Calculate steering angle and throttle using MPC.
           * Both are in between [-1, 1].
           */

          // System Delay
          doubel delay = 100/1000; // seconds
          doubel Lf = 2.67;

          double delayed_px  = px + v * cos(psi) * delay;
          double delayed_py  = py + v * sin(psi) * delay;
          double delayed_psi = psi + (v * tan(-delta) / Lf) * delay +
                              ( (throttle * tan(-delta) / (2*Lf)) * pow(delay,2) );
          double delayed_v = v + throttle * delay;

          // Transform from map coordinate to vehicle coordinate
          int num_points = ptsx.size();
          Eigen::VectorXd points_px(num_points);
          Eigen::VectorXd points_py(num_points);

          for (int i=0; i<num_points; ++i){
            doubel delta_px = ptsx[i] - delayed_px;
            doubel delta_py = ptsy[i] - delayed_py;

            points_px[i] = delta_px * cos(-delayed_psi) - delta_py * sin(-delayed_psi);
            points_py[i] = delta_py * cos(-delayed_psi) + delta_px * sin(-delayed_psi);
          }

          // Polynomials coefficients
          auto coeffs = polyfit(points_px, points_py, 3);
          // State
          double cte = polyeval(coeffs, 0);
          double epsi = -atan(coeffs[1])
          Eigen::VectorXd state(6);
          state << 0, 0, 0, v, cte, epsi;

          // Calculate the best acutation
          auto vars = mpc.Solve(state, coeffs);
          double steer_value = -vars[0] / deg2rad(25);
          double throttle_value = vars[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the
          //   steering value back. Otherwise the values will be in between
          //   [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          /**
           * TODO: add (x,y) points to list here, points are in reference to
           *   the vehicle's coordinate system the points in the simulator are
           *   connected by a Green line
           */
          auto N = (vars.size()-2)/2;

          doubel temp_x;
          doubel temp_y;

          for(int i=3; i<N+2; ++i){
            temp_x = vars[i];
            temp_y = vars[i+N];
            mpc_x_vals.push_back(temp_x);
            mpc_y_vals.push_back(temp_y);
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: add (x,y) points to list here, points are in reference to
           *   the vehicle's coordinate system the points in the simulator are
           *   connected by a Yellow line
           */
         for (int i = 0; i < points_px.size(); i++)
         {
           next_x_vals.push_back(points_px[i]);
           next_y_vals.push_back(points_py[i]);
         }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
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
