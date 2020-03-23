#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include <time.h>

// for convenience
using nlohmann::json;
using std::string;

using namespace std::chrono;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}


//bool SIM_RESET = false;


int main() {
  uWS::Hub h;

  PID pid;
  PID thr_pid;

  /**
   * TODO: Initialize the pid variable.
   */



  h.onMessage([&pid,&thr_pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */

          double tau_p = 0.2;
          double tau_d = 1.4;
          double tau_i = 1.0;

          pid.Init(tau_p,tau_d,tau_i);

          double tau_p2 = 0.2;
          double tau_d2 = 0.0;
          double tau_i2 = 0.0;

          thr_pid.Init(tau_p2,tau_d2,tau_i2);


          double setpoint_steering = 0.0;
          //double steering_value = pid.Run(cte, setpoint_steering);
          int n=30;
          double steering_value = pid.Twiddle(cte, setpoint_steering, n);
          std::cout<< pid.loops << std::endl;
          std::cout<< pid.TotalError() << std::endl;
          double setpoint_speed = 20;
          double throttle_value = thr_pid.Run(speed, setpoint_speed);

          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steering_value
            //        << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steering_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";

          // Reset the car to starting position
          if (pid.SIM_RESET){
            msg = "42[\"reset\",{}]";
            std::cout << "SIMULATOR RESET"<< std::endl;
          }

          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
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
