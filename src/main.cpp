#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

//Latency in sec
const double Latency = 0.1;
const double Lf = 2.67;
const bool debug = 0;

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
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}


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
    // cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px  = j[1]["x"];
          double py  = j[1]["y"];
          double psi = j[1]["psi"];
          double v   = j[1]["speed"];

          /*
          *Calculate steering angle and throttle using MPC.
          * Both are in between [-1, 1].
          */

          v = 0.44704*v ; // Convert Miles per hour to meter/sec

          //Retrieve stearing angle:
          double steer_value = j[1]["steering_angle"];
          //steering is reversed, needs transformation
          steer_value = - steer_value;

          // Convert from map coordinates to car coordinates
          vector<double> ptsx_car(ptsx.size());
          vector<double> ptsy_car(ptsy.size());

          for (size_t i = 0; i < ptsx.size(); i++){
            double translate_x = ptsx[i] - px;
            double translate_y = ptsy[i] - py;
            // cos(0-x) = cosx, sin(0-x) = -sin(x)
            ptsx_car[i] = translate_x * cos(psi) + translate_y*sin(psi);
            ptsy_car[i] = -translate_x * sin(psi) + translate_y*cos(psi);
          }

          // fit 3rd order polynomian to road
          Eigen::VectorXd ptsx_road = Eigen::VectorXd::Map(ptsx_car.data(), ptsx_car.size());
          Eigen::VectorXd ptsy_road = Eigen::VectorXd::Map(ptsy_car.data(), ptsy_car.size());

          auto coeffs = polyfit(ptsx_road, ptsy_road,3);

          //Current State, x=0, y=0

          //Cross Track Error (CTE) in car coord.,  polyeval(coeffs, x) - y;
          double cte = polyeval(coeffs, 0);

          //Orientation error, psi - atan(coeffs[1]);
          double epsi = - atan(coeffs[1]);


          //Calculate state in 100ms
          // x will be 0.1s * v m/s, y=0, psi= 0.1s * v/Lf*steer_val, ~v, ~cte, ~epsi
          Eigen::VectorXd est_state(6);

          est_state <<  v*Latency,                    //x
                        0,                            //y
                        (v/Lf)* steer_value*Latency,  //angle
                        v,                            //we don't know the change
                        cte + v* sin(epsi)*Latency,   //CTE
                        epsi + (v/Lf)*steer_value*Latency;//PSI

          // solve the equation
          auto est_traj = mpc.Solve(est_state, coeffs);

          //double est_steer = -est_traj[0]/deg2rad(25);
          //double est_throt = est_traj[1];
          steer_value = -est_traj[0]/ deg2rad(25);
          double throttle_value = est_traj[1];


          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;


          //Display the MPC predicted trajectory

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          // retrieve the N (x,y) values of MPC, starting from 3rd value
          for ( size_t i = 2; i < est_traj.size(); i++){
            if (i%2 == 0) {
              mpc_x_vals.push_back(est_traj[i]);
            } else {
              mpc_y_vals.push_back(est_traj[i]);
            }
          }


          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals = ptsx_car;
          vector<double> next_y_vals = ptsy_car;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          if (debug) std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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