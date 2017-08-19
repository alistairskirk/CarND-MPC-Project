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
    cout << sdata << endl;
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
		  
		  // Coordinates given by ptsx and ptsy are global x and y positions of the waypoints. Need to convert this to car's coordinate system
		  // webSocket data as per https://github.com/udacity/CarND-MPC-Project/blob/master/DATA.md

		  // *******
		  // Rotate and shift such that new reference system is centered on the origin at 0 degrees, adapted from Slack discussion on #p-mpc 
		  
		  for (size_t i = 0; i < ptsx.size(); ++i) {

			  double shift_x = ptsx[i] - px;
			  double shift_y = ptsy[i] - py;

			  ptsx[i] = shift_x * cos(-psi) - shift_y * sin(-psi);
			  ptsy[i] = shift_x * sin(-psi) + shift_y * cos(-psi);
		  }

		  // Convert to Eigen::VectorXd
		  double *ptrx = &ptsx[0];
		  Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx, 6);

		  double *ptry = &ptsy[0];
		  Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry, 6);

		  // *******
		  
		  // have to get the current steering angle and throttle from the json object, for use in latency prediction
		  double kronickerdelta = j[1]["steering_angle"];		  
		  double a = j[1]["throttle"];
		  
		  // Calc coefficients by fitting polynomial (polyfit and polyeval provided above)
		  auto coeffs = polyfit(ptsx_transform, ptsy_transform, 3);
		  double cte = polyeval(coeffs, 0);

		  // Predict the next state to account for latency:
		  const double Lf = 2.67; // needed for normalizing steering value, taken from vehicle calibration
		  const double d_t = 0.1; // needed for predicting future state, must match the delta_t set in MPC class.

		  // Prediction equations from Udacity course shifted for t= t+1:
		  // x_t+1 = x_t + vt * cos(psi_t) * d_t
		  // y_t+1 = y_t + vt * sin(psi_t) * d_t
		  // psi_t+1 = psi_t + vt / Lf * kronikerdelta_t * d_t
		  // v_t+1 = vt + at * d_t
		  // cte_t+1 = f(x_t) - y_t + vt * sin(epsi_t) * d_t
		  // epsi_t+1 = psi_t+1 - psi_des_t + v_t/Lf * krockerdelta_t * dt

		  // Recall the equations for the model from Udacity course:
		  // x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
		  // y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
		  // psi_[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
		  // v_[t] = v[t-1] + a[t-1] * dt
		  // cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
		  // epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt

		  // need to define the psi_des at time t term
		  double psi_des_t = atan(coeffs[1]); // This trick was learned from Lesson 19, Chp. 9 Solution: Mind The Line in Udacity Course
		  
		  // The trick here is that the prediction equations can be simplified as we are using the car's coordinate system
		  // Therefore x_t, y_t, psi_t all equal 0, cos(0) = 1 , sin(0) = 0
		  
		  kronickerdelta *= -1; // because reasons

		  // Using these substitutions in the t+1 prediction equations:
		  double x_pred = 0.0 + v * d_t;
		  double y_pred = 0.0;
		  double psi_pred = 0.0 + v / Lf * kronickerdelta * d_t;
		  double v_pred = v + a*d_t;
		  double cte_pred = cte - 0.0 + v*sin(-psi_des_t)*d_t;
		  double epsi_pred = 0.0 - psi_des_t + v * kronickerdelta / Lf * d_t;


		  // Init state vector, using the six state variables, and their predicted values
		  Eigen::VectorXd state(6);
		  state << x_pred, y_pred, psi_pred, v_pred, cte_pred, epsi_pred;

		  // Call MPC::Solve using state vector and coefficients
		  auto vars = mpc.Solve(state, coeffs);

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
		  		  
		  double steer_value;
		  double throttle_value;
		  
		  //std::cout << "assign steer and throttle from vars" << std::endl;		  
		  std::cout << "steer and throttle from vars: " <<vars[0] <<" " <<vars[1] << std::endl;

		  steer_value = vars[0]; // define the steering value from the solved variables
		  throttle_value = vars[1]; // define the throttle value from the solved variables

		  //std::cout << "normalize steer_value" << std::endl;

		  // Normalize the steering value:
		  steer_value = -1*steer_value / deg2rad(25) / Lf;

		  std::cout << "steer and throttle: " << steer_value << " " << throttle_value << std::endl;

		  // Write the steering and throttle values:
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
		  // get the x and y values from vars list
		  //std::cout << "push mpc x and y vals" << std::endl;
		  for (size_t i = 2; i < vars.size(); i++) {
			  if (i % 2 == 0) mpc_x_vals.push_back(vars[i]);
			  else            mpc_y_vals.push_back(vars[i]);
		  }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
		  
		  // create and add extrapolated points
		  double x_step = 2.5; // set step increment for each x
		  int future_points = 25;    // number of predicted future points
		  //std::cout << "push extrapolated values" << std::endl;
		  for (int i = 1; i < future_points; i++) {
			  double extrap_x = x_step * i;
			  double extrap_y = polyeval(coeffs, extrap_x);
			  next_x_vals.push_back(extrap_x);
			  next_y_vals.push_back(extrap_y);
		  }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does not actuate the commands instantly.
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
