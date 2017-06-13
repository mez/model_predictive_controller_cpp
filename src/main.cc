#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "mpc.h"
#include "json.hpp"
#include "Utilities.h"

// for convenience
using json = nlohmann::json;
using namespace Eigen;
using namespace std;
using namespace utilities;

int main() {
  uWS::Hub h;

  // mpc is initialized here!
  mpc mpc;

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
          double delta = j[1]["steering_angle"];
          double a = j[1]["throttle"];

          /**
           * 1st. We must do the following..
           *
           * * Need to convert global way points (ptsx, ptsy) into Car Coord.
           * * Map those converted coords to Eigen VectorXd.
           *
           */

          VectorXd x_way_points(ptsx.size());
          VectorXd y_way_points(ptsy.size());
          for (int i = 0; i < ptsx.size(); ++i) {
            const double dx = ptsx[i]-px;
            const double dy = ptsy[i]-py;

            x_way_points(i) = dx * cos(0-psi) - dy * sin(0-psi);
            y_way_points(i) = dx * sin(0-psi) + dy * cos(0-psi);
          }


          /**
           * 2nd. We use polyfit to fit a 3rd order polynomial like the following:
           *
           * f(x)  = C0 + C1*x + C2*X^2 + C3*X^3
           *
           * Derived form will take the following form:
           *
           * f'(x) =      C1   + 2*C2*X + 3*C3*X^2
           *
           */
          auto coeffs = polyfit(x_way_points, y_way_points, 3);

          /**
           * 3rd. We now calculate the initial CTE and EPSI.
           * We will use x=0.0, y=0.0, psi=0.0 for the calculations.
           *
           * CTE  = Cross Track Error : f(x) - y
           * EPSI = Steering error is : psi - atan(f'(x))
           */

          double cte  = coeffs[0]; //skip polyval to save compute since X=0.0
          double epsi = -atan(coeffs[1]);

          /**
           * 4th. We must create the initial state....
           *
           * * NOTE: Add a 100 milliseconds to the initial state
           */

          const double Lf         = 2.67;
          const double v_over_Lf  = v/Lf;
          const double time_delay = 0.1; //100 milliseconds
          const int number_of_state_dimensions = 6;
          VectorXd state(number_of_state_dimensions);

          const double time_delay_px    = 0.0 + v * time_delay;
          const double time_delay_py    = 0.0;
          const double time_delay_psi   = 0.0 + v * (-delta) / Lf * time_delay;
          const double time_delay_v     = v + a * time_delay;
          const double time_delay_cte   = cte + v * sin(epsi) * time_delay;
          const double time_delay_epsi  = epsi + v * (-delta) / Lf * time_delay;

          state << time_delay_px, time_delay_py, time_delay_psi, time_delay_v, time_delay_cte, time_delay_epsi;

          /**
           * 5th. Ask MPC to make prediction for actuation
           * solver must return steering, throttle and future states for N timesteps
           */

          mpc.Solve(state, coeffs);

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = mpc.steering;
          msgJson["throttle"]       = mpc.throttle;

          msgJson["mpc_x"] = mpc.predicted_x_vals;
          msgJson["mpc_y"] = mpc.predicted_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          const double step_size    = 2.5;
          const int number_of_steps = 25;
          for (int i = 0; i < number_of_steps; i++)
          {
            next_x_vals.push_back(step_size*i);
            next_y_vals.push_back(polyeval(coeffs, step_size*i));
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;

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
