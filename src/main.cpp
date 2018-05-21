#include "Tools.h"
#include "UKFTracker.h"
#include "colors.h"

#include "json.hpp"

#include <uWS/uWS.h>
#include <math.h>
#include <time.h>
#include <iostream>
#include <math.h>
#include <time.h>


#define SENSOR 'B' // TODO: Prompt the user for this!


// For convenience:
using json = nlohmann::json;
using namespace std;


// HELPER FUNCTIONS:

/*
* Checks if the SocketIO event has JSON data.
* If there is data the JSON object in string format will be returned,
* else the empty string "" will be returned.
*/
string hasData(const string s) {
    const auto found_null = s.find("null");
    const auto b1 = s.find_first_of("[");
    const auto b2 = s.rfind("}]");

    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }

    return "";
}


// MAIN:

int main() {
    // Create a EKFTracker instance
    UKFTracker tracker; // TODO: This can use the abstract class type

    // MESSAGE PROCESSING:

    uWS::Hub h;  // Initialize WebSocket.

    h.onMessage([
        &tracker
    ](
        uWS::WebSocket<uWS::SERVER> ws,
        char *data,
        size_t length,
        uWS::OpCode opCode
    ) {

        // "42" at the start of the message means there's a websocket message event:
        // - The 4 signifies a websocket message
        // - The 2 signifies a websocket event
        const string sdata = string(data).substr(0, length);

        if (sdata.size() <= 2 || sdata[0] != '4' || sdata[1] != '2') {
            return;
        }

        const string s = hasData(sdata);

        if (s == "") {
            // Manual driving:

            std::string msg = "42[\"manual\",{}]";

            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

            return;
        }

        const auto j = json::parse(s);
        const string event = j[0].get<string>();

        if (event != "telemetry") {
            return;
        }

        // j[1] is the data JSON object:

        const string sensor_measurment = j[1]["sensor_measurement"];

        MeasurementPackage meas_package;
        istringstream iss(sensor_measurment);

        // Reads first element from the current line:
        string sensor_type;
        iss >> sensor_type;

        // Process laser and/or radar data:
        // The compiler will remove the unused code blocks.

        if (SENSOR != 'R' && sensor_type.compare("L") == 0) {

            // Set sensor type:
            meas_package.sensor_type_ = MeasurementPackage::LASER;

            // Read measurements:

            double px;
            double py;

            iss >> px;
            iss >> py;

            // Set them in the package:

            meas_package.raw_measurements_ = VectorXd(2);
            meas_package.raw_measurements_ << px, py;

        } else if (SENSOR != 'L' && sensor_type.compare("R") == 0) {

            // Set sensor type:
            meas_package.sensor_type_ = MeasurementPackage::RADAR;

            // Read measurements:

            double ro;
            double theta;
            double ro_dot;

            iss >> ro;
            iss >> theta;
            iss >> ro_dot;

            // Set them in the package:

            meas_package.raw_measurements_ = VectorXd(3);
            meas_package.raw_measurements_ << ro, theta, ro_dot;

        }

        // Read timestamp, x, y, vx and vy ground truth values:

        long long timestamp;
        double x_gt;
        double y_gt;
        double vx_gt;
        double vy_gt;

        iss >> timestamp;
        iss >> x_gt;
        iss >> y_gt;
        iss >> vx_gt;
        iss >> vy_gt;

        // Set them in the package:

        meas_package.timestamp_ = timestamp;
        meas_package.gt_ = VectorXd(4);
        meas_package.gt_ << x_gt, y_gt, vx_gt, vy_gt;

        // Process the current measurement:
        tracker.processMeasurement(meas_package);

        // Get the current state:
        VectorXd state = tracker.getCurrentState();

        // Get the current RMSE:
        VectorXd RMSE = tracker.getCurrentRMSE();

        // Get all individual components from the current state and RMSE:

        const double px = state(0);
        const double py = state(1);

        const double RMSE_X = RMSE(0);
        const double RMSE_Y = RMSE(1);
        const double RMSE_VX = RMSE(2);
        const double RMSE_VY = RMSE(3);

        // Send estimated position and RMSEs back to the simulator:

        json msgJson;

        msgJson["estimate_x"] = px;
        msgJson["estimate_y"] = py;

        msgJson["rmse_x"] = RMSE_X;
        msgJson["rmse_y"] = RMSE_Y;
        msgJson["rmse_vx"] = RMSE_VX;
        msgJson["rmse_vy"] = RMSE_VY;

        auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";

        // Send it:
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
    });

    // ON HTTP REQUEST:
    // We don't need this since we're not using HTTP but if it's removed the program doesn't compile :-(

    h.onHttpRequest([](
        uWS::HttpResponse *res,
        uWS::HttpRequest req,
        char *data,
        size_t,
        size_t
    ) {
        const std::string s = "<h1>Hello world!</h1>";

        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // I guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    // ON CONNECTION:

    h.onConnection([&h](
        uWS::WebSocket<uWS::SERVER> ws,
        uWS::HttpRequest req
    ) {
        cout
            << endl
            << " Connected!" << endl
            << endl
            << "──────────────────────────────────────────────────────" << endl
            << endl;
    });

    // ON DISCONNECTION:

    h.onDisconnection([&h](
        uWS::WebSocket<uWS::SERVER> ws,
        int code,
        char *message,
        size_t length
    ) {
        ws.close();

        std::cout << "Disconnected!" << std::endl << std::endl << std::endl;
    });

    // START LISTENING:

    const int port = 4567;

    if (h.listen(port)) {

        cout
            << endl
            << " Listening on port " << port << "..." << endl
            << endl
            << "──────────────────────────────────────────────────────" << endl;

    } else {
        std::cerr << std::endl << "Failed to listen on port" << port << "!" << std::endl << std::endl;

        return -1;
    }

    h.run();
}
