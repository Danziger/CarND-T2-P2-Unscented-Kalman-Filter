#include "Tools.h"

#include <iostream>
#include <sstream>
#include <string>
#include <iomanip>


#define C_RED "\033[38;5;196m"
#define C_RST "\033[0;m"


using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;


Tools::Tools() {}
Tools::~Tools() {}


VectorXd Tools::calculateRMSE(
    const vector<VectorXd> &estimations,
    const vector<VectorXd> &groundTruth
) {

    const unsigned int estimationsCount = estimations.size();

    VectorXd rmse(4);

    rmse << 0, 0, 0, 0;

    // Check the validity of the following inputs:
    // - The estimation vector size should not be zero.
    // - The estimation vector size should equal ground truth vector size.

    if (estimationsCount == 0 || estimationsCount != groundTruth.size()) {
        cout << "Invalid estimation or ground_truth data" << endl;

        return rmse;
    }

    // Accumulate squared residuals:

    for (unsigned int i = 0; i < estimationsCount; ++i) {

        const VectorXd residual = estimations[i] - groundTruth[i];

        // Coefficient-wise multiplication:
        rmse += (residual.array().pow(2)).matrix();
    }

    // Calculate the mean:
    rmse = rmse / estimationsCount;

    // Calculate the squared root and return final result:
    return rmse.array().sqrt();

}


double Tools::normalizeAngle(double angle) {
    const int times = angle / M_PI;

    return angle - M_PI * (times >= 0 ? ceil(times) : floor(times));
}

double Tools::prompt(string name, string unit, double def, double min, double max) {
    string raw;
    double value;

    while (true) {
        cout << "   Enter " << name << " (default = " << def << ") = ";

        getline(cin, raw);

        cout << endl;

        if (raw.length() == 0) {
            cout << "   " << name << " = " << setfill(' ') << setw(5) << def << " " << unit << " (default)" << endl << endl;

            return def; // Return default value if emtpy:
        }

        try {
            value = stod(raw);

            if (value < min || value > max) {
                cerr << C_RED << "   " << value << " not in range [" << min << ", " << max << "]." << C_RST << endl;
            } else {
                cout << "   " << name << " = " << setfill(' ') << setw(5) << value << " " << unit << (value == def ? " (default)" : "") << endl << endl;

                return value;
            }
        } catch (std::invalid_argument e) {
            cerr << C_RED << "   Invalid or not numeric input." << C_RST << endl;
        }

        cout << endl;
    }

}
