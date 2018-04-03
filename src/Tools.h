#ifndef TOOLS_H_
#define TOOLS_H_


#include "Eigen/Dense"

#include <vector>


using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;


class Tools {

public:

    Tools();
    virtual ~Tools();

    /**
    * Helper method to calculate RMSE.
    */
    static VectorXd calculateRMSE(
        const vector<VectorXd> &estimations,
        const vector<VectorXd> &groundTruth
    );

    /**
    * Helper method to normalize angles that will hopefully be inlined by the compiler.
    */
    static double normalizeAngle(double angle);


    // PROMPTS:

    /**
    * Prompts the user for a double value and validates it is in the [min, max] interval.
    */
    static double prompt(string message, string unit, double def, double min, double max);

    /**
    * Prompts the user for a char value and validates it is one of the options inside options.
    */
    // static char prompt(string message, char def, vector<char> options);

    /**
    * Prompts the user for an int value and validates it is in the [min, max] interval.
    */
    // static int prompt(string message, int def, int min, int max);
};

// TODO: https://stackoverflow.com/questions/18114550/create-my-own-pre-name-like-std-in-c

#endif /* TOOLS_H_ */
