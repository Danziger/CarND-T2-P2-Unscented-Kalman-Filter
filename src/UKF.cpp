#include "UKF.h"

#include <iostream>


#define EPS 0.0001
#define NEPS -EPS
#define KEEP_IN_RANGE(n) (n < NEPS ? NEPS : (n < EPS ? EPS : n))
#define ATAN00 atan2(EPS, EPS)
#define N_X 5
#define N_A 7 // N_X + 2
#define N_SIGMA 15 // 2 * N_A + 1
#define LAMBDA -4 // 3 - N_A

using Eigen::MatrixXd;
using Eigen::VectorXd;



// PRIVATE:


MatrixXd UKF::calculateAugmentedSigmaPoints() {
    std::cout << "START calculateAugmentedSigmaPoints" << std::endl;

    // Create augmented covariance matrix and sigma point matrix:
    MatrixXd P_aug = MatrixXd(N_A, N_A);
    MatrixXd Xsig_aug = MatrixXd(N_A, N_SIGMA);

    // Create augmented mean state:
    VectorXd mean = VectorXd(N_A);
    mean << x_, 0, 0;

    // Fill augmented covariance matrix:
    P_aug.fill(0.0); // Not needed?
    P_aug.topLeftCorner(N_X, N_X) = P_;

    // P_aug.block(0, n_x, n_aug, n_aug - n_x) = MatrixXd::Zero(n_aug, n_aug - n_x);
    // P_aug.block(n_x, 0, n_aug - n_x, n_x) = MatrixXd::Zero(n_aug - n_x, n_x);

    P_aug(5, 5) = std_a_ * std_a_;
    P_aug(6, 6) = std_yawdd_ * std_yawdd_;

    std::cout << "P_aug" << std::endl << std::endl << P_aug << std::endl << std::endl;


    // Create square root matrix:
    const MatrixXd L = P_aug.llt().matrixL();

    // Create augmented sigma points:

    const float lambda_nx_sqrt = sqrt(LAMBDA + N_A);

    const MatrixXd matrix_sqrt_term = lambda_nx_sqrt * L;
    const MatrixXd replicated_mean = mean.replicate(1, N_A);

    Xsig_aug.col(0) = mean;
    Xsig_aug.block(0, 1, N_A, N_A) = replicated_mean + matrix_sqrt_term;
    Xsig_aug.block(0, N_A + 1, N_A, N_A) = replicated_mean - matrix_sqrt_term;

    std::cout << "Xsig_out" << std::endl << std::endl << Xsig_aug << std::endl << std::endl;

    return Xsig_aug;
}


MatrixXd UKF::predictSigmaPoints(const double dt, const MatrixXd Xsig_aug) {
    std::cout << "START predictSigmaPoints" << std::endl;

    // Create matrix with predicted sigma points as columns:
    MatrixXd Xsig_pred = MatrixXd(N_X, N_SIGMA);

    // Precalculate the ddt term which is common for all the iterations:
    const double ddt = dt * dt / 2.0;

    // Calculate column by column:

    for (int i = 0; i < N_SIGMA; ++i) {
        // Get individual components:

        const VectorXd col = Xsig_aug.col(i);

        const double px = col(0);
        const double py = col(1);
        const double vel = col(2);
        const double yaw = col(3);
        const double yaw_rate = col(4);
        const double n_acc = col(5);
        const double n_yaw = col(6);

        // Calculate noise components:

        const double dt_n_acc = dt * n_acc;
        const double dt_n_yaw = dt * n_yaw;

        const double dt2_n_acc_by2 = ddt * n_acc;
        const double dt2_n_yaw_by2 = ddt * n_yaw;

        // Calculate prediction, avoiding division by 0:

        VectorXd prediction = VectorXd(N_X);

        if (fabs(yaw_rate) < EPS) {
            const double dp = vel * dt + dt2_n_acc_by2;

            prediction <<
                px + cos(yaw) * dp,
                py + sin(yaw) * dp,
                vel + dt_n_acc,
                yaw + yaw_rate * dt + dt2_n_yaw_by2,
                yaw_rate + dt_n_yaw;
        } else {
            const double a = vel / yaw_rate;
            const double b = yaw_rate * dt;
            const double c = b + yaw;
            const double yaw_cos = cos(yaw);
            const double yaw_sin = sin(yaw);

            // std::cout << "v = " << vel << "yaw = " << yaw_rate << "a = " << a << std::endl;

            prediction <<
                px + a * (sin(c) - yaw_sin) + yaw_cos * dt2_n_acc_by2,
                py + a * (yaw_cos - cos(c)) + yaw_sin * dt2_n_acc_by2,
                vel + dt_n_acc,
                yaw + b + dt2_n_yaw_by2,
                yaw_rate + dt_n_yaw;
        }

        // Set column i:
        Xsig_pred.col(i) = prediction;
    }

    return Xsig_pred;
}


// PUBLIC:


UKF::UKF() {}

UKF::~UKF() {}


void UKF::initStateCovarianceMatrix(const MatrixXd &P) {
    P_ = P;
}


void UKF::initState(
    const float px,
    const float py,
    const float v_abs,
    const float yaw_angle,
    const float yaw_rate
) {
    x_ = VectorXd(5);

    x_ << px, py, v_abs, yaw_angle, yaw_rate;

    std::cout << "END initState" << std::endl;
}


void UKF::initNoise(
    const float std_a,
    const float std_yawdd,
    const float std_laspx,
    const float std_laspy,
    const float std_radr,
    const float std_radphi,
    const float std_radrd
) {
    std_a_ = std_a;
    std_yawdd_ = std_yawdd;
    std_laspx_ = std_laspx;
    std_laspy_ = std_laspy;
    std_radr_ = std_radr;
    std_radphi_ = std_radphi;
    std_radrd_ = std_radrd;
}


VectorXd UKF::getCurrentState() {
    std::cout << "START getCurrentState" << std::endl;

    return x_;
}


void UKF::predict(const double dt) {
    std::cout << "START predict" << std::endl;

    // Calculate sigma points and predict sigma points at timestep k + 1:

    const MatrixXd Xsig_pred = predictSigmaPoints(dt, calculateAugmentedSigmaPoints());


    // Predict mean state vector and state covariance matrix:

    // Create vector for predicted state:
    VectorXd x = VectorXd(N_X);

    // Create covariance matrix for prediction:
    MatrixXd P = MatrixXd(N_X, N_X);

    // Calculate weights and state mean in a single loop and without an additional
    // vector for weights:

    const double divisor = LAMBDA + N_A; // TODO: This could be 0
    const double weight0 = LAMBDA / divisor;
    const double weightN = 0.5 / divisor;

    x = weight0 * Xsig_pred.col(0);

    for (int i = 1; i < N_SIGMA; ++i) {
        x += weightN * Xsig_pred.col(i);
    }

    // Predict state covariance matrix:

    // TODO: Could be DRY-ed
    // TODO: Normalization stuff could go to tools as an inline function

    VectorXd diff = Xsig_pred.col(0) - x;

    // Normalize angle in range [-PI, PI]:

    const float yaw = diff(3);
    const int times = yaw / M_PI;

    diff(3) = yaw - M_PI * (times >= 0 ? ceil(times) : floor(times));

    // Update predicted covariance matrix:

    P = weight0 * diff * diff.transpose();

    for (int i = 1; i < N_SIGMA; ++i) {
        VectorXd diff = Xsig_pred.col(i) - x;

        // Normalize angle in range [-PI, PI]:

        const float yaw = diff(3);
        const int times = yaw / M_PI;

        diff(3) = yaw - M_PI * (times >= 0 ? ceil(times) : floor(times));

        // Update predicted covariance matrix:

        P += weightN * diff * diff.transpose();
    }

    // Update state vector and state covariance matrix:
    Xsig_pred_ = Xsig_pred;
    x_ = x;
    P_ = P;
}


void UKF::updateLidar(const VectorXd &z) {
    std::cout << "START updateLidar" << std::endl;

    /**
    TODO:

    Complete this function! Use lidar data to update the belief about the object's
    position. Modify the state vector, x_, and covariance, P_.

    You'll also need to calculate the lidar NIS.
    */
}


void UKF::updateRadar(const VectorXd &z) {
    std::cout << "START updateRadar" << std::endl;

    /**
    TODO:

    Complete this function! Use radar data to update the belief about the object's
    position. Modify the state vector, x_, and covariance, P_.

    You'll also need to calculate the radar NIS.
    */

    const short N_Z = 3;

    // Calculate weights:

    const double divisor = LAMBDA + N_A; // TODO: This could be 0
    const double weight0 = LAMBDA / divisor;
    const double weightN = 0.5 / divisor;

    // Create matrix for sigma points in measurement space:
    MatrixXd Zsig = MatrixXd(N_Z, N_SIGMA);

    // Mean predicted measurement:
    VectorXd z_pred = VectorXd(N_Z);

    // Measurement covariance matrix S:
    MatrixXd S = MatrixXd(N_Z, N_Z);

    // Transform sigma points into measurement space:

    for (int i = 0; i < N_SIGMA; ++i) {
        VectorXd col = Xsig_pred_.col(i);

        const double px = col(0);
        const double py = col(1);
        const double v = col(2);
        const double yaw = col(3);

        const double rho  = sqrt(px * px + py * py);
        const double phi = px == 0 && py == 0 ? ATAN00 : atan2(py, px);

        Zsig.col(i) << rho, phi, v * (px * cos(yaw) + py * sin(yaw)) / KEEP_IN_RANGE(rho); 
    }

    // Calculate mean predicted measurement:

    z_pred = weight0 * Zsig.col(0);

    for (int i = 1; i < N_SIGMA; ++i) {
        z_pred += weightN * Zsig.col(i);
    }

    // Calculate measurement covariance matrix S:

    VectorXd diff = Zsig.col(0) - z_pred;

    // Normalize angle in range [-PI, PI]:

    double phi = diff(1);
    int times = phi / M_PI;

    diff(1) = phi - M_PI * (times >= 0 ? ceil(times) : floor(times));

    S = weight0 * diff * diff.transpose();

    for (int i = 1; i < N_SIGMA; ++i) {
        VectorXd diff = Zsig.col(i) - z_pred;

        // Normalize angle in range [-PI, PI]:

        const double phi = diff(1);
        const int times = phi / M_PI;

        diff(1) = phi - M_PI * (times >= 0 ? ceil(times) : floor(times));

        S += weightN * diff * diff.transpose();
    } 

    S(0, 0) += std_radr_ * std_radr_;
    S(1, 1) += std_radphi_ * std_radphi_;
    S(2, 2) += std_radrd_ * std_radrd_;





    // Update state mean and state covariance matrix:

    // TODO: Called "estimate" in the previous project

    // Create matrix for cross correlation Tc:
    MatrixXd Tc = MatrixXd(N_X, N_Z);


    // Calculate cross correlation matrix

    Tc.fill(0.0);

    for (int i = 0; i < N_SIGMA; ++i) {
        // Calculate diff vectors:

        VectorXd diffX = Xsig_pred_.col(i) - x_;
        VectorXd diffZ = Zsig.col(i) - z_pred;

        // Normalize angle in range [-PI, PI] for diffX:

        const double yaw = diffX(3);
        const int timesYaw = yaw / M_PI;

        diffX(3) = yaw - M_PI * (timesYaw >= 0 ? ceil(timesYaw) : floor(timesYaw));

        // Normalize angle in range [-PI, PI] for diffZ:

        const double phi = diffZ(1);
        const int timesPhi = phi / M_PI;

        diffZ(1) = phi - M_PI * (timesPhi >= 0 ? ceil(timesPhi) : floor(timesPhi));

        // Update Tc:
        // TODO: FIX THIS

        Tc += (i == 0 ? weight0 : weightN) * diffX * diffZ.transpose();
    }

    // Calculate Kalman gain K:

    MatrixXd K = Tc * S.inverse();

    // Update state mean and covariance matrix:

    diff = z - z_pred;

    // Normalize angle in range [-PI, PI] for diffX:

    phi = diff(1);
    times = phi / M_PI;

    diff(1) = phi - M_PI * (times >= 0 ? ceil(times) : floor(times));


    // Update state mean:
    x_ += K * diff;

    // Update covariance matrix:
    P_ -= K * S * K.transpose();
}
