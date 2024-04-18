#include "LinearKalmanFilter.h"
#include "Matrix.h"
LinearKalmanFilter::LinearKalmanFilter(Matrix X, Matrix U, Matrix P, Matrix F, Matrix G, Matrix R, Matrix Q) {
    state.X = X;
    state.U = U;
    state.P = P;
    state.F = F;
    state.G = G;
    state.R = R;
    state.Q = Q;
    calculate_initial_values();
}

void LinearKalmanFilter::predict_state() {
    state.X = (state.F * state.X) + (state.G * state.U);
}

void LinearKalmanFilter::estimate_state(Matrix measurement) {
    state.X = state.X + state.K * (measurement - state.H * state.X);
}

void LinearKalmanFilter::calculate_kalman_gain() {
    state.K = state.P * state.H.T() * (state.H * state.P * state.H.T() + state.R).inv();
}

void LinearKalmanFilter::covariance_update() {
    int n = state.X.getRows();
    state.P = ((Matrix::ident(n) - state.K * state.H) * state.P * ((Matrix::ident(n) - state.K * state.H).T())) + state.K * state.R * state.K.T();
}

void LinearKalmanFilter::covariance_extrapolate() {
    state.P = state.F * state.P * state.F.T() + state.Q;
}

void LinearKalmanFilter::calculate_initial_values() {
    predict_state();
    covariance_extrapolate();
}

Matrix LinearKalmanFilter::iterate(Matrix measurement, Matrix control, Matrix F, Matrix G, Matrix H) {
    state.F = F;
    state.G = G;
    state.H = H;
    state.U = control;
    std::cout << "\nInitial State: \n" << std::endl;
    displayAll(measurement);
    predict_state();
    std::cout << "\nPredicted State: \n" << std::endl;
    displayAll(measurement);
    covariance_extrapolate();
    std::cout << "\nExtrapolated Covariance: \n" << std::endl;
    displayAll(measurement);
    calculate_kalman_gain();
    std::cout << "\nKalman Gain: \n" << std::endl;
    displayAll(measurement);
    estimate_state(measurement);
    std::cout << "\nEstimated State: \n" << std::endl;
    displayAll(measurement);
    covariance_update();
    std::cout << "\nUpdated Covariance: \n" << std::endl;
    displayAll(measurement);
    return state.X;
}

void LinearKalmanFilter::displayAll(Matrix meas) {
    std::cout << "X - State Vector: " << std::endl;
    state.X.disp();
    std::cout << "U - Control Vector: " << std::endl;
    state.U.disp();
    std::cout << "Measurement Vector: " << std::endl;
    meas.disp();
    std::cout << "P - Estimate Covariance Matrix: " << std::endl;
    state.P.disp();
    std::cout << "F - State Transition Matrix: " << std::endl;
    state.F.disp();
    std::cout << "G - Control Matrix: " << std::endl;
    state.G.disp();
    std::cout << "R - Measurement Covariance Matrix: " << std::endl;
    state.R.disp();
    std::cout << "K - Kalman Gain Matrix" << std::endl;
    state.K.disp();
    std::cout << "Q - Process Noise Matrix: " << std::endl;
    state.Q.disp();
    std::cout << "H - Observation Matrix: " << std::endl;
    state.H.disp();
}