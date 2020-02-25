#include <homework/hw1/hw1.h>
#include <iostream>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

VectorXd
LinearMLESolve(vector<double> d_Lis, // vector of distance projected onto each landmark
               vector<double> sigmas, // vector of measurement noise std devs
               vector<VectorXd> G_p_Lis)  // vector of global position of each landmark
{
    // TODO: Use your Maximum Likelihood Estimate to solve the Weighted Linear Least Squares Problem
    assert(d_Lis.size() == sigmas.size() && sigmas.size() == G_p_Lis.size());
    Matrix<double, 2, 2> A;
    A << 0, 0, 0, 0;
    Matrix<double, 2, 1> b;
    b << 0, 0;
    for (int i = 0; i < G_p_Lis.size(); ++i) {
        VectorXd H_i_T = G_p_Lis[i].normalized();
        double sigma_i = sigmas[i];
        double d_Li = d_Lis[i];
        Matrix<double, 2, 2> Ai = H_i_T * (1 / (sigma_i * sigma_i)) * H_i_T.transpose();
        Matrix<double, 2, 1> bi = H_i_T * (1 / (sigma_i * sigma_i)) * d_Li;
        A += Ai;
        b += bi;
    }
    if (A.determinant() < 1e-6) {
        cerr << "Singular matrix formed. Cannot estimate position of robot" << endl;
    }
    Matrix<double, 2, 1> G_P_R = A.inverse() * b;
    return G_P_R;
}

double euclideanDistance(VectorXd p1, VectorXd p2) {
    VectorXd dist = p1 - p2;
    return dist.norm();
}

VectorXd
NonLinearLSSolve(vector<double> dists, // vector of distance to each landmark
                 vector<double> sigmas, // vector of measurement noise std devs
                 vector<VectorXd> G_p_Lis, // vector of global position of each landmark
                 VectorXd G_p_R_0, // Initial guess of robot position
                 double alpha, // Learning rate for Gauss-Newton
                 size_t n_iter)  // Number of iterations of Gauss-Newton to run
{
    // TODO: Use Gauss-Newton to iteratively solve the Weighted Non-Linear Least Squares Problem
    assert(dists.size() == sigmas.size() && sigmas.size() == G_p_Lis.size());
    VectorXd currentGuess = G_p_R_0;

    for (int iter = 0; iter < n_iter; ++iter) {
        Matrix<double, 2, 2> A;
        A << 0, 0, 0, 0;
        Matrix<double, 2, 1> b;
        b << 0, 0;
        for (int i = 0; i < G_p_Lis.size(); ++i) {
            double h_i_currentGuess = euclideanDistance(currentGuess, G_p_Lis[i]);
            VectorXd H_i_T = (currentGuess - G_p_Lis[i]) * (1 / h_i_currentGuess);
            double sigma_i = sigmas[i];
            double r_i = dists[i] - h_i_currentGuess;
            Matrix<double, 2, 2> Ai = H_i_T * (1 / (sigma_i * sigma_i)) * H_i_T.transpose();
            Matrix<double, 2, 1> bi = H_i_T * (1 / (sigma_i * sigma_i)) * r_i;
            A += Ai;
            b += bi;
        }
        if (A.determinant() < 1e-6) {
            cerr << "Singular matrix formed. Cannot estimate position of robot" << endl;
        }
        VectorXd change = A.inverse() * b;
        currentGuess = currentGuess + alpha * change;
    }
    return currentGuess;
}