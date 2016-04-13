#ifndef POINTMASSLINEAR_H_
#define POINTMASSLINEAR_H_

/*
 * N-dimensional 2nd order point robot tracking linear desired dynamics with quadratic cost.
 * State, x = [x_rob ; (x_d - x_rob)]
 * Cost_to_go = x^T Q x + u^T R u
 * Final_cost = x^T Q_track_f x
*/

#include <eigen3/Eigen/Dense>
#include <robot_motion_generation/OCProblemFH.h>

typedef Eigen::VectorXd Vector;
typedef Eigen::MatrixXd Matrix;

class PointMassLinear: public motion::OCProblemFH
{
private :
    Matrix A_, B_, Q_, Q_f_, R_;                          // State and cost matrices of the full compound state  x = [x_rob ; (x_d - x_rob)]
    Vector A_d_x_d_center_;

public :

    //Robot linear dynamics A_rob, B_rob and desired trajectory dynamics A_d
    // Q_track and Q_track_f are the weighting for the tracking error (x_d - x_rob)^T Q_track (x_d - x_rob)
    PointMassLinear(int time_horizon, const Matrix & A_rob, const Matrix & B_rob, const Matrix & A_d, const Vector & x_d_center, const Matrix & Q_track, const Matrix & Q_track_f, const Matrix & R)
        : OCProblemFH(time_horizon), R_(R) {

        size_t x_dim = A_rob.rows()*2;
        size_t u_dim = B_rob.cols();
        A_.resize(x_dim, x_dim);
        B_.resize(x_dim, u_dim);
        Q_.resize(x_dim, x_dim);
        Q_f_.resize(x_dim, x_dim);
        A_d_x_d_center_.resize(x_dim);

        A_.topLeftCorner(x_dim/2, x_dim/2) = A_rob;
        A_.topRightCorner(x_dim/2, x_dim/2).setZero();
        A_.bottomLeftCorner(x_dim/2, x_dim/2) = - A_d - A_rob;
        A_.bottomRightCorner(x_dim/2, x_dim/2).setZero();
        //A_.bottomLeftCorner(x_dim/2, x_dim/2).setZero();
        //A_.bottomRightCorner(x_dim/2, x_dim/2) = A_d + A_rob;

        Q_.setZero();
        Q_.bottomRightCorner(x_dim/2, x_dim/2) = Q_track;

        Q_f_.setZero();
        Q_f_.bottomRightCorner(x_dim/2, x_dim/2) = Q_track_f;

        A_d_x_d_center_ = A_d * x_d_center;

        //B_ << B_rob , Matrix::Zero(x_dim/2, u_dim) ;
        B_ << B_rob , -B_rob;
    }

    PointMassLinear(int time_horizon, const Matrix & A, const Matrix & B, const Vector & A_d_x_d_center, const Matrix & Q, const Matrix & Q_f, const Matrix & R)
        : OCProblemFH(time_horizon), A_(A), B_(B), A_d_x_d_center_(A_d_x_d_center), Q_(Q), Q_f_(Q_f), R_(R) {
    }

    PointMassLinear * clone() const { return new PointMassLinear(time_horizon_, A_, B_, A_d_x_d_center_, Q_, Q_f_, R_); }

    // Dynamics
    void dynamics(const Vector & x, const Vector & u, Vector & x_dot){
        x_dot = A_*x + B_*u;
        x_dot.tail(x.rows()/2) = x_dot.tail(x.rows()/2) + A_d_x_d_center_;
    }

    // Dynamics with 1st order state and control derivatives
    void dynamics(const Vector & x, const Vector & u, Matrix & A, Matrix & B) {
        A = A_;
        B = B_;
    }

    // Cost at sample k
    double cost(const Vector & x, const Vector & u, const int & k) {
        return ((x.transpose() * (Q_ * x)) + (u.transpose() * (R_ * u)))(0,0);
    }

    // Cost at sample T
    double finalCost(const Vector & x, const Vector & u) {
        return x.transpose() * Q_f_ * x;
    }

    // Cost at sample k with 2nd order state and control derivatives
    double cost(const Vector & x, const Vector & u, const int & k, double & q_0, Vector & q, Vector & r, Matrix & Q, Matrix & R) {
        R = 2*R_;
        Q = 2*Q_;
        r = 2 * R_ * u;
        q = 2 * Q_ * x;
        q_0 = cost(x,u,k);
        return q_0;
    }

    // Cost at sample T with 2nd order state derivatives
    double finalCost(const Vector & x, const Vector & u, double & q_0, Vector & q, Matrix & Q) {
        Q = 2*Q_f_;
        q = 2 * Q_f_ * x;
        q_0 = finalCost(x,u);
        return q_0;
    }
};


#endif /* POINTMASSLINEAR_H_ */
