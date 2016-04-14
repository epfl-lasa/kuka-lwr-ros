#ifndef OCPROBLEMFH_H_
#define OCPROBLEMFH_H_

/*
 * This abstract class defines an arbitrary optimal control finite horizon problem given by:
 * - time horizon
 * - dynamics (including 1st order derivatives)
 * - instantaneous cost function (including 2nd order derivatives)
*/

#include <eigen3/Eigen/Dense>

namespace motion{

    typedef Eigen::VectorXd Vector;
    typedef Eigen::MatrixXd Matrix;

class OCProblemFH
{
protected :
    int time_horizon_;
    OCProblemFH(int time_horizon) {time_horizon_ = time_horizon;}

public :
    virtual OCProblemFH * clone() const = 0;

    // Dynamics
    virtual void dynamics(const Vector & x, const Vector & u, Vector & x_dot) = 0;

    // Dynamics with 1st order state and control derivatives
    virtual void dynamics(const Vector & x, const Vector & u, Matrix & A, Matrix & B) = 0;

    // Cost at sample k
    virtual double cost(const Vector & x, const Vector & u, const int & k) = 0;

    // Cost at sample T
    virtual double finalCost(const Vector & x, const Vector & u) = 0;

    // Cost at sample k with 2nd order state and control derivatives
    virtual double cost(const Vector & x, const Vector & u, const int & k, double & q_0, Vector & q, Vector & r, Matrix & Q, Matrix & R) = 0;

    // Cost at sample T with 2nd order state derivatives
    virtual double finalCost(const Vector & x, const Vector & u, double & q_0, Vector & q, Matrix & Q) = 0;
};

}


#endif /* OCPROBLEMFH_H_ */
