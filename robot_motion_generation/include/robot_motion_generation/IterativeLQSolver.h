#ifndef ITERATIVELQSOLVER_H_
#define ITERATIVELQSOLVER_H_

#include "robot_motion_generation/LQSolver.h"
#include "robot_motion_generation/OCProblemFH.h"
#include <eigen3/Eigen/Dense>
#include <memory>

namespace motion{

    typedef Eigen::VectorXd Vector;
    typedef Eigen::MatrixXd Matrix;


struct IterativeLQSolverParams{
    // Line search parameters
    double epsilonMin;      // Min value for epsilon
    double epsilonInit;     // Initial epsilon at each iteration

    // Stopping Criteria:
    double relConverge;     // Relative improvement limit
    int maxIter;            // Max number of iteration

    IterativeLQSolverParams(double emin, double einit, double relc, int maxi) : epsilonMin(emin), epsilonInit(einit), relConverge(relc), maxIter(maxi){}
};


class IterativeLQSolver
{

private :

    unsigned int T;         // Time horizon
    double sample_time;     // for Euler
    int x_dim, u_dim;

    IterativeLQSolverParams solver_params;

    // Optimal control problem
    std::shared_ptr<OCProblemFH> prob;

    // Current LQ approximation
    LQProblem lqprob;

    // Solutions
    LQSolution nominal, nominal_new, deviations;

public :

    IterativeLQSolver(int time_horizon, int x_di, int u_di, double sampling_time,
                      std::shared_ptr<OCProblemFH> ocProb, const IterativeLQSolverParams & sparams,
                      const Vector & x_initial, const LQSolution & sol_ini);

    // Necessary for MPC (update the initial state)
    void setInitialState(const Vector & x_initial);

    double solve(LQSolution & sol);

private :
    void simulate(LQSolution & x_u_traj);
    void approximate(LQSolution & x_u_traj, LQProblem & lqproblem);
    double overallCost(LQSolution & x_u_traj);

};

}


#endif /* ITERATIVELQSOLVER_H_ */
