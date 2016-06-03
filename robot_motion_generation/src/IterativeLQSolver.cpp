/*
 * CDDynamics.cpp
 *
 *  Created on: May 29, 2012
 *      Author: Seungsu KIM
 */

#include "robot_motion_generation/IterativeLQSolver.h"
#include <eigen3/Eigen/Eigenvalues>
#include <iostream>

namespace motion{

IterativeLQSolver::IterativeLQSolver(int time_horizon, int x_di, int u_di, double sampling_time, std::shared_ptr<OCProblemFH> ocProb, const IterativeLQSolverParams & sparams, const Vector & x_initial, const LQSolution & sol_ini) :
    lqprob(time_horizon, x_dim, u_dim), nominal(time_horizon, x_dim, u_dim),
    nominal_new(time_horizon, x_dim, u_dim), deviations(time_horizon, x_dim, u_dim),
    T(time_horizon), solver_params(sparams), sample_time(sampling_time), x_dim(x_di), u_dim(u_di)
{
    prob = ocProb;

    nominal.x[0] = x_initial;
    nominal.ff = sol_ini.ff;
    nominal.K = sol_ini.K;
}

void IterativeLQSolver::setInitialState(const Vector & x_initial) {
    nominal.x[0] = x_initial;
    nominal_new.x[0] = x_initial;
}


double IterativeLQSolver::solve(LQSolution & sol) {

    double cost = 0, cost_new, epsilon;
    Vector du(u_dim), dx(x_dim);
    du.setZero();
    dx.setZero();

    epsilon = solver_params.epsilonInit;

    for (int iter = 0 ; iter < solver_params.maxIter ; iter ++) {
        // Simulate system and and approximate LQ problem

        if (iter == 0) {
            simulate(nominal);
            cost = overallCost(nominal);
            //std::cout << "Initial cost: " << cost << std::endl;
            //std::cout << "Initial traj: " << nominal.x[0] << std::endl << nominal.x[1] << std::endl << nominal.x[2] << std::endl;
        }

        approximate(nominal,lqprob);

        std::cout << "Iteration: " << iter << std::endl;

        // Solve local LQ problem and get optimal control deviations
        LQSolver::solve(lqprob, deviations);

        while (true) {
            // Update nominal trajectory performing a line search algorithm
            for (size_t i = 0; i < T-1 ; i ++) {
                du = epsilon * deviations.ff[i] + deviations.K[i]*dx;
                dx = lqprob.A[i]*dx + lqprob.B[i]*du;
                nominal_new.ff[i] = nominal.ff[i] + du;
            }

            simulate(nominal_new);
            cost_new = overallCost(nominal_new);
            //std::cout << "Candidate traj: " << nominal_new.x[0] << std::endl << nominal_new.x[1] << std::endl;
            //std::cout << "Cost: " << cost  << " , cost_new: " << cost_new << " , epsilon: " << epsilon << std::endl;

            //std::cout << "epsilonMin: " << solver_params.epsilonMin  << " , relConverge: " << solver_params.relConverge << " , maxIter: " << solver_params.maxIter << std::endl;

            if (cost_new < cost) {
                nominal.ff = nominal_new.ff;
                nominal.x = nominal_new.x;
                nominal.K = nominal_new.K;

                // Reset epsilon for next iteration
                epsilon = solver_params.epsilonInit;

                // Check for convergence
                if ( (fabs(cost_new - cost)/cost_new) < solver_params.relConverge ) {
                    //std::cout << "Improvement too small" << std::endl;
                    cost = cost_new;
                    goto finished;
                }
                // Set new cost
                cost = cost_new;
                break;

            } else {
                epsilon = epsilon / 2;

                if (epsilon < solver_params.epsilonMin) {
                    std::cout << "Converged through epsilonMin" << std::endl;
                    goto finished;
                }
            }
        }
    }

    finished:
    sol.x = nominal.x;
    sol.ff = nominal.ff;
    sol.K = nominal.K;

    //std::cout << "Final cost: " << cost<< std::endl;
    //std::cout << "Final traj: " << nominal.x[0] << std::endl << nominal.x[1] << std::endl << nominal.x[2] << std::endl;
    return cost;
}

// Simulates the control trajectory stored in x_u_traj.ff to obtain a state trajectory
void IterativeLQSolver::simulate(LQSolution & x_u_traj) {
    Vector x_dot;

    for (size_t i = 0; i < T-1 ; i ++) {
        // Simulate dynamics with Euler + first-order approximation
        prob->dynamics(x_u_traj.x[i], x_u_traj.ff[i], x_dot);
        x_u_traj.x[i+1] = x_u_traj.x[i] + sample_time * x_dot;
    }
}

// LQ approximation around the given nominal trajectory of states and controls x_u_traj
void IterativeLQSolver::approximate(LQSolution & x_u_traj, LQProblem & lqproblem) {
    Matrix f_dx(x_dim, x_dim), f_du(x_dim, u_dim);

    for (size_t i = 0; i < T-1 ; i ++) {
        // Simulate dynamics with Euler + first-order approximation
        prob->dynamics(x_u_traj.x[i], x_u_traj.ff[i], f_dx, f_du);
        lqproblem.A[i] = Matrix::Identity(x_dim, x_dim) + sample_time * f_dx;
        lqproblem.B[i] = sample_time * f_du;

        // Cost-to-go + quadratic approximation
        prob->cost(x_u_traj.x[i],x_u_traj.ff[i], i, lqproblem.q0[i], lqproblem.q[i], lqproblem.r[i], lqproblem.Q[i], lqproblem.R[i]);
        lqproblem.q0[i] = sample_time * lqproblem.q0[i];
        lqproblem.q[i] = sample_time * lqproblem.q[i];
        lqproblem.r[i] = sample_time * lqproblem.r[i];
        lqproblem.Q[i] = sample_time * lqproblem.Q[i];
        lqproblem.R[i] = sample_time * lqproblem.R[i];
    }
    // Final cost
    prob->finalCost(x_u_traj.x[T-1],x_u_traj.ff[T-1], lqproblem.q0[T-1], lqproblem.q[T-1], lqproblem.Q[T-1]);
}

// Compute overall cost analytically
double IterativeLQSolver::overallCost(LQSolution & x_u_traj) {
    double cost = 0;

    for (size_t i = 0; i < T-1 ; i ++) {
        // Cost-to-go
        cost = cost + sample_time * prob->cost(x_u_traj.x[i],x_u_traj.ff[i], i);
    }
    // Final cost
    cost = cost + prob->finalCost(x_u_traj.x[T-1],x_u_traj.ff[T-1]);

    return cost;
}



}
