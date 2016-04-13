#ifndef LQSOLVER_H_
#define LQSOLVER_H_


#include <eigen3/Eigen/Dense>
#include <iostream>

namespace motion{

    typedef Eigen::VectorXd Vector;
    typedef Eigen::MatrixXd Matrix;

struct LQProblem {
    // Initial state
    Vector x0;

    // Dynamics
    std::vector<Matrix> A, B;

    // Cost
    std::vector<Matrix> Q, R;
    std::vector<Vector> q, r;
    std::vector<double> q0;

    LQProblem(int T, int x_dim, int u_dim) {
        // resize variables
        A.resize(T);
        B.resize(T);
        Q.resize(T);
        R.resize(T);
        q.resize(T);
        r.resize(T);
        q0.resize(T);

        x0.resize(x_dim);
        for (int i=0; i<T; i++) {
            A[i].resize(x_dim, x_dim);
            A[i].setZero();
            B[i].resize(x_dim, u_dim);
            B[i].setZero();
            Q[i].resize(x_dim, x_dim);
            Q[i].setZero();
            R[i].resize(x_dim, u_dim);
            R[i].setZero();
            q[i].resize(x_dim);
            q[i].setZero();
            r[i].resize(u_dim);
            r[i].setZero();
        }
    }
};

struct LQSolution {
    // Optimal linear feedback solution
    std::vector<Vector> ff;
    std::vector<Vector> x;
    std::vector<Matrix> K;

    LQSolution(int T, int x_dim, int u_dim) {
        ff.resize(T);
        K.resize(T);
        x.resize(T);

        for (int i=0; i<T; i++) {
            ff[i].resize(u_dim);
            ff[i].setZero();
            K[i].resize(u_dim, x_dim);
            K[i].setZero();
            x[i].resize(x_dim);
            x[i].setZero();
        }
    }
};


class LQSolver
{

public :
    static double solve(const LQProblem & prob, LQSolution & sol) {
        return solve(prob.A.size(), prob.A, prob.B, prob.Q, prob.R, prob.q, prob.r, prob.q0, sol.ff, sol.K);
    }

    // SOLVER: returns expected cost
    static double solve(int time_horizon, const std::vector<Matrix>& A, const std::vector<Matrix>& B, const std::vector<Matrix>& Q, const std::vector<Matrix>& R,
                        const std::vector<Vector>& q, const std::vector<Vector>& r, const std::vector<double>& q0, std::vector<Vector>& ff, std::vector<Matrix>& K) {
        size_t x_dim = A[0].rows();
        size_t u_dim = B[0].cols();

        // Cost to go function = x^T W x + x^T w + w_0
        Matrix W(x_dim, x_dim);
        Vector w(x_dim);
        double w0;
        W << Q[time_horizon-1];
        w << q[time_horizon-1];
        w0 = q0[time_horizon-1];

        // Shortcuts
        Vector g(u_dim);
        Matrix G(u_dim, x_dim);
        Matrix H(u_dim, u_dim);
        Matrix H_inverse(u_dim, u_dim);

        // Bellman recursion
        for (int i=time_horizon-2 ; i >= 0 ; i --) {

            // Update shortcuts
            g = r[i] + B[i].transpose() * w;
            G = B[i].transpose()*W*A[i];
            H = R[i] + B[i].transpose()*W*B[i];

            // Compute optimal control
            Eigen::EigenSolver<Matrix> es(H, true);

            // TODO: check for negative (or too small) eigenvalues and correct them
            H_inverse = es.eigenvectors().real() * es.eigenvalues().real().asDiagonal().inverse()
                    * es.eigenvectors().real().transpose();

            ff[i] = - H_inverse * g;
            K[i] = - H_inverse * G;

            // Update cost-to-go
            W = Q[i] + A[i].transpose()*W*A[i] + K[i].transpose()*H*K[i] + K[i].transpose()*G + G.transpose()*K[i];
            w = q[i] + A[i].transpose()*w + K[i].transpose()*H*ff[i] + K[i].transpose()*g + G.transpose()*ff[i];
            w0 = q0[i] + w0 + 0.5*ff[i].transpose()*H*ff[i] + ff[i].transpose()*g;
        }

        return w0;
    }
};

}


#endif /* LQSOLVER_H_ */
