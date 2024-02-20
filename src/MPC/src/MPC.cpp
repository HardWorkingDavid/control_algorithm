#include "MPC.h"

MPC::MPC(int nx, int nu, int t) : NX(nx), NU(nu), T(t) {}

vector<double> MPC::LMPC(MatrixXd xref, Vector3d x0, MatrixXd ref_delta, KinematicModel model)
{
    int NX = xref.rows();      // 3
    int NU = ref_delta.rows(); // 2
    int T = xref.cols() - 1;  // Horizon length. // 2

    // Define optimization variables.
    MatrixXd x(NX, T + 1); // 3, 3
    MatrixXd u(NU, T); // 1, 2
    // Store A matrices.
    vector<MatrixXd> A_vec;
    // Store B matrices.
    vector<MatrixXd> B_vec;

    // Initialize A and B matrices.
    for (int t = 0; t < T; ++t) {
        auto state_space = model.stateSpace(ref_delta(1, t), xref(2, t));
        A_vec.push_back(state_space[0]);
        B_vec.push_back(state_space[1]);
    }

    // Define the optimization problem.
    VectorXd cost(T + 1); // t + 1
    // List of constraint indices.
    vector<vector<int>> constraints;
    for (int t = 0; t < T; ++t) {
        cost(t) = (u.col(t) - ref_delta.col(t)).transpose() * R * (u.col(t) - ref_delta.col(t));

        if (t != 0) {
            cost(t) += (x.col(t) - xref.col(t)).transpose() * Q * (x.col(t) - xref.col(t));
        }

        MatrixXd A = A_vec[t];
        MatrixXd B = B_vec[t];
        // 3 6  6 9
        constraints.push_back({(t + 1) * NX, (t + 1) * NX + NX});  // State constraints.
        // 0 2 2 4
        constraints.push_back({t * NU, t * NU + NU});  // Input constraints.

        x.col(t + 1) = A * x.col(t) + B * (u.col(t) - ref_delta.col(t));
    }
    
    // Final state cost.
    cost(T) = (x.col(T) - xref.col(T)).transpose() * Qf * (x.col(T) - xref.col(T));

    // Set initial state.
    x.col(0) = x0; // 3

    VectorXd lower_bound(T * NU); // 4
    VectorXd upper_bound(T * NU); // 4

    for (int t = 0; t < T; t++)
    {
        lower_bound.segment(t * NU, NU) << -MAX_VEL, -MAX_STEER; // (1:2) (3:4)
        upper_bound.segment(t * NU, NU) << MAX_VEL, MAX_STEER; // (1:2) (3:4)
    }

    // Solve the optimization problem.
    OsqpEigen::Solver solver;

   // solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    // A: T * (NU + NX) x NX * (T + 1)
   // solver.data()->setNumberOfVariables(NX * (T + 1)); // 9
   // solver.data()->setNumberOfConstraints(T * (NU + NX));// 10

    solver.data()->setNumberOfVariables(NX * (T + 1)); //变量数n 9
    solver.data()->setNumberOfConstraints(T * (NU + NX));// 约束数m 10

    // P:NX * (T + 1) x NX * (T + 1)
    // Define the Hessian matrix dimension.
    int N = NX * (T + 1); // 9

    // Define and set the Hessian matrix.
    Eigen::SparseMatrix<double> P(N, N); // 9 x 9

    for (int t = 0; t < T; ++t) {
        for (int i = 0; i < NU; ++i) {
            P.coeffRef(t * NU + i, t * NU + i) = R(i, i);
        }
    }

    if (!solver.data()->setHessianMatrix(P)) return {};

    // Define the gradient vector (cost vector).
    VectorXd q(N); // 9 x 1

    for (int t = 0; t < T; ++t) {
        q.segment(t * NU, NU) = cost(t) * VectorXd::Ones(NU);
    }

    if (!solver.data()->setGradient(q)) return {};

    // Define the linear equality constraint matrix Aeq.
    int M = T * (NX + NU);  // Number of equality constraints. // M = 10
    Eigen::SparseMatrix<double> linearMatrix(M, N);  // 10 x 9
    for (int i = 0; i < M; i++)
    {
        for (int j = 0; j < N; j++)
        {
            if (i == j) linearMatrix.insert(i, j) = 1;
            else linearMatrix.insert(i, j) = 0;
        }
    }

    if (!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return {};
    // Define the equality constraint vector beq.
  //  VectorXd beq(M); // 9 x 1
  //  for (int i = 0; i < M; i++)
  //  {
 //       beq(i) = 0;
 //   }

    // You should populate Aeq and beq based on your state dynamics.

    // Set lower and upper bounds for variables and constraints.
    if (!solver.data()->setLowerBound(lower_bound)) {
        cerr << "Error setting lower bound." << endl;
        return {};
    }

    if (!solver.data()->setUpperBound(upper_bound)) {
        cerr << "Error setting upper bound." << endl;
        return {};
    }

    // Initialize the solver.
    if (!solver.initSolver()) return {};

    // Solve the problem.
    if (!solver.solve()) {
        cerr << "Error solving the optimization problem." << endl;
        return {};
    }
    VectorXd optimal_solution = solver.getSolution();

    // Extract optimal control inputs.
    vector<double> optimal_input;

    for (int t = 0; t < T; ++t) {
        VectorXd u_t = optimal_solution.segment(t * NU, NU); // (1,2) (3,4)
        optimal_input.push_back(u_t(0));  // Extract the velocity input.
    }
    cout << optimal_input[1] << endl;
    return optimal_input;
}