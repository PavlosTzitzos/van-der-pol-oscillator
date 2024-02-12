#pragma once

#include <iostream>
#include <random>
#include <complex>
#include <fstream>
#include <array>
#include <numeric>
#include <vector>
//#include "eigen/Eigen/Dense"

#include "gnuplot-iostream.h"

#define timeFinal 1000   /* Stop Value of performance function and Value function of AC */

/* FD and GD */
#define dtheta 0.001    /* Rate of change of theta angle between -0.1 <= dtheta <= -0.01 and 0.1 <= dtheta <= 0.01 */
#define hetta 0.01      /* Rate of change of the gradient descend */
#define timeStep 0.01   /* Time Step (must approach zero: dt->0) */
#define MAX_REPEATS 100 /* Maximum number of iterations for Gradient Descent Top algorithm */
#define version 1       /* Choose version of code: 1 for FD, 2 for SPSA, 3 will be added */

/* LQR */
#define N 100           /* Riccati number of iterations */
#define timeFinalLQR 5000 /* Final time for LQR performance */

/* Adaptive Control Parameters */
#define dt 0.01         /* Step parameter for AC */
#define gac 0.01        /* Step parameter gamma for AC */
#define k1 0.5          /* Parameter for AC between 0 < k1 < 1 */
#define k2 0.5          /* Parameter for Adaptive Control between 0 < k2 < 1 */

#define AC_END 1000     /* Creteria value for AC termination should be big enough */

/* SPSA Parameters */
/* c_k : */
#define betta 2.1       /* non-negative coefficient for SPSA */
#define gamma 0.1       /* non-negative coefficient for SPSA */

/* a_k : */
#define a 0.1           /* non-negative coefficient for SPSA */
#define A 0.1           /* non-negative coefficient for SPSA */
#define alpha 0.1       /* non-negative coefficient for SPSA */

/* D_k : */
#define p 0.5           /* propability for Delta_k bernoulli distribution for SPSA*/

#define SPSA_END 0.01   /* Creteria value for SPSA termination should be small enough */


/** An enum type.
*   Choose available algorithms.
*/
enum algorithms
{
    FD2,    // Finite Differences with 2 theta 
    FD3,    // Finite Differences with 3 theta 
    SPSA2,  // Simultaneous Perturbation Stochastic Approximation with 2 theta 
    SPSA3,  // Simultaneous Perturbation Stochastic Approximation with 3 theta 
    LQR,    // Linear Quadratic Regulator 
    AC      // Adaptive Controller 
};

/** Van der Pol State Space System of equations. This is used for FD and SPSA.
* @param x : Array of state space variables x
* @param u : Control signal
* @param k : System parameter k default is 1
* @param m : System parameter m default is 1
* @param c : System parameter c default is 1
* @return Array of derivatives of state space variables x
*/
std::array<double, 2> f(std::array<double, 2> x, double u_local, double k, double m, double c);

/** Van der Pol State Space System of equations. This is used for LQR.
* @param x : Array of state space variables x
* @param u : Control signal
* @return Array of derivatives of state space variables x
*/
std::array<double, 2> fLQR(std::array<double, 2> x, double u_local);

/** Van der Pol State Space System of equations. This is used for Adaptive Control only.
* @param x : Array of state space variables x
* @param u_local : Control signal
* @param thetaR : real theta
* @param m : System parameter m default is 1
* @return Array of derivatives of state space variables x
*/
std::array<double, 2> fAC(std::array<double, 2> x, double u_local, std::array<double, 2> thetaR, double m);

/** This is the control signal we apply to our system to guide it to our desired state. Version with 3 theta parameters.
* @param x : Array of state space variables x
* @param theta : Array of parameters(1,2,3)
* @return The control signal to be applied
*/
double u3(std::array<double, 2> x, std::array<double, 3> theta);

/** This is the control signal we apply to our system to guide it to our desired state. Version with 2 theta parameters.
* @param x : Array of state space variables x
* @param theta : Array of parameters(1,2)
* @return The control signal to be applied
*/
double u2(std::array<double, 2> x, std::array<double, 3> theta);

/** This is the control signal we apply to our system to guide it to our desired state. This is used for LQR.
* @param x : Array of state space variables x
* @param K : Array of parameters(1,2)
* @return The control signal to be applied
*/
double uLQR(std::array<double, 2> x, std::array<double, 2> K);

/** This is the control signal we apply to our system to guide it to our desired state. This is used for Adaptive Control.
* @param x : Array of state space variables x
* @param thetaApprox : Array of theta estimated parameters
* @return The control signal to be applied
*/
double uAC(std::array<double, 2> x, std::array<double, 2> thetaApprox, double m);

/** A performance function, used for FD and SPSA.
* @param x_old : Array of state space variables x
* @param theta : Array of theta estimated parameters
* @param theta_sel : Theta dimension. Must be 0 for 2 theta or 1 for 3 theta
* @param filename : File name to save data
* @return Total performance
*/
std::array<std::array<double, timeFinal + 1>, 3> performance(std::array<double, 2> x_old, std::array<double, 3> theta, int theta_sel, std::array<double, 3> systemParameters, std::string filename);

/** A performance function, used for LQR.
* @param x_old : Initial value of state space variables x
* @param K : K parameters
* @param filename : File name to save data
* @return Total performance
*/
std::array<std::array<double, timeFinalLQR + 1>, 3> performanceLQR(std::array<double, 2> x_old, std::array<double, 2> K, std::string filename);

/** Algebraic Riccati Equation Solver. Solved for a 2x2 matrices or a system of 4 equations.
* @param matA : A matrix (2x2)
* @param matB : B matrix (1x2)
* @param matQ : Q cost matrix (2x2)
* @param matPold : Previous P solution
* @param R : R value difault is 1
* @return The final time P=P(N) matrix
*/
std::array<double, 4> riccati2(std::array<double, 4> matA, std::array<double, 2> matB, std::array<double, 4> matQ, std::array<double, 4> matPold, double R);

/** LQR method using Algebraic Riccati Equation.
* @param x_old : Initial state
* @param q : Diagonal elements of Q 2x2 matrix
* @param r : R element
* @return The calculated parameters k1, k2
*/
std::array<double, 2> lqr(std::array<double, 2>x_old, double q, double r);

/** This is the Value function, which gives the minimum LQR cost-to-go.
* @param x : Initial state
* @param thetaA :Theta Approximation(Estimated) (theta hat)
* @param thetaR : Theta Real (theta we try to approximate)
* @param dthetaE : Slope of Theta Error (theta difference from real)
* @param m : System parameter default is 1
* @return The thetaA derivative
*/
std::array<std::array<double, timeFinal + 3>, 2> value(std::array<double, 2> x, std::array<double, 2> thetaA, std::array<double, 2> thetaR, std::array<double, 2> dThetaE, double m);

// Correct LQR implementation - basicly this works
std::array<double, timeFinalLQR + 1> lqrTop(std::array<double, 2> x0, std::array<double, 3> theta, std::array<double, 3> systemParameters, std::string st, algorithms algo_sel, std::array<double, 10> constantParameters);

/** Gradient Descent with Algorithm selection.
* @param x0 : Initial state
* @param theta :Theta theta
* @param systemParameters : System parameters (k,m,c) default is (1,1,1)
* @param constantParameters : Parameters that are constant during the execution of the function defaults can be setted in line 17 - Used for Sensitivity Analysis
* @return Array of performances
*/
std::array<std::array<double, MAX_REPEATS>, 2> gradientDescent(std::array<double, 2> x0, std::array<double, 3> theta, std::array<double, 3> systemParameters, std::string st, algorithms algo_sel, std::array<double, 10> constantParameters);

/**Function for Sensitivity Analysis - NOT ready YET
* Varies different constants of the algorithms except the system paramters (m,c,k) and the initial conditions.
* @param xInit : Initial x state default is (1,1)
* @param theta : theta initial (default is 1,1,0)
* @param sysPar : System Parameters (default is m=c=k=1)
* @param max : Maximum value of the parameters - final value (default is 1)
* @param step : Step the parameters (default is 1)
* @param min : Minimum value of the parameters - Starting value (default is 0)
*/
void sensitivityAnalysis(std::array<double, 2> xInit, std::array<double, 3> theta, std::array<double, 3> sysPar, double max, double step, double min);
