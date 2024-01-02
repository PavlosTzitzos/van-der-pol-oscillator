// VanDerPolOscillator.cpp : This file contains the 'main' function. Program execution begins and ends there.
// LaTeX Comments Extension : https://github.com/kindermannhubert/VsTeXCommentsExtension

#include <iostream>
#include <random>
#include <complex>
#include <fstream>
#include <array>
#include <numeric>
#include <vector>

//#include "gnuplot-iostream.h"

#define dtheta 0.001    /* Rate of change of theta angle between -0.1 <= dtheta <= -0.01 and 0.1 <= dtheta <= 0.01 */
#define hetta 0.01      /* Rate of change of the gradient descend */
#define timeStep 0.01   /* Time Step (must approach zero: dt->0) */
#define timeFinal 100   /* Stop Value of performance function */
#define MAX_REPEATS 100 /* Maximum number of iterations for Gradient Descent Top algorithm */
#define version 1       /* Choose version of code: 1 for FD, 2 for SPSA, 3 will be added */

/* LQR */
#define N 100           /* Riccati number of iterations */

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

/// <summary>
/// Choose available algorithms
/// </summary>
enum algorithms
{
    /// <summary>
    /// Finite Differences with 2 theta
    /// </summary>
    FD2,
    /// <summary>
    /// Finite Differences with 3 theta
    /// </summary>
    FD3,
    /// <summary>
    /// Simultaneous Perturbation Stochastic Approximation with 2 theta
    /// </summary>
    SPSA2,
    /// <summary>
    /// Simultaneous Perturbation Stochastic Approximation with 3 theta
    /// </summary>
    SPSA3,
    /// <summary>
    /// Linear Quadratic Regulator
    /// </summary>
    LQR,
    /// <summary>
    /// Adaptive Controller
    /// </summary>
    AC
};

/// <summary>
/// Van der Pol State Space System of equations. This is used for FD , SPSA and LQR.
/// </summary>
/// <param name="x">: Array of state space variables x</param>
/// <param name="u">: Control signal</param>
/// <param name="k">: System parameter k default is 1</param>
/// <param name="m">: System parameter m default is 1</param>
/// <param name="c">: System parameter c default is 1</param>
/// <returns>Array of derivatives of state space variables x</returns>
std::array<double, 2> f(std::array<double, 2> x, double u_local, double k = 1, double m = 1, double c = 1)
{

    //tex:
    //$\begin{align*} \dot{\vec{x}} = f(\vec{x},u) \end{align*}$

    //tex:
    //$\begin{align*} \left[ \begin{matrix} \dot{x_1} \\ \dot{x_2} \end{matrix} \right] = \begin{bmatrix} x_2 \\ - \frac{c}{m} \cdot(x_1 ^ 2 - 1) \cdot x_2 - \frac{k}{m} \cdot x_1 + \frac{u}{m} \end{bmatrix} \end{align*}$
    
    
    std::array<double, 2> d_x {0,0}; // derivative of x

    d_x[0] = x[1];
    d_x[1] = -(c / m) * (std::pow(x[0], 2) - 1) * x[1] - (k / m) * x[0] + (u_local / m);

    return d_x;
}

/// <summary>
/// Van der Pol State Space System of equations. This is used for Adaptive Control only.
/// </summary>
/// <param name="x">: Array of state space variables x</param>
/// <param name="u_local">: Control signal</param>
/// <param name="thetaR">: real theta</param>
/// <param name="m">: System Parameter m (default value is 1)</param>
/// <returns>Array of derivatives of state space variables x</returns>
std::array<double, 2> fAC(std::array<double, 2> x, double u_local, std::array<double, 2> thetaR, double m = 1)
{

    //tex:
    //$\begin{align*} \dot{\vec{x}} = f(\vec{x},u) \end{align*}$

    //tex:
    //$\begin{align*} \left[ \begin{matrix} \dot{x_1} \\ \dot{x_2} \end{matrix} \right] = \begin{bmatrix} x_2 \\ \theta_1 \cdot(x_1 - 1) ^ 2 \cdot x_2 + \theta_2 \cdot x_1 + \frac{u}{m} \end{bmatrix} \end{align*}$

    
    std::array<double, 2> d_x{ 0,0 }; // derivative of x

    d_x[0] = x[1];
    d_x[1] = thetaR[0] * (std::pow(x[0], 2) - 1) * x[1] + thetaR[1] * x[0] + u_local;

    return d_x;
}

/// <summary>
/// This is the control signal we apply to our system to guide it to our desired state. Version with 3 theta parameters.
/// </summary>
/// <param name="x">: Array of state space variables x</param>
/// <param name="theta">: Array of parameters(1,2,3)</param>
/// <returns>The control signal to be applied.</returns>
double u3(std::array<double,2> x, std::array<double,3> theta)
{
    //tex:
    //$\begin{align*} u(\vec{\theta} ,\vec{x}) = \theta_1 \cdot x_1 + \theta_2 \cdot x_2 + \theta_3 \cdot x_2 \cdot (x_1 ^2 - 1) \end{align*}$
    
    double u = theta[0]*x[0] + theta[1] * x[1] + theta[2] * x[1] * (std::pow(x[0],2) - 1);

    return u;
}

/// <summary>
/// This is the control signal we apply to our system to guide it to our desired state. Version with 2 theta parameters.
/// </summary>
/// <param name="x">: Array of state space variables x</param>
/// <param name="theta">: Array of parameters (1,2)</param>
/// <returns>The control signal to be applied.</returns>
double u2(std::array<double, 2> x, std::array<double, 3> theta)
{
    //tex:
    //$\begin{align*} u(\vec{\theta} ,\vec{x}) = \theta_1 \cdot x_1 + \theta_2 \cdot x_2 \end{align*}$

    double u = theta[0] * x[0] + theta[1] * x[1];

    return u;
}

/// <summary>
/// This is the control signal we apply to our system to guide it to our desired state. This is used for LQR.
/// </summary>
/// <param name="x">: Array of state space variables x</param>
/// <param name="K">: Array of parameters (1,2)</param>
/// <returns>The control signal to be applied.</returns>
double uLQR(std::array<double, 2> x, std::array<double, 2> K)
{
    //tex:
    //$\begin{align*} u(\vec{K} ,\vec{x}) = k_1 \cdot x_1 + k_2 \cdot x_2 \end{align*}$

    double u = K[0] * x[0] + K[1] * x[1];

    return u;
}


/// <summary>
/// This is the control signal we apply to our system to guide it to our desired state. This is used for Adaptive Control.
/// </summary>
/// <param name="x">: Array of state space variables x</param>
/// <param name="thetaApprox">: Array of theta approximation parameters</param>
/// <returns>The control signal to be applied.</returns>
double uAC(std::array<double, 2> x, std::array<double, 2> thetaApprox, double m = 1)
{
    //tex:
    //$\begin{align*} u(\vec{x},\vec{\hat{\theta}}) = \hat{\theta}_1 \cdot x_2 \cdot (x_1 - 1)^2 + \hat{\theta}_2 \cdot x_1 + k_1 \cdot x_1 + k_2 \cdot x_2 \end{align*}$

    double u = m*(thetaApprox[0] * x[1] * (x[0] - 1) * (x[0] - 1) + thetaApprox[1] * x[0] + k1 * x[0] + k2 * x[1]);

    return u;
}


/// <summary>
/// A performance function, used for FD and SPSA.
/// </summary>
/// <param name="x_old">: Initial value of state space variables x</param>
/// <param name="theta">: Theta parameters</param>
/// <param name="theta_sel">: Theta dimension. Must be 0 for 2 theta or 1 for 3 theta</param>
/// <param name="filename">: File name to save data</param>
/// <returns>Total performance</returns>
double performance(std::array<double,2> x_old, std::array<double,3> theta, int theta_sel, std::array<double, 3> systemParameters = { 1,1,1 }, std::string filename = "dump.txt")
{
    std::ofstream results;
    results.open(filename, std::ofstream::app);
    results << "\n" << std::endl;
    results << "t" << "\t" << "x[0]" << "\t" << "x[1]" << "\t" << "P" << "\t" << "theta[0]" << "\t" << "theta[1]" << "\t" << "theta[2]" << std::endl;

    double P = 0;
    std::array<double,2> x_new;
    x_new[0] = x_old[0];
    x_new[1] = x_old[1];
    
    std::array<double,timeFinal> norm;

    for (int i = 0;i < timeFinal;i++)
        norm[i] = 0;

    for (int t = 0 ; t < timeFinal ; t++)
    {
        // Store x values into vectors to plot them later.

        // Calculate the next vector x_new:
        //tex:
        //$\begin{align*} \vec{x}_{new} = \vec{x}_{old} + dt \cdot \dot{\vec{x}} = \vec{x}_{old} + dt \cdot f(\vec{x}_{old} , u(\vec{x}_{old} , \vec{\theta} ) ) \end{align*}$

        if (theta_sel == 0)
        {
            x_new[0] = x_old[0] + timeStep * f(x_old, u2(x_old, theta), systemParameters[0], systemParameters[1], systemParameters[2])[0];
            x_new[1] = x_old[1] + timeStep * f(x_old, u2(x_old, theta), systemParameters[0], systemParameters[1], systemParameters[2])[1];
        }
        else if (theta_sel == 1)
        {
            x_new[0] = x_old[0] + timeStep * f(x_old, u3(x_old, theta), systemParameters[0], systemParameters[1], systemParameters[2])[0];
            x_new[1] = x_old[1] + timeStep * f(x_old, u3(x_old, theta), systemParameters[0], systemParameters[1], systemParameters[2])[1];
        }
        // Calculate the norm
        norm[t] = sqrt(std::abs(std::pow(x_new[0], 2)) + std::abs(std::pow(x_new[1], 2)));
        // Calculate the performance
        //tex:
        //$\begin{align*} P = \sum_{t=0}^{t_{final}} \sqrt{x_{new , t , 1}^2 + x_{new , t , 2}^2} \end{align*}$
        
        
        P = P + norm[t];

        // Replace x old with x new:
        x_old[0] = x_new[0];
        x_old[1] = x_new[1];

        if (results.is_open())
        {
            results << t << "\t" << x_new[0] << "\t" << x_new[1] << "\t" << norm[t] << "\t" << P << "\t" << theta[0] << "\t" << theta[1] << "\t" << theta[2] << std::endl;
        }
        else std::cout << "\nUnable to open file\n";
    }
    return P;
}

/// <summary>
/// A performance function, used for LQR.
/// </summary>
/// <param name="x_old">: Initial value of state space variables x</param>
/// <param name="theta">: Theta parameters</param>
/// <param name="theta_sel">: Theta dimension. Must be 0 for 2 theta or 1 for 3 theta</param>
/// <param name="filename">: File name to save data</param>
/// <returns>Total performance</returns>
double performanceLQR(std::array<double, 2> x_old, std::array<double, 2> K, std::array<double, 3> systemParameters = { 1,1,1 }, std::string filename = "dump.txt")
{
    std::ofstream results;
    results.open(filename, std::ofstream::app);
    results << "\n" << std::endl;
    results << "t" << "\t" << "x[0]" << "\t" << "x[1]" << "\t" << "P" << "\t" << "theta[0]" << "\t" << "theta[1]" << "\t" << "theta[2]" << std::endl;

    double P = 0;
    std::array<double, 2> x_new;
    x_new[0] = x_old[0];
    x_new[1] = x_old[1];

    std::array<double, timeFinal> norm;

    for (int i = 0;i < timeFinal;i++)
        norm[i] = 0;

    for (int t = 0; t < timeFinal; t++)
    {
        // Store x values into vectors to plot them later.

        // Calculate the next vector x_new:
        //tex:
        //$\begin{align*} \vec{x}_{new} = \vec{x}_{old} + dt \cdot \dot{\vec{x}} = \vec{x}_{old} + dt \cdot f(\vec{x}_{old} , u(\vec{x}_{old} , \vec{\theta} ) ) \end{align*}$

        x_new[0] = x_old[0] + timeStep * f(x_old, uLQR(x_old, K), systemParameters[0], systemParameters[1], systemParameters[2])[0];
        x_new[1] = x_old[1] + timeStep * f(x_old, uLQR(x_old, K), systemParameters[0], systemParameters[1], systemParameters[2])[1];
        
        // Calculate the norm
        norm[t] = sqrt(std::abs(std::pow(x_new[0], 2)) + std::abs(std::pow(x_new[1], 2)));
        // Calculate the performance
        //tex:
        //$\begin{align*} P = \sum_{t=0}^{t_{final}} \sqrt{x_{new , t , 1}^2 + x_{new , t , 2}^2} \end{align*}$


        P = P + norm[t];

        // Replace x old with x new:
        x_old[0] = x_new[0];
        x_old[1] = x_new[1];

        if (results.is_open())
        {
            results << t << "\t" << x_new[0] << "\t" << x_new[1] << "\t" << norm[t] << "\t" << P << "\t" << K[0] << "\t" << K[1] << std::endl;
        }
        else std::cout << "\nUnable to open file\n";
    }
    return P;
}

/// <summary>
/// Algebraic Riccati Equation Solver. Solved for a 2x2 system of equations.
/// </summary>
/// <param name="matA">: A matrix (2x2)</param>
/// <param name="matB">: B matrix (1x2)</param>
/// <param name="matQ">: Q cost matrix (2x2)</param>
/// <param name="R">: R value difault is 1</param>
/// <param name="matPold">: Previous P solution</param>
/// <returns>The final time P=P(N) matrix</returns>
std::array<double, 4> riccati2(std::array<double, 4> matA, std::array<double, 2> matB, std::array<double, 4> matQ, std::array<double, 4> matPold, double R = 1 )
{
    
    std::array<double, 4> transA;
    transA[0] = matA[0];
    transA[1] = matA[1];
    transA[2] = matA[2];
    transA[3] = matA[3];

    std::array<double, 2> transB;
    transB[0] = matB[0];
    transB[1] = matB[1];

    std::array<double, 4> temp;

    std::array<double, 4> mat1;
    std::array<double, 2> mat2;
    std::array<double, 1> mat3;
    std::array<double, 1> invMat3;
    std::array<double, 2> mat4;
    std::array<double, 4> mat5;

    // Step 1: Calculate A^T x Pold x A (2x2 * 2x2 * 2x2 = 2x2)
    temp[0] = transA[0] * matPold[0] + transA[1] * matPold[1];
    temp[1] = transA[0] * matPold[2] + transA[1] * matPold[3];
    temp[2] = transA[2] * matPold[0] + transA[3] * matPold[1];
    temp[3] = transA[2] * matPold[2] + transA[3] * matPold[3];

    mat1[0] = temp[0] * matA[0] + temp[1] * matA[1];
    mat1[1] = temp[0] * matA[2] + temp[1] * matA[3];
    mat1[2] = temp[2] * matA[0] + temp[3] * matA[1];
    mat1[3] = temp[2] * matA[2] + temp[3] * matA[3];

    // Step 2: Calculate A^T x Pold x B (2x2 * 2x2 * 2x1 = 2x1)
    mat2[0] = temp[0] * matB[0] + temp[1] * matB[1];
    mat2[1] = temp[2] * matB[0] + temp[3] * matB[1];

    // Step 3: Calculate B^T x Pold x B (1x2 * 2x2 * 2x1 = 1x1)
    mat3[0] = (transB[0] * matPold[0] + transB[1] * matPold[2]) * matB[0] + (transB[0] * matPold[1] + transB[1] * matPold[3]) * matB[1];

    // Step 4: Calculate B^T x Pold x A (1x2 * 2x2 * 2x2 = 1x2)
    mat4[0] = (transB[0] * matPold[0] + transB[1] * matPold[2]) * matA[0] + (transB[0] * matPold[1] + transB[1] * matPold[3]) * matA[1];
    mat4[1] = (transB[0] * matPold[0] + transB[1] * matPold[2]) * matA[2] + (transB[0] * matPold[1] + transB[1] * matPold[3]) * matA[3];

    // Step 5: Calculate (R + B^T x Pold x B)^-1
    invMat3[0] = 1 / (mat3[0] + R);

    // Step 6: Calculate the product of matrices from step 2,4 and 5
    //tex:
    //$\begin{align*} \left[ \begin{matrix} a_1 \\ a_2 \end{matrix} \right] \cdot c \cdot \begin{bmatrix} d_1 && d_2 \end{bmatrix} \end{align*}$

    
    mat5[0] = mat2[0] * invMat3[0] * mat4[0];
    mat5[1] = mat2[0] * invMat3[0] * mat4[1];
    mat5[2] = mat2[1] * invMat3[0] * mat4[0];
    mat5[3] = mat2[1] * invMat3[0] * mat4[1];

    std::array<double, 4> matPnew = { 0.0, 0.0, 0.0, 0.0 };
    // Step 6: Calculate sum and difference of Q + step 1 - above product
    //std::cout << "P new matrix: [ ";
    for (int i = 0;i < 4;i++)
    {
        matPnew[i] = matQ[i] + mat1[i] - mat5[i];
        //std::cout << matPnew[i] << " ";
    }
    //std::cout << "]\n";
    return matPnew;
}


/// <summary>
/// LQR method using Algebraic Riccati Equation.
/// </summary>
/// <param name="x_old">: Initial position</param>
/// <param name="q">: Diagonal elements of Q 2x2 matrix</param>
/// <param name="r">: R element</param>
/// <param name="systemParameters">System parameters (k,m,c) default is (1,1,1)</param>
/// <returns>The calculated parameters k1, k2.</returns>
std::array<double, 2> lqr(std::array<double, 2>x_old, double q, double r, std::array<double, 3> systemParameters = { 1,1,1 })
{
    double k = systemParameters[0];
    double m = systemParameters[1];
    double c = systemParameters[2];

    double matR = r;
    double invR = 1 / r;

    std::array<double, 4> matQ = { q, 0, 0, q };

    std::array<double, 4> tempP, matP;
    tempP[0] = matQ[0];
    tempP[1] = matQ[1];
    tempP[2] = matQ[2];
    tempP[3] = matQ[3];

    std::array<double, 4> matA = { 0, 1, k1, k2 };

    std::array<double, 4> transA = { 0, k1, 1, k2 };

    std::array<double, 2> matB = { 0 , 1 / m };

    std::array<double, 2> transB = { 0, 1 / m };

    std::array<double, 2> matK;

    std::array<double, 2> x_new;

    double u_bar; // or sometimes in bibliography u_star

    // Step 1 : Calculate the P matrix backwards using algebraic riccati equation
    //tex:
    //$\begin{align*} P =  Q + A^T P A - A^T P B (R + B^T P B)^{-1} B^T P A\end{align*}$
    for (int i=N-1;i>=0;i--) // riccati iterations
    {
        tempP = riccati2(matA, matB, matQ, tempP, matR);
    }
    // Final Solution :
    matP[0] = tempP[0];
    matP[1] = tempP[1];
    matP[2] = tempP[2];
    matP[3] = tempP[3];
    
    // Step 2 : Calculate the K matrix
    //tex:
    //$\begin{align*} K = (R + B^T P B)^{-1} \cdot B^T P A \end{align*}$

    matK[0] = 1/(matR + (transB[0] * matP[0] + transB[1] * matP[2]) * matB[0]) * ( (transB[0] * matP[0] + transB[1] * matP[2]) * matA[0] + (transB[0] * matP[1] + transB[1] * matP[3]) * matA[1] );
    matK[1] = 1/(matR + (transB[0] * matP[1] + transB[1] * matP[3]) * matB[1]) * ( (transB[0] * matP[0] + transB[1] * matP[2]) * matA[2] + (transB[0] * matP[1] + transB[1] * matP[3]) * matA[3] );

    return matK;
}

/// <summary>
/// This is the Value function, which gives the minimum LQR cost-to-go.
/// </summary>
/// <param name="x">: The position x</param>
/// <param name="thetaA">: Theta Approximation (theta hat)</param>
/// <param name="thetaR">: Theta Real (theta we try to approximate)</param>
/// <param name="dThetaE">: Theta Error (theta difference from real)</param>
/// <param name="m">: System parameter</param>
/// <returns>The thetaA derivative</returns>
std::array<double, 2> value(std::array<double, 2> x, std::array<double, 2> thetaA, std::array<double, 2> thetaR, std::array<double, 2> dThetaE, double m = 1)
{
    //Error of theta = real(actual) theta - approximated(calculated) theta
    //tex:
    //$\begin{align*} \vec{\tilde{\theta}} = \vec{\theta} - \vec{\hat{\theta}} \end{align*}$

    std::array<double, 3> theta_error = { 0,0,0 };    // The error of the approximated theta

    theta_error[0] = thetaR[0] - thetaA[0];
    theta_error[1] = thetaR[1] - thetaA[1];

    std::array<double, 2> x_new = { 0,0 };

    double A1 = 0;
    double A2 = 0; // check below

    double dW = 0; // Value of the derivative of the function W
    double dV = 0; // Value of the derivative of the function V

    // Trying to minimize the W function, actually trying to find the values of theta hat that make the derivative W zero
    while (std::abs(dW) > 0.01)
    {

        //tex:
        //$\begin{align*} A_1 = 2 x_2 ( x_1 - 1 )^2 ( x_2 - x_1 ) \end{align*}$

        A1 = 2 * x[1] * (x[0] - 1) * (x[0] - 1) * (x[1] - x[0]);

        //tex:
        //$\begin{align*} A_2 = 2 x_1 ( x_2 - x_1 ) \end{align*}$

        A2 = 2 * x[0] * (x[1] - x[0]);

        //tex:
        //$\begin{align*} \dot{V} = - x_1 ^2 - x_2 ^2 + A_1 \tilde{\theta}_1 + A_2 \tilde{\theta}_2 \end{align*}$

        dV = -1 * x[0] * x[0] - x[1] * x[1] + theta_error[0] * A1 + theta_error[1] * A2;

        //tex:
        //$\begin{align*} \dot{W} = \dot{V} + \frac{1}{\gamma} \tilde{\theta}_1 \dot{\tilde{\theta}}_1 + \frac{1}{\gamma} \tilde{\theta}_2 \dot{\tilde{\theta}}_2 \end{align*}$


        dW = dV + (theta_error[0] * dThetaE[0]) / gac + (theta_error[1] * dThetaE[1]) / gac;

        // Calculate the next vector x_new:
        //tex:
        //$\begin{align*} \vec{x}_{new} = \vec{x}_{old} + dt \cdot \dot{\vec{x}} = \vec{x}_{old} + dt \cdot f(\vec{x}_{old} , u(\vec{x}_{old} , \vec{\theta} ) ) \end{align*}$

        thetaA[0] = thetaR[0] - theta_error[0];
        thetaA[1] = thetaR[1] - theta_error[1];

        x_new[0] = x[0] + timeStep * fAC(x, uAC(x, thetaA), thetaR, m)[0];
        x_new[1] = x[1] + timeStep * fAC(x, uAC(x, thetaA), thetaR, m)[1];

        // NOTE : maybe x_new is needed in the future
        x[0] = x_new[0];
        x[1] = x_new[1];
    }

    std::array<double, 2> res = { A1, A2 };

    return res;
}

/// <summary>
/// Gradient Descent with Performance calculation.
/// </summary>
/// <param name="x0">Initial position</param>
/// <param name="theta">Initial theta</param>
/// <param name="systemParameters">System parameters (k,m,c) default is (1,1,1)</param>
/// <returns>Array of performances</returns>
std::array<std::array<double, MAX_REPEATS>, 2> gradient_descent(std::array<double, 2> x0, std::array<double, 3> theta, std::array<double, 3> systemParameters = {1,1,1}, std::string st = "", algorithms algo_sel = FD2)
{
    std::array<std::array<double, MAX_REPEATS>, 2> P_res;
    
    for (int i = 0;i < 2;i++)
        for (int j = 0;j < MAX_REPEATS;j++)
            P_res[i][j] = 0;

    switch (algo_sel)
    {
        case FD2: // Finite Differences Algorithm with 2 theta
        {
            std::array<double, 4> P = { 0,0,0,0 };
            double Perf = 0.01;

            std::cout << "Initial Perf = [ " << P[0] << " , " << P[1] << " , " << P[2] << " , " << P[3] << " ]" << std::endl;

            int counter = 0;
            while (std::abs(Perf) < 10 && counter < MAX_REPEATS)
            {
                std::ofstream results;
                std::string filename = "results_fd2_" + st + "_" + std::to_string(counter) + ".txt";
                results.open(filename);

                std::array<double, 3> local_theta;
                local_theta[0] = theta[0];
                local_theta[1] = theta[1];
                local_theta[2] = 0; // NOT USED

                // std::cout << "Gradient Descend loop : " << i << "\n" << std::endl;
                results << "\n" << "Current theta : [ " << theta[0] << " , " << theta[1] << " ] " << std::endl;

                // Compute Performances
                //tex:
                //$\begin{align*} P_i = perfomrnace(x_0 , \theta_i + \Delta \theta_i) \end{align*}$

                P[0] = performance(x0, local_theta, 0, systemParameters, filename);
                local_theta[0] = theta[0] + dtheta;
                P[1] = performance(x0, local_theta, 0, systemParameters, filename);
                local_theta[0] = theta[0];
                local_theta[1] = theta[1] + dtheta;
                P[2] = performance(x0, local_theta, 0, systemParameters, filename);
                local_theta[1] = theta[1];

                // Calculate new theta :
                //tex:
                //$\begin{align*} \theta_{i+1} = \theta_i - \eta \cdot \frac{P_i(x_0 , \theta_i + \Delta \theta) - P_i(x_0 , \theta_i)}{\Delta \theta} \end{align*}$

                
                theta[0] = theta[0] - hetta * (P[1] - P[0]) / dtheta;
                theta[1] = theta[1] - hetta * (P[2] - P[0]) / dtheta;

                // Save performance
                P_res[0][counter] = Perf = P[0];

                // Close file
                results.close();

                // Check for anomalies
                if (isinf<double>(P[0]) || isnan<double>(P[0])) return P_res;
                if (isinf<double>(P[1]) || isnan<double>(P[1])) return P_res;
                if (isinf<double>(P[2]) || isnan<double>(P[2])) return P_res;
                if (isinf<double>(P[3]) || isnan<double>(P[3])) return P_res;

                counter += 1;
            }

            return P_res;
        }
        case FD3: // Finite Differences Algorithm with 3 theta
        {
            std::array<double, 4> P = { 0,0,0,0 };
            double Perf = 0.01;

            std::cout << "Initial Perf = [ " << P[0] << " , " << P[1] << " , " << P[2] << " , " << P[3] << " ]" << std::endl;

            int counter = 0;
            while (std::abs(Perf) < 100 && counter < MAX_REPEATS)
            {
                std::ofstream results;
                std::string filename = "results_fd3_" + std::to_string(counter) + ".txt";
                results.open(filename);

                std::array<double, 3> local_theta;
                local_theta[0] = theta[0];
                local_theta[1] = theta[1];
                local_theta[2] = theta[2];

                // std::cout << "Gradient Descend loop : " << i << "\n" << std::endl;
                results << "\n" << "Current theta : [ " << theta[0] << " , " << theta[1] << " , " << theta[2] << " ] " << std::endl;

                // Compute Performances
                //tex:
                //$\begin{align*} P_i = perfomrnace(x_0 , \theta_i + \Delta \theta_i) \end{align*}$

                P[0] = performance(x0, local_theta, 1, systemParameters, filename);
                local_theta[0] = theta[0] + dtheta;
                P[1] = performance(x0, local_theta, 1, systemParameters, filename);
                local_theta[0] = theta[0];
                local_theta[1] = theta[1] + dtheta;
                P[2] = performance(x0, local_theta, 1, systemParameters, filename);
                local_theta[1] = theta[1];
                local_theta[2] = theta[2] + dtheta;
                P[3] = performance(x0, local_theta, 1, systemParameters, filename);
                local_theta[2] = theta[2];

                // Calculate new theta :
                //tex:
                //$\begin{align*} \theta_{i+1} = \theta_i - \eta \cdot \frac{P_i(x_0 , \theta_i + \Delta \theta) - P_i(x_0 , \theta_i)}{\Delta \theta} \end{align*}$


                theta[0] = theta[0] - hetta * (P[1] - P[0]) / dtheta;
                theta[1] = theta[1] - hetta * (P[2] - P[0]) / dtheta;
                theta[2] = theta[2] - hetta * (P[3] - P[0]) / dtheta;

                // Save performance
                P_res[0][counter] = Perf = P[0];

                // Close file
                results.close();

                // Check for anomalies
                if (isinf<double>(P[0]) || isnan<double>(P[0])) return P_res;
                if (isinf<double>(P[1]) || isnan<double>(P[1])) return P_res;
                if (isinf<double>(P[2]) || isnan<double>(P[2])) return P_res;
                if (isinf<double>(P[3]) || isnan<double>(P[3])) return P_res;

                counter += 1;
            }

            return P_res;
        }
        case SPSA2: // Simultaneous Perturbation Stochastic Approximation Algorithm with 2 theta
        {
            // Note:
            // Paper: k -> iterations , but here : i -> iterations

            double ck, ak;
            std::array<double, 2> Dk;                       // 
            std::array<double, 2> end_creteria;
            std::array<double, 6> P = { 0,0,0,0,0,0 };      // Performances
            double Perf = 1000;                             // Performance 0

            for (int i = 0;i < MAX_REPEATS;i++)
            {
                // Filename to save data
                std::ofstream results;
                std::string filename = "results_spsa2_" + std::to_string(i) + ".txt";
                results.open(filename);

                // Local variable
                std::array<double, 3> local_theta;
                local_theta[0] = theta[0];
                local_theta[1] = theta[1];
                local_theta[2] = 0; // NOT USED

                // Step 1 : Initialization and coefficient selection
                //Calculate gain c_k :
                //tex:
                //$\begin{align*} c_k = \frac{c}{k^{\gamma}} \end{align*}$

                ck = betta / powl(i + 1, gamma);

                //Calculate gain a_k :
                //tex:
                //$\begin{align*} a_k = \frac{a}{(A + k)^\alpha} \end{align*}$

                ak = a / powl(A + i + 1, alpha);

                // Step 2 : Generation of the simultaneous perturbation vector
                std::random_device rd;
                std::mt19937 gen(rd());
                std::bernoulli_distribution d(p);
                for (int i = 0;i < 2;i++)
                    Dk[i] = d(gen);

                // Step 3 : Calculate Performance
                //tex:
                //$\begin{align*} P_i = perfomrnace(x_0 , \theta_i + \Delta \theta) \end{align*}$
                
                // Calculate Positive P_i ( aka y_plus ) :
                local_theta[0] = theta[0] + ck * Dk[0];
                P[0] = performance(x0, local_theta, 0, systemParameters, filename);
                local_theta[0] = theta[0];
                local_theta[1] = theta[1] + ck * Dk[1];
                P[1] = performance(x0, local_theta, 0, systemParameters, filename);
                local_theta[1] = theta[1];

                // Calculate Negative P_i ( aka y_minus ) :
                local_theta[0] = theta[0] - ck * Dk[0];
                P[3] = performance(x0, local_theta, 0, systemParameters, filename);
                local_theta[0] = theta[0];
                local_theta[1] = theta[1] - ck * Dk[1];
                P[4] = performance(x0, local_theta, 0, systemParameters, filename);
                local_theta[1] = theta[1];

                // Step 4 : Gradient approximation
                //tex:
                //$\begin{align*} \hat{g}_k (\hat{\theta} _k) = \frac{y(\hat{\theta}_k + c_k \Delta_k) - y(\hat{\theta}_k - c_k \Delta_k)}{2 c_k} \begin{bmatrix} \Delta_{k,1} ^{-1} \\ \Delta_{k,2} ^{-1} \\ \vdots \\ \Delta_{k,p} ^{-1} \end{bmatrix} \end{align*}$

                // This step is seperated into two parts , the first is at Step 3 above , the second at step 5 below.
                // Please note that in the above perfomances the P[0] ~ P[2] are the positive y(+) values
                // and the P[3] ~ P[5] are the negative y(-) values of the above formula.
                // A compact view is :
                //tex:
                //$\begin{align*} \hat{g}_{k} ( \hat{\theta_k} ) = \frac{y_{plus,k} - y_{minus,k}}{2 c_k \Delta_{k}} = \frac{P_{plus,k} - P_{minus,k}}{2 c_k \Delta_{k}} \end{align*}$

                
                // Step 5 : Updating theta estimate
                //tex:
                //$\begin{align*} \hat{\theta}_{k+1} = \hat{\theta}_{k} - a_k \cdot \hat{g}_k(\hat{\theta}_{k}) \end{align*}$

                theta[0] = local_theta[0] - ak * (P[0] - P[3]) / (2 * ck * Dk[0]);
                theta[1] = local_theta[1] - ak * (P[1] - P[4]) / (2 * ck * Dk[1]);

                // Step 6 : Iteration or termination

                end_creteria[0] = std::abs(theta[0] - local_theta[0]);
                end_creteria[1] = std::abs(theta[1] - local_theta[1]);

                // Check for anomalies
                if (isinf<double>(P[0]) || isnan<double>(P[0])) return P_res;
                if (isinf<double>(P[1]) || isnan<double>(P[1])) return P_res;
                if (isinf<double>(P[2]) || isnan<double>(P[2])) return P_res;
                if (isinf<double>(P[3]) || isnan<double>(P[3])) return P_res;
                if (isinf<double>(P[4]) || isnan<double>(P[4])) return P_res;
                if (isinf<double>(P[5]) || isnan<double>(P[5])) return P_res;

                // Save Performance
                P_res[0][i] = Perf = P[0];
                P_res[1][i] = Perf = P[3];

                if (end_creteria[0] < SPSA_END || end_creteria[1] < SPSA_END)
                    return P_res;
            }
            return P_res;
        }
        case SPSA3: // Simultaneous Perturbation Stochastic Approximation Algorithm with 3 theta
        {
            // Note:
            // Paper: k -> iterations , but here : i -> iterations

            double ck, ak;
            std::array<double, 3> Dk;                       // 
            std::array<double, 3> end_creteria;
            std::array<double, 6> P = { 0,0,0,0,0,0 };      // Performances
            double Perf = 1000;                             // Performance 0

            for (int i = 0;i < MAX_REPEATS;i++)
            {
                // Filename to save data
                std::ofstream results;
                std::string filename = "results_spsa3_" + std::to_string(i) + ".txt";
                results.open(filename);

                // Local variable
                std::array<double, 3> local_theta;
                local_theta[0] = theta[0];
                local_theta[1] = theta[1];
                local_theta[2] = theta[2];

                // Step 1 : Initialization and coefficient selection
                //Calculate gain c_k :
                //tex:
                //$\begin{align*} c_k = \frac{c}{k^{\gamma}} \end{align*}$

                ck = betta / powl(i + 1, gamma);

                //Calculate gain a_k :
                //tex:
                //$\begin{align*} a_k = \frac{a}{(A + k)^\alpha} \end{align*}$

                ak = a / powl(A + i + 1, alpha);

                // Step 2 : Generation of the simultaneous perturbation vector
                std::random_device rd;
                std::mt19937 gen(rd());
                std::bernoulli_distribution d(p);
                for (int i = 0;i < 3;i++)
                    Dk[i] = d(gen);

                // Step 3 : Calculate Performance
                //tex:
                //$\begin{align*} P_i = perfomrnace(x_0 , \theta_i + \Delta \theta) \end{align*}$

                // Calculate Positive P_i ( aka y_plus ) :
                local_theta[0] = theta[0] + ck * Dk[0];
                P[0] = performance(x0, local_theta, 1, systemParameters, filename);
                local_theta[0] = theta[0];
                local_theta[1] = theta[1] + ck * Dk[1];
                P[1] = performance(x0, local_theta, 1, systemParameters, filename);
                local_theta[1] = theta[1];
                local_theta[2] = theta[2] + ck * Dk[2];
                P[2] = performance(x0, local_theta, 1, systemParameters, filename);
                local_theta[2] = theta[2];

                // Calculate Negative P_i ( aka y_minus ) :
                local_theta[0] = theta[0] - ck * Dk[0];
                P[3] = performance(x0, local_theta, 1, systemParameters, filename);
                local_theta[0] = theta[0];
                local_theta[1] = theta[1] - ck * Dk[1];
                P[4] = performance(x0, local_theta, 1, systemParameters, filename);
                local_theta[1] = theta[1];
                local_theta[2] = theta[2] - ck * Dk[2];
                P[5] = performance(x0, local_theta, 1, systemParameters, filename);
                local_theta[2] = theta[2];

                // Step 4 : Gradient approximation
                //tex:
                //$\begin{align*} \hat{g}_k (\hat{\theta} _k) = \frac{y(\hat{\theta}_k + c_k \Delta_k) - y(\hat{\theta}_k - c_k \Delta_k)}{2 c_k} \begin{bmatrix} \Delta_{k,1} ^{-1} \\ \Delta_{k,2} ^{-1} \\ \vdots \\ \Delta_{k,p} ^{-1} \end{bmatrix} \end{align*}$

                
                // This step is seperated into two parts , the first is at Step 3 above , the second at step 5 below.
                // Please note that in the above perfomances the P[0] ~ P[2] are the positive y(+) values
                // and the P[3] ~ P[5] are the negative y(-) values of the above formula.
                // A compact view is :
                //tex:
                //$\begin{align*} \hat{g}_{k} ( \hat{\theta_k} ) = \frac{y_{plus,k} - y_{minus,k}}{2 c_k \Delta_{k}} = \frac{P_{plus,k} - P_{minus,k}}{2 c_k \Delta_{k}} \end{align*}$


                // Step 5 : Updating theta estimate
                //tex:
                //$\begin{align*} \hat{\theta}_{k+1} = \hat{\theta}_{k} - a_k \cdot \hat{g}_k(\hat{\theta}_{k}) \end{align*}$

                theta[0] = local_theta[0] - ak * (P[0] - P[3]) / (2 * ck * Dk[0]);
                theta[1] = local_theta[1] - ak * (P[1] - P[4]) / (2 * ck * Dk[1]);
                theta[2] = local_theta[2] - ak * (P[2] - P[5]) / (2 * ck * Dk[2]);

                // Step 6 : Iteration or termination

                end_creteria[0] = std::abs(theta[0] - local_theta[0]);
                end_creteria[1] = std::abs(theta[1] - local_theta[1]);
                end_creteria[2] = std::abs(theta[2] - local_theta[2]);

                // Check for anomalies
                if (isinf<double>(P[0]) || isnan<double>(P[0])) return P_res;
                if (isinf<double>(P[1]) || isnan<double>(P[1])) return P_res;
                if (isinf<double>(P[2]) || isnan<double>(P[2])) return P_res;
                if (isinf<double>(P[3]) || isnan<double>(P[3])) return P_res;
                if (isinf<double>(P[4]) || isnan<double>(P[4])) return P_res;
                if (isinf<double>(P[5]) || isnan<double>(P[5])) return P_res;

                // Save Performance
                P_res[0][i] = Perf = P[0];
                P_res[1][i] = Perf = P[3];

                if (end_creteria[0] < SPSA_END || end_creteria[1] < SPSA_END || end_creteria[2] < SPSA_END)
                    return P_res;
            }
            return P_res;
        }
        case LQR: // Linear Quadratic Regulator Algorithm
        {
            double Perf = 0.01;
            int counter = 0;
            std::array<double, 2> local_x, K;
            local_x[0] = x0[0];
            local_x[1] = x0[1];

            while (std::abs(Perf) < 10 && counter < MAX_REPEATS)
            {
                std::ofstream results;
                std::string filename = "results_lqr_" + std::to_string(counter) + ".txt";
                results.open(filename);

                // Step 1 : Calculate K matrix
                K = lqr(local_x, 1, 1);
                results << "\n" << "Current K : [ " << K[0] << " , " << K[1] << " ] " << std::endl;

                // Step 2 : Calculate performance
                Perf = performanceLQR(x0, K, systemParameters, filename);

                // Close file
                results.close();

                // Check for anomalies
                if (isinf<double>(Perf) || isnan<double>(Perf)) return P_res;

                counter += 1;
            }
        }
        case AC: // Adaptive Control ALgorithm
        {
            std::array<double, 2> P = { 0,0 };
            std::array<double, 2> thetaHat = { theta[0],theta[1]};
            std::array<double, 2> dThetaError = { 0,0 };
            std::array<double, 3> end_creteria;

            // System Parameters :
            double m = 1;
            double c = 1;
            double k = 1;
            std::array<double, 2> thetaR{ -c / m, -k / m }; // theta Real

            // NOTE :
            
            // Theoretical symbol :
            //tex:
            // $\begin{align*} \hat{\theta}\end{align*}$ 
            
            //is actually the variable theta

            for (int i = 0;i < MAX_REPEATS;i++)
            {
                // Filename to save data
                std::ofstream results;
                std::string filename = "results_ac_" + std::to_string(i) + ".txt";
                results.open(filename);

                dThetaError[0] = gac * P[0];
                dThetaError[1] = gac * P[1];

                // Step 1 : Calculate Gradient of theta hat
                P = value(x0, thetaHat, thetaR, dThetaError, m);

                // Step 2 : Calculate New theta hat
                //tex:
                // $\begin{align*} \vec{\hat{\theta}} = \vec{\hat{\theta}} + dt \cdot \gamma \cdot \vec{A} \end{align*}$
                theta[0] = theta[0] + dt * gac * P[0];
                theta[1] = theta[1] + dt * gac * P[1];

                // Step 4 : Iteration or termination

                end_creteria[0] = std::abs(dt * gac * P[0]);
                end_creteria[1] = std::abs(dt * gac * P[1]);

                // Check for anomalies
                if (isinf<double>(P[0]) || isnan<double>(P[0])) return P_res;
                if (isinf<double>(P[1]) || isnan<double>(P[1])) return P_res;

                // Save Performance value
                P_res[0][i] = P[0];
                P_res[1][i] = P[1];


                if (end_creteria[0] > AC_END || end_creteria[1] > AC_END)
                    return P_res;
            }
            return P_res;
        }
        default:
            return P_res;
    }
}

int main( int argc, char *argv[] )
{
    std::cout << "Start of program ... \n" << std::endl;
    // Step 1: Initial conditions
    std::array<double,2> x0 = {1,-1}; // initial values of state space variables x1 and x2
    // please note that :
    // x1 = position
    // x2 = velocity 
    std::array<double,3> theta = {0,0,0}; // theta parameters
    std::array<double, 3> temp_sysParameters = { 1,1,1 }; // system parameters (k,m,c)
    std::array<std::array<double, 3>, 8> sysParameters { { // system parameters (k,m,c)
        { 1, 1, 1},
        { 1, 1,-1},
        { 1,-1, 1},
        { 1,-1,-1},
        {-1, 1, 1},
        {-1, 1,-1},
        {-1,-1, 1},
        {-1,-1,-1}
    } };

    std::cout << "Initial Postion is x0 = [ "<< x0[0] <<" , " << x0[1] <<" ]" << std::endl;
    std::cout << "Initial Theta is theta = [ " << theta[0] << " , " << theta[1] << " , " << theta[2] << " ]" << std::endl;

    std::array<std::array<double, MAX_REPEATS>, 2> res;

    // Step 1: Calculate using Finite Differences with 2 theta and m=c=k=1
    algorithms sel = FD2;
    res = gradient_descent(x0, theta, temp_sysParameters, "step1", sel);
    std::cout << "Final Performance is " << res[0][MAX_REPEATS - 1] << std::endl;
    
    // Step 2: Calculate using Finite Differences with 3 theta and m=c=k=1
    sel = FD3;
    res = gradient_descent(x0, theta, temp_sysParameters, "step2", sel);
    std::cout << "Final Performance is " << res[0][MAX_REPEATS - 1] << std::endl;

    // Step 3: Calculate using SPSA with 2 theta and m=c=k=1
    sel = SPSA2;
    res = gradient_descent(x0, theta, temp_sysParameters, "step3", sel);
    std::cout << "Final Performance is P+ = " << res[0][MAX_REPEATS - 1] << " P- =  " << res[1][MAX_REPEATS - 1] << std::endl;

    // Step 4: Calculate using SPSA with 3 theta and m=c=k=1
    sel = SPSA3;
    res = gradient_descent(x0, theta, temp_sysParameters, "step4", sel);
    std::cout << "Final Performance is P+ = " << res[0][MAX_REPEATS - 1] << " P- =  " << res[1][MAX_REPEATS - 1] << std::endl;

    // Step 5: Calculate using Finite Differences with 2 theta and (m,c,k) = sysParameters
    for (int i = 0;i < 8;i++)
    {
        for (int j = 0;j < 3; j++)
        {
            temp_sysParameters[j] = sysParameters[i][j];
        }
        sel = FD2;
        res = gradient_descent(x0, theta, temp_sysParameters, "step5_" + std::to_string(i), sel);
        // Print the result
        std::cout << "Final Performance is " << res[0][MAX_REPEATS - 1] << std::endl;

    }
    // Step 6: Calculate using Finite Differences with 3 theta and (m,c,k) = sysParameters
    for (int i = 0;i < 8;i++)
    {
        for (int j = 0;j < 3; j++)
        {
            temp_sysParameters[j] = sysParameters[i][j];
        }
        sel = FD3;
        res = gradient_descent(x0, theta, temp_sysParameters, "step6_" + std::to_string(i), sel);
        std::cout << "Final Performance is " << res[0][MAX_REPEATS - 1] << std::endl;

    }
    // Step 7: Calculate using SPSA with 2 theta and (m,c,k) = sysParameters
    for (int i = 0;i < 8;i++)
    {
        for (int j = 0;j < 3; j++)
        {
            temp_sysParameters[j] = sysParameters[i][j];
        }
        sel = SPSA2;
        res = gradient_descent(x0, theta, temp_sysParameters, "step7_" + std::to_string(i), sel);
        std::cout << "Final Performance is P+ = " << res[0][MAX_REPEATS - 1] << " P- =  " << res[1][MAX_REPEATS - 1] << std::endl;

    }
    // Step 8: Calculate using SPSA with 3 theta and (m,c,k) = sysParameters
    for (int i = 0;i < 8;i++)
    {
        for (int j = 0;j < 3; j++)
        {
            temp_sysParameters[j] = sysParameters[i][j];
        }
        sel = SPSA3;
        res = gradient_descent(x0, theta, temp_sysParameters, "step8_" + std::to_string(i), sel);
        std::cout << "Final Performance is P+ = " << res[0][MAX_REPEATS - 1] << " P- =  " << res[1][MAX_REPEATS - 1] << std::endl;
    }
    // Step 9: Calculate using Linear Quandratic Control
    sel = LQR;
    temp_sysParameters[0] = sysParameters[0][0];
    temp_sysParameters[0] = sysParameters[0][1];
    temp_sysParameters[0] = sysParameters[0][2];

    res = gradient_descent(x0, theta, temp_sysParameters, "step9", sel);
    std::cout << "Final Performance is P+ = " << res[0][MAX_REPEATS - 1] << " P- = " << res[1][MAX_REPEATS - 1] << std::endl;

    // Step 10: Calculate using Adaptive Control
    sel = AC;
    res = gradient_descent(x0, theta, temp_sysParameters, "step10", sel);
    std::cout << "Final Performance is P0 = " << res[0][MAX_REPEATS - 1] << " P0 = " << res[1][MAX_REPEATS - 1] << std::endl;

    // Step 10: Sensitivity Analysis

    /*
    // Step 11: Plots
    Gnuplot gp("\"C:\\Program Files\\gnuplot\\bin\\gnuplot.exe\"");
    std::vector<double> v0;
    for (int i = 0; i < MAX_REPEATS;i++)
    {
        v0.push_back(res_fd[i]);
    }
    //std::partial_sum(v0.begin(), v0.end(), v0.begin());
    gp << "set title 'Graph of Performance'\n";
    gp << "plot '-' with lines title 'v0'\n";
    gp.send(v0);
    std::cin.get();
    */

    std::cout << "End of program ... \n" << std::endl;
    return 0;
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
