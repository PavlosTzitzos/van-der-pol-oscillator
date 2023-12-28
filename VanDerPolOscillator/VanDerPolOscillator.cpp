// VanDerPolOscillator.cpp : This file contains the 'main' function. Program execution begins and ends there.
// LaTeX Comments Extension : https://github.com/kindermannhubert/VsTeXCommentsExtension

#include <iostream>
#include <random>
#include <complex>
#include <fstream>
#include <array>
#include <numeric>
#include <vector>

#include "gnuplot-iostream.h"

#define dtheta 0.001    /* Rate of change of theta angle between -0.1 <= dtheta <= -0.01 and 0.1 <= dtheta <= 0.01 */
#define hetta 0.01      /* Rate of change of the gradient descend */
#define timeStep 0.01   /* Time Step (must approach zero: dt->0) */
#define timeFinal 100   /* Stop Value of performance function */
#define MAX_REPEATS 100 /* Maximum number of iterations for Gradient Descent Top algorithm */
#define version 1       /* Choose version of code: 1 for FD, 2 for SPSA, 3 will be added */

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

std::vector<double> x1;
std::vector<double> x2;

/// <summary>
/// Choose available algorithms
/// </summary>
enum algorithms
{
    /// <summary>
    /// Finite Differences
    /// </summary>
    FD,
    /// <summary>
    /// Simultaneous Perturbation Stochastic Approximation
    /// </summary>
    SPSA,
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
/// Van der Pol State Space System of equations.
/// </summary>
/// <param name="x">Array of current postion</param>
/// <param name="u">Control signal</param>
/// <param name="sysPar">System Parameters</param>
/// <returns>Array of x derivatives</returns>
std::array<double, 2> f(std::array<double, 2> x, double u_local, double k = 1, double m = 1, double c = 1)
{

    //tex:
    //$\begin{align*} \dot{\vec{x}} = f(\vec{x},u) \end{align*}$

    //tex:
    //$\begin{align*} \left[ \begin{matrix} \dot{x_1} \\ \dot{x_2} \end{matrix} \right] = \begin{bmatrix} x_2 \\ - \frac{c}{m} \cdot(x_1 ^ 2 - 1) \cdot x_2 - \frac{k}{m} \cdot x_1 + \frac{u}{m} \end{bmatrix} \end{align*}$
    
    
    std::array<double, 2> d_x {0,0};

    d_x[0] = x[1];
    d_x[1] = -(c / m) * (std::pow(x[0], 2) - 1) * x[1] - (k / m) * x[0] + (u_local / m);

    return d_x;
}

/// <summary>
/// Van der Pol State Space System of equations. This is used for Adaptive Control
/// </summary>
/// <param name="x">Array of current postion</param>
/// <param name="u_local">Control signal</param>
/// <param name="thetaR">real theta</param>
/// <param name="m">System Parameter m (default value is 1)</param>
/// <returns>Array of x derivatives</returns>
std::array<double, 2> fAC(std::array<double, 2> x, double u_local, std::array<double, 2> thetaR, double m = 1)
{

    //tex:
    //$\begin{align*} \dot{\vec{x}} = f(\vec{x},u) \end{align*}$

    //tex:
    //$\begin{align*} \left[ \begin{matrix} \dot{x_1} \\ \dot{x_2} \end{matrix} \right] = \begin{bmatrix} x_2 \\ \theta_1 \cdot(x_1 - 1) ^ 2 \cdot x_2 + \theta_2 \cdot x_1 + \frac{u}{m} \end{bmatrix} \end{align*}$

    std::array<double, 2> d_x{ 0,0 };

    d_x[0] = x[1];
    d_x[1] = thetaR[0] * (std::pow(x[0], 2) - 1) * x[1] + thetaR[1] * x[0] + u_local;

    return d_x;
}

/// <summary>
/// This is the control signal we apply to our system to guide it to our desired state.
/// </summary>
/// <param name="x">Array of x - current position</param>
/// <param name="theta">Array of parameters(1,2,3)</param>
/// <returns>The control signal to be applied.</returns>
double u3(std::array<double,2> x, std::array<double,3> theta)
{
    //tex:
    //$\begin{align*} u(\vec{\theta} ,\vec{x}) = \theta_1 \cdot x_1 + \theta_2 \cdot x_2 + \theta_3 \cdot x_2 \cdot (x_1 ^2 - 1) \end{align*}$
    
    double res = theta[0]*x[0] + theta[1] * x[1] + theta[2] * x[1] * (std::pow(x[0],2) - 1);

    return res;
}

/// <summary>
/// This is the control signal we apply to our system to guide it to our desired state.
/// </summary>
/// <param name="x">Array of x - current position</param>
/// <param name="theta">Array of parameters (1,2)</param>
/// <returns>The control signal to be applied.</returns>
double u2(std::array<double, 2> x, std::array<double, 2> theta)
{
    //tex:
    //$\begin{align*} u(\vec{\theta} ,\vec{x}) = \theta_1 \cdot x_1 + \theta_2 \cdot x_2 \end{align*}$

    double res = theta[0] * x[0] + theta[1] * x[1];

    return res;
}

/// <summary>
/// This is the control signal we apply to our system to guide it to our desired state. This is used for Adaptive Control.
/// </summary>
/// <param name="x">Array of x - current position</param>
/// <param name="thetaA">Array of parameters theta Approximation</param>
/// <returns>The control signal to be applied.</returns>
double uAC(std::array<double, 2> x, std::array<double, 2> thetaA, double m = 1)
{
    //tex:
    //$\begin{align*} u(\vec{x},\vec{\hat{\theta}}) = \hat{\theta}_1 \cdot x_2 \cdot (x_1 - 1)^2 + \hat{\theta}_2 \cdot x_1 + k_1 \cdot x_1 + k_2 \cdot x_2 \end{align*}$

    double res = m*(thetaA[0] * x[1] * (x[0] - 1) * (x[0] - 1) + thetaA[1] * x[0] + k1 * x[0] + k2 * x[1]);

    return res;
}


/// <summary>
/// A performance function, used for FD, SPSA.
/// </summary>
/// <param name="x_old">initial performance</param>
/// <param name="theta">theta parameters</param>
/// <param name="filename">Name of file to save data</param>
/// <returns> Total performance</returns>
double performance(std::array<double,2> x_old, std::array<double,3> theta, std::string filename)
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

    // std::cout << "Performance P starts from : " << P << std::endl;
    // std::cout << "Position x starts from : [ " << x[0] << " , " << x[1] << " ]" << std::endl;
    // std::cout << "Theta starts from : [ " << theta[0] << " , " << theta[1] << " , " << theta[2] << " ]" << std::endl;

    for (int t = 0 ; t < timeFinal ; t++)
    {
        // Store x values into vectors to plot them later.

        // Calculate the next vector x_new:
        //tex:
        //$\begin{align*} \vec{x}_{new} = \vec{x}_{old} + dt \cdot \dot{\vec{x}} = \vec{x}_{old} + dt \cdot f(\vec{x}_{old} , u(\vec{x}_{old} , \vec{\theta} ) ) \end{align*}$

        x_new[0] = x_old[0] + timeStep * f(x_old, u(x_old, theta))[0];
        x_new[1] = x_old[1] + timeStep * f(x_old, u(x_old, theta))[1];
        
        // Calculate the norm
        norm[t] = sqrt(std::abs(std::pow(x_new[0], 2)) + std::abs(std::pow(x_new[1], 2)));
        // Calculate the performance
        //tex:
        //$\begin{align*} P = \sum_{t=0}^{t_{final}} \sqrt{x_{new , t , 1}^2 + x_{new , t , 2}^2} \end{align*}$
        
        
        P = P + norm[t];
        // std::cout << "Performance is : " << P << std::endl;
        
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
/// A cost function, used for LRQ.
/// </summary>
/// <param name="x">: initial guess</param>
/// <param name="theta">: parameter to optimize</param>
/// <param name="r">: for R matrix</param>
/// <param name="q">: for Q matrix</param>
/// <returns>Total cost</returns>
double cost(std::array<double, 2> x,std::array<double,3> theta, double r = 1, double q = 1)
{
    int t = 0;      // time
    int t_f = 10;   // final time
    double sum = 0; // sum of cost

    std::array<double, 2> x_new = { 0,0 };

    // The integral form is :
    //tex:
    //$\begin{align*} J(u) = \int_0 ^{t_f} (x(t)^T Q x(t) + u(t)^T R u(t) ) dt + x(t_f)^T Q_f x(t_f)\end{align*}$

    // The discrete time form is :
    //tex:
    //$\begin{align*} J(u) = \sum_{t=0} ^{t_f - 1} (x(t)^T Q x(t) + u(t)^T R u(t) ) + x(t_f)^T Q_f x(t_f)\end{align*}$

    for (t = 0;t < t_f;t++)
    {
        sum += x[0] * x[0] * q + x[1] * x[1] * q + u(x,theta) * u(x,theta);
        
        // Calculate the next vector x_new:
        //tex:
        //$\begin{align*} \vec{x}_{new} = \vec{x}_{old} + dt \cdot \dot{\vec{x}} = \vec{x}_{old} + dt \cdot f(\vec{x}_{old} , u(\vec{x}_{old} , \vec{\theta} ) ) \end{align*}$

        x_new[0] = x[0] + timeStep * f(x, u(x, theta))[0];
        x_new[1] = x[1] + timeStep * f(x, u(x, theta))[1];
        // NOTE : maybe x_new is needed in the future
        x[0] = x_new[0];
        x[1] = x_new[1];
    }
    sum += x[0] * x[0] * q + x[1] * x[1] * q;

    return sum;
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

    std::array<double, 3> theta_error = {0,0,0};    // The error of the approximated theta
    
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

        
        dW = dV + (theta_error[0] * dThetaE[0] ) / gac + (theta_error[1] * dThetaE[1]) / gac;

        // Calculate the next vector x_new:
        //tex:
        //$\begin{align*} \vec{x}_{new} = \vec{x}_{old} + dt \cdot \dot{\vec{x}} = \vec{x}_{old} + dt \cdot f(\vec{x}_{old} , u(\vec{x}_{old} , \vec{\theta} ) ) \end{align*}$

        thetaA[0] = thetaR[0] - theta_error[0];
        thetaA[1] = thetaR[1] - theta_error[1];

        x_new[0] = x[0] + timeStep * f2(x, u2(x, thetaA), thetaR, m)[0];
        x_new[1] = x[1] + timeStep * f2(x, u2(x, thetaA), thetaR, m)[1];

        // NOTE : maybe x_new is needed in the future
        x[0] = x_new[0];
        x[1] = x_new[1];
    }

    std::array<double, 2> res = {A1, A2};

    return res;
}


/// <summary>
/// Gradient Descent with Performance calculation.
/// </summary>
/// <param name="x0">Initial position</param>
/// <param name="theta">Initial theta</param>
/// <returns>Array of performances</returns>
std::array<std::array<double, MAX_REPEATS>, 2> gradient_descent(std::array<double,2> x0, std::array<double,3> theta, algorithms algo_sel)
{
    std::array<std::array<double, MAX_REPEATS>, 2> P_res;
    
    for (int i = 0;i < MAX_REPEATS;i++)
        for (int j = 0;j < MAX_REPEATS;j++)
            P_res[i][j] = 0;

    switch (algo_sel)
    {
        case FD: // Finite Differences Algorithm
        {
            std::array<double, 4> P = { 0,0,0,0 };
            double Perf = 1000;

            std::cout << "Initial Perf = [ " << P[0] << " , " << P[1] << " , " << P[2] << " , " << P[3] << " ]" << std::endl;

            int counter = 0;
            while (std::abs(Perf) > 0.1 && counter < MAX_REPEATS)
            {
                std::ofstream results;
                std::string filename = "results_fd_" + std::to_string(counter) + ".txt";
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

                P[0] = performance(x0, local_theta, filename);
                local_theta[0] = theta[0] + dtheta;
                P[1] = performance(x0, local_theta, filename);
                local_theta[0] = theta[0];
                local_theta[1] = theta[1] + dtheta;
                P[2] = performance(x0, local_theta, filename);
                local_theta[1] = theta[1];
                local_theta[2] = theta[2] + dtheta;
                P[3] = performance(x0, local_theta, filename);
                local_theta[2] = theta[2];

                // Calculate new theta :
                //tex:
                //$\begin{align*} \theta_{i+1} = \theta_i - \eta \cdot \frac{P_i(x_0 , \theta_i + \Delta \theta) - P_i(x_0 , \theta_i)}{\Delta \theta} \end{align*}$

                
                theta[0] = theta[0] - hetta * (P[1] - P[0]) / dtheta;
                theta[1] = theta[1] - hetta * (P[2] - P[0]) / dtheta;
                theta[2] = theta[2] - hetta * (P[3] - P[0]) / dtheta;

                // Save performance
                P_res[counter][0] = Perf = P[0];

                // Close file
                results.close();

                // Check for anomalies
                if (isinf<double>(P[0]) || isnan<double>(P[0])) break;
                if (isinf<double>(P[1]) || isnan<double>(P[1])) break;
                if (isinf<double>(P[2]) || isnan<double>(P[2])) break;
                if (isinf<double>(P[3]) || isnan<double>(P[3])) break;

                counter += 1;
            }

            return P_res;
        }
        case SPSA: // Simultaneous Perturbation Stochastic Approximation Algorithm
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
                std::string filename = "results_spsa_" + std::to_string(i) + ".txt";
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
                P[0] = performance(x0, local_theta, filename);
                local_theta[0] = theta[0];
                local_theta[1] = theta[1] + ck * Dk[1];
                P[1] = performance(x0, local_theta, filename);
                local_theta[1] = theta[1];
                local_theta[2] = theta[2] + ck * Dk[2];
                P[2] = performance(x0, local_theta, filename);
                local_theta[2] = theta[2];

                // Calculate Negative P_i ( aka y_minus ) :
                local_theta[0] = theta[0] - ck * Dk[0];
                P[3] = performance(x0, local_theta, filename);
                local_theta[0] = theta[0];
                local_theta[1] = theta[1] - ck * Dk[1];
                P[4] = performance(x0, local_theta, filename);
                local_theta[1] = theta[1];
                local_theta[2] = theta[2] - ck * Dk[2];
                P[5] = performance(x0, local_theta, filename);
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
                if (isinf<double>(P[0]) || isnan<double>(P[0])) break;
                if (isinf<double>(P[1]) || isnan<double>(P[1])) break;
                if (isinf<double>(P[2]) || isnan<double>(P[2])) break;
                if (isinf<double>(P[3]) || isnan<double>(P[3])) break;
                if (isinf<double>(P[4]) || isnan<double>(P[4])) break;
                if (isinf<double>(P[5]) || isnan<double>(P[5])) break;

                // Save Performance
                P_res[i][0] = Perf = P[0];
                P_res[i][1] = Perf = P[3];

                if (end_creteria[0] < SPSA_END || end_creteria[1] < SPSA_END || end_creteria[2] < SPSA_END)
                    return P_res;
            }
            return P_res;
        }
        case LQR: // Linear Quadratic Regulator Algorithm
        {
            // Step 1 : Calculate Cost J(u)

            //P[0] = cost(x0, theta);


            // Step 2 : Calculate new theta

            theta[0] = theta[0] - 0;

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
                if (isinf<double>(P[0]) || isnan<double>(P[0])) break;
                if (isinf<double>(P[1]) || isnan<double>(P[1])) break;
                if (isinf<double>(P[2]) || isnan<double>(P[2])) break;
                if (isinf<double>(P[3]) || isnan<double>(P[3])) break;
                if (isinf<double>(P[4]) || isnan<double>(P[4])) break;
                if (isinf<double>(P[5]) || isnan<double>(P[5])) break;

                // Save Performance value
                P_res[i][0] = P[0];
                P_res[i][1] = P[1];


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
    std::array<double,2> x0 = {1,-1};
    std::array<double,3> theta = {0,0,0};

    std::cout << "Initial Postion is x0 = [ "<< x0[0] <<" , " << x0[1] <<" ]" << std::endl;
    std::cout << "Initial Theta is theta = [ " << theta[0] << " , " << theta[1] << " , " << theta[2] << " ]" << std::endl;
    
    // Step 1: Calculate using Finite Differences
    algorithms sel = FD;
    std::array<std::array<double, MAX_REPEATS>, 2> res_fd;
    res_fd = gradient_descent(x0, theta,sel);
    // Print the result
    std::cout << "Final Performance is " << res_fd[MAX_REPEATS-1][0] << std::endl;
    
    // Step 2: Calculate using SPSA
    sel = SPSA;
    std::array<std::array<double, MAX_REPEATS>, 2> res_spsa;
    res_spsa = gradient_descent(x0, theta, sel);
    // Print the result
    std::cout << "Final Performance is P+ = " << res_spsa[MAX_REPEATS - 1][0] << " P- =  " << res_spsa[MAX_REPEATS - 1][1] << std::endl;

    // Step 3 : Calculate using Linear Quandratic Control
    sel = LQR;
    std::array<std::array<double, MAX_REPEATS>, 2> res_lqr;
    res_lqr = gradient_descent(x0, theta, sel);
    // Print the result
    std::cout << "Final Performance is P+ = " << res_lqr[MAX_REPEATS - 1][0] << " P- = " << res_lqr[MAX_REPEATS - 1][1] << std::endl;

    // Step 4 : Calculate using Adaptive Control
    sel = AC;
    std::array<std::array<double, MAX_REPEATS>, 2> res_ac;
    res_ac = gradient_descent(x0, theta, sel);
    // Print the result
    std::cout << "Final Performance is P0 = " << res_ac[MAX_REPEATS - 1][0] << " P0 = " << res_ac[MAX_REPEATS - 1][1] << std::endl;

    /*
    // Step 5: Plots
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

    // Step 5: Closing message
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
