// VanDerPolOscillator.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <random>
#include <complex>
#include <fstream>
#include <array>
#include <numeric>
#include <vector>

#include "gnuplot-iostream.h"

#define c 1             /* System Prameter c */
#define m 1             /* System Prameter m */
#define k 1             /* System Prameter k */
#define dtheta 0.001    /* Rate of change of theta angle between -0.1 <= dtheta <= -0.01 and 0.1 <= dtheta <= 0.01 */
#define h 0.01          /* Rate of change of the gradient descend */
#define step 0.01       /* Time Step */
#define timeFinal 100   /* Final Time */
#define MAX_REPEATS 100 /* Maximum number of iterations for Gradient Descent */
#define version 1       /* Choose version of code: 1, 2, 3 */

/* c_k : */
#define betta 2.1       /* non-negative coefficient for SPSA */
#define gamma 0.1       /* non-negative coefficient for SPSA */

/* a_k : */
#define a 0.1           /* non-negative coefficient for SPSA */
#define A 0.1           /* non-negative coefficient for SPSA */
#define alpha 0.1       /* non-negative coefficient for SPSA */

/* D_k : */
#define p 0.5           /* propability for Delta_k bernoulli distribution for SPSA*/

#define END_CONDITION 0.01/* End value for SPSA termination */

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
    SPSA
};

/// <summary>
/// Van der Pol State Space System of equations.
/// </summary>
/// <param name="x">Array of x - current postion</param>
/// <param name="u">Control signal u</param>
/// <returns>Array of x derivatives</returns>
std::array<double,2> f( std::array<double,2> x, double u)
{
    //tex:
    //$\begin{align*} \dot{\vec{x}} = f(\vec{x},u) \end{align*}$

    //tex:
    //$\begin{align*} \left[ \begin{matrix} \dot{x_1} \\ \dot{x_2} \end{matrix} \right] = \begin{bmatrix} x_2 \\ - \frac{c}{m} \cdot(x_1 - 1) ^ 2 \cdot x_2 - \frac{k}{m} \cdot x_1 + \frac{u}{m} \end{bmatrix} \end{align*}$
    
    std::array<double, 2> d_x {0,0};

    d_x[0] = x[1];
    d_x[1] = -(c / m) * (std::pow(x[0], 2) - 1) * x[1] - (k / m) * x[0] + (u / m);

    return d_x;
}


/// <summary>
/// This is the control signal we apply to our system to guide it to our desired state.
/// </summary>
/// <param name="x">Array of x - current position</param>
/// <param name="theta">Array of parameters</param>
/// <returns>The control signal to be applied.</returns>
double u(std::array<double,2> x, std::array<double,3> theta)
{
    //tex:
    //$\begin{align*} u(\theta ,\vec{x}) = \theta_1 \cdot x_1 + \theta_2 \cdot x_2 + \theta_2 \cdot x_2 \cdot (x_1 - 1)^2 \end{align*}$
    
    double res = theta[0]*x[0] + theta[1] * x[1] + theta[2] * x[1] * (std::pow(x[0],2) - 1);

    return res;
}


/// <summary>
/// A performance function.
/// </summary>
/// <param name="x_old">initial performance</param>
/// <param name="theta">theta parameters</param>
/// <param name="filename">Name of file to save data</param>
/// <returns></returns>
double performace(std::array<double,2> x_old, std::array<double,3> theta, std::string filename)
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
        //tex:
        //$\begin{align*} \vec{x}_{new} = \vec{x}_{old} + dt \cdot \dot{\vec{x}} = \vec{x}_{old} + dt \cdot f(\vec{x}_{old} , u(\vec{x}_{old} , \vec{\theta} ) ) \end{align*}$
        
        // Call the f function - derivative of x vector
        // Calculate the next x vector 
        x_new[0] = x_old[0] + step * f(x_old, u(x_old, theta))[0];
        x_new[1] = x_old[1] + step * f(x_old, u(x_old, theta))[1];
        // std::cout << "Position x at time " << t << " is : [ " << x[0] << " , " << x[1] << " ]" << std::endl;

        // Calculate the norm
        norm[t] = sqrt(std::abs(std::pow(x_new[0], 2)) + std::abs(std::pow(x_new[1], 2)));
        // Calculate the performance
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

double loss(double value)
{
    //tex:
    //$\begin{align*} = \end{align*}$

    double res = 0;//f(theta,);

    return res;
}

std::array<double, 3> y(std::array<double, 3> theta, double ck, std::array<double,MAX_REPEATS> Dk)
{
    
    std::array<double, 3> res;

    res[0] = loss(theta[0] + ck * Dk[0]);
    res[1] = loss(theta[1] + ck * Dk[1]);
    res[2] = loss(theta[2] + ck * Dk[2]);

    return res;
}

std::array<double,3> gk(std::array<double,3> theta, double ck, std::array<double,MAX_REPEATS> Dk)
{
    //tex:
    //$\begin{align*} \hat{g}_{k,i} ( \hat{\theta_k} ) = \frac{y_{plus} - y_{minus}}{2 c_k \Delta_{k,i}} \end{align*}$

    std::array<double, 3> res;
    double minus = 0;
    
    std::array<double,3>y_plus = y(theta, ck, Dk);
    std::array<double,3>y_minus = y(theta, -1 * ck, Dk);

    res[0] = (y_plus[0] - y_minus[0]) / (2 * ck * Dk[0]);
    res[1] = (y_plus[1] - y_minus[1]) / (2 * ck * Dk[1]);
    res[2] = (y_plus[2] - y_minus[2]) / (2 * ck * Dk[2]);

    return res;
}


/// <summary>
/// Gradient Descent Algorithm with Performance calculation.
/// </summary>
/// <param name="x0">Initial position</param>
/// <param name="theta">Initial theta</param>
/// <returns>Array of performances</returns>
std::array<double,MAX_REPEATS> gradient_descent(std::array<double,2> x0, std::array<double,3> theta, algorithms algo_sel)
{
    std::array<double, MAX_REPEATS> P_res;
    
    for (int i = 0;i < MAX_REPEATS;i++)
        P_res[i] = 0;

    switch (algo_sel)
    {
        case FD:
        {
            std::array<double, 4> P = { 0,0,0,0 };
            double Perf = 1000;

            std::cout << "Initial Perf = [ " << P[0] << " , " << P[1] << " , " << P[2] << " , " << P[3] << " ]" << std::endl;

            int counter = 0;
            while (std::abs(Perf) > 0.1 && counter < MAX_REPEATS)
            {
                std::ofstream results;
                std::string filename = "results_" + std::to_string(counter) + ".txt";
                results.open(filename);

                std::array<double, 3> local_theta;
                local_theta[0] = theta[0];
                local_theta[1] = theta[1];
                local_theta[2] = theta[2];

                // std::cout << "Gradient Descend loop : " << i << "\n" << std::endl;
                results << "\n" << "Current theta : [ " << theta[0] << " , " << theta[1] << " , " << theta[2] << " ] " << std::endl;

                // Compute Performances
                P[0] = performace(x0, local_theta, filename);
                local_theta[0] = theta[0] + dtheta;
                P[1] = performace(x0, local_theta, filename);
                local_theta[0] = theta[0];
                local_theta[1] = theta[1] + dtheta;
                P[2] = performace(x0, local_theta, filename);
                local_theta[1] = theta[1];
                local_theta[2] = theta[2] + dtheta;
                P[3] = performace(x0, local_theta, filename);

                //tex:
                //$\begin{align*} \theta_i = \theta_i + \hat{\eta} \cdot \frac{P_0 - P_{i+1}}{\Delta \theta} \end{align*}$

                // Calculate new theta
                theta[0] = theta[0] + h * (P[0] - P[1]) / dtheta;
                theta[1] = theta[1] + h * (P[0] - P[2]) / dtheta;
                theta[2] = theta[2] + h * (P[0] - P[3]) / dtheta;

                // Save performance
                P_res[counter] = P[0];

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
        case SPSA:
        {
            // Note: k -> iterations , here : i -> iterations => k=i

            double ck, ak;
            std::array<double, MAX_REPEATS> Dk;
            std::array<double, 3> theta_old, end_creteria;

            for (int i = 0;i < MAX_REPEATS;i++)
            {
                // Step 1 : Initialization and coefficient selection
                //Calculate gain c_k :
                //tex:
                //$\begin{align*} c_k = \frac{c}{k^{\gamma}} \end{align*}$

                ck = betta / powl(i, gamma);

                //Calculate gain a_k :
                //tex:
                //$\begin{align*} a_k = \frac{a}{(A + k)^\alpha} \end{align*}$

                ak = a / powl(A + i, alpha);

                // Step 2 : Generation of the simultaneous perturbation vector
                std::random_device rd;
                std::mt19937 gen(rd());
                std::bernoulli_distribution d(p);
                for (int i = 0;i < MAX_REPEATS;i++)
                    Dk[i] = d(gen);


                // Step 4 : Gradient approximation
                //tex:
                //$\begin{align*} \hat{g}_k (\hat{\theta} _k) = \frac{y(\hat{\theta}_k + c_k \Delta_k) - y(\hat{\theta}_k - c_k \Delta_k)}{2 c_k} \begin{bmatrix} \Delta_{k,1} ^{-1} \\ \Delta_{k,2} ^{-1} \\ \vdots \\ \Delta_{k,p} ^{-1} \end{bmatrix} \end{align*}$

                std::array<double, 3> gk_hat = gk(theta, ck, Dk);

                // Step 5 : Updating theta estimate
                //tex:
                //$\begin{align*} \hat{\theta}_{k+1} = \hat{\theta}_{k} - a_k \cdot \hat{g}_k(\hat{\theta}_{k}) \end{align*}$

                theta_old[0] = theta[0];
                theta_old[1] = theta[1];
                theta_old[2] = theta[2];

                theta[0] = theta[0] - ak * gk_hat[0];
                theta[1] = theta[1] - ak * gk_hat[1];
                theta[2] = theta[2] - ak * gk_hat[2];

                // Step 6 : Iteration or termination

                end_creteria[0] = std::abs(theta[0] - theta_old[0]);
                end_creteria[1] = std::abs(theta[1] - theta_old[1]);
                end_creteria[2] = std::abs(theta[2] - theta_old[2]);

                if (end_creteria[0] < END_CONDITION || end_creteria[0] < END_CONDITION || end_creteria[0] < END_CONDITION)
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
    std::array<double,MAX_REPEATS> res_fd;
    res_fd = gradient_descent(x0, theta,sel);
    // Print the result
    std::cout << "Final Performance is " << res_fd[MAX_REPEATS-1] << std::endl;
    
    // Step 2: Calculate using SPSA
    //sel = SPSA;
    //std::array<double, MAX_REPEATS> res_spsa;
    //res_spsa = gradient_descent(x0, theta, sel);
    // Print the result
    //std::cout << "Final Performance is " << res_spsa[MAX_REPEATS - 1] << std::endl;

    // Step 3: Plots
    Gnuplot gp("\"C:\\Program Files\\gnuplot\\bin\\gnuplot.exe\"");
    std::vector<double> v0;
    for (int i = 0; i < MAX_REPEATS;i++)
    {
        v0.push_back(res_fd[i]);
    }
    std::partial_sum(v0.begin(), v0.end(), v0.begin());

    gp << "set title 'Graph of Performance'\n";
    gp << "plot '-' with lines title 'v0'\n";
    gp.send(v0);

    std::cin.get();

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
