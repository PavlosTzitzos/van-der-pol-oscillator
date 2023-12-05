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

#define dtheta 0.001    /* Rate of change of theta angle between -0.1 <= dtheta <= -0.01 and 0.1 <= dtheta <= 0.01 */
#define hetta 0.01      /* Rate of change of the gradient descend */
#define timeStep 0.01      /* Time Step (must approach zero: dt->0) */
#define timeFinal 100   /* Stop Value of performance function */
#define MAX_REPEATS 100 /* Maximum number of iterations for Gradient Descent Top algorithm */
#define version 1       /* Choose version of code: 1 for FD, 2 for SPSA, 3 will be added */

/* c_k : */
#define betta 2.1       /* non-negative coefficient for SPSA */
#define gamma 0.1       /* non-negative coefficient for SPSA */

/* a_k : */
#define a 0.1           /* non-negative coefficient for SPSA */
#define A 0.1           /* non-negative coefficient for SPSA */
#define alpha 0.1       /* non-negative coefficient for SPSA */

/* D_k : */
#define p 0.5           /* propability for Delta_k bernoulli distribution for SPSA*/

#define SPSA_END 0.01   /* Creteria value for SPSA termination */

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
    SPSA
};

/// <summary>
/// Van der Pol State Space System of equations.
/// </summary>
/// <param name="x">Array of current postion</param>
/// <param name="u">Control signal</param>
/// <param name="sysPar">System Parameters</param>
/// <returns>Array of x derivatives</returns>
std::array<double, 2> f(std::array<double, 2> x, double u, std::array<double, 3> sysPar = {1,1,1})
{
    double k = 1;
    double m = 1;
    double c = 1;

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
                P_res[counter] = Perf = P[0];

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

                ck = betta / powl(i, gamma);

                //Calculate gain a_k :
                //tex:
                //$\begin{align*} a_k = \frac{a}{(A + k)^\alpha} \end{align*}$

                ak = a / powl(A + i, alpha);

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

                if (end_creteria[0] < SPSA_END || end_creteria[0] < SPSA_END || end_creteria[0] < SPSA_END)
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
    //std::partial_sum(v0.begin(), v0.end(), v0.begin());

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
