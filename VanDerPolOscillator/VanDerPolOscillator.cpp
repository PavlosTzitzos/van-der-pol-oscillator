// VanDerPolOscillator.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <complex>
#include <fstream>
#include <array>

#define c 1             /* System Prameter c */
#define m 1             /* System Prameter m */
#define k 1             /* System Prameter k */
#define dtheta 0.001    /* Rate of change of theta angle between -0.1 <= dtheta <= -0.01 and 0.1 <= dtheta <= 0.01 */
#define h 0.01          /* Rate of change of the gradient descend */
#define dt 0.01         /* Time Step */
#define timeFinal 100   /* Final Time */
#define MAX_REPEATS 10  /* Maximum number of iterations for Gradient Descent */
#define version 1       /* Choose version of code: 1, 2, 3 */


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
        // Call the f function - derivative of x vector

        // Calculate the next x vector 
        x_new[0] = x_old[0] + dt * f(x_old, u(x_old, theta))[0];
        x_new[1] = x_old[1] + dt * f(x_old, u(x_old, theta))[1];
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


/// <summary>
/// Gradient Descent Algorithm with Performance calculation.
/// </summary>
/// <param name="x0">Initial position</param>
/// <param name="theta">Initial theta</param>
/// <returns>Array of performances</returns>
std::array<double,MAX_REPEATS> gradient_descent(std::array<double,2> x0, std::array<double,3> theta)
{
    std::array<double, 4> P = {0,0,0,0};
    double Perf = 1000;
    std::array<double, MAX_REPEATS> P_res;

    for (int i = 0;i < MAX_REPEATS;i++)
        P_res[i] = 0;

    std::cout << "Initial Perf = [ "<< P[0] <<" , " << P[1] <<" , " << P[2] << " , " << P[3] << " ]" << std::endl;

    int counter = 0;
    while(std::abs(Perf)>0.1 && counter < MAX_REPEATS)
    {
        std::ofstream results;
        std::string filename = "results_" + std::to_string(counter) + ".txt";
        results.open(filename);
        
        std::array<double,3> local_theta;
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



int main( int argc, char *argv[] )
{
    std::cout << "Start of program ... \n" << std::endl;
    // Step 1: Initial conditions
    std::array<double,2> x0 = {1,-1};
    std::array<double,3> theta = {0,0,0};

    std::cout << "Initial Postion is x0 = [ "<< x0[0] <<" , " << x0[1] <<" ]" << std::endl;
    std::cout << "Initial Theta is theta = [ " << theta[0] << " , " << theta[1] << " , " << theta[2] << " ]" << std::endl;
    
    // Step 1: Call the performance function
    std::array<double,MAX_REPEATS> res;
    res = gradient_descent(x0, theta);
    
    // Step 3: Print the result
    std::cout << "Final Performance is " << res[MAX_REPEATS-1] << std::endl;
    
    // Step 4: Plots
    

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
