// VanDerPolOscillator.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <complex>
#include <fstream>

#define c 1         /*System Prameter c*/
#define m 1         /*System Prameter m*/
#define k 1         /*System Prameter k*/
#define dtheta 0.05 /*Rate of change of theta angle between -0.1 <= dtheta <= -0.01 and 0.1 <= dtheta <= 0.01 */
#define h 1       /*Rate of change of the gradient descend*/

double* f(double x[2], double u);
double* u(double x[2], double u);
double perf(double x0[2], double theta[3], double tt[3]);


/// <summary>
/// Van der Pol State Space System
/// </summary>
/// <param name="x">Array of input x</param>
/// <returns>Array of x derivatives</returns>
double* f( double x[2], double u)
{
    double d_x[2] = {
        x[1],
        -(c / m) * (x[0] - 1) * (x[0] - 1) * x[1] - (k / m) * x[0] + (u / m)
    };

    return d_x;
}

double u(double x[2], double theta[3])
{
    double res = theta[0]*x[0] + theta[1] * x[1] + theta[2] * x[1] * (x[0] - 1) * (x[0] -1);

    return res;
}

double perf(double x0[2], double theta[3], double tt[3], std::string filename)
{
    std::ofstream results;
    results.open(filename, std::ofstream::app);
    results << "\n" << std::endl;
    results << "t" << "\t" << "x[0]" << "\t" << "x[1]" << "\t" << "P" << "\t" << "theta[0]" << "\t" << "theta[1]" << "\t" << "theta[2]" << std::endl;

    double P = 0;
    double x[2] = { 0,0 };
    x[0] = x0[0];
    x[1] = x0[1];
    // std::cout << "Performance P starts from : " << P << std::endl;
    // std::cout << "Position x starts from : [ " << x[0] << " , " << x[1] << " ]" << std::endl;
    // std::cout << "Theta starts from : [ " << theta[0] << " , " << theta[1] << " , " << theta[2] << " ]" << std::endl;

    for (double t = tt[0] ; t < tt[2] ; t+=tt[1])
    {
        // Call the f function - derivative of x vector

        // Calculate the next x vector 
        x[0] = x[0] + tt[2] * f(x, u(x, theta))[0];
        x[1] = x[1] + tt[2] * f(x, u(x, theta))[1];
        // std::cout << "Position x at time " << t << " is : [ " << x[0] << " , " << x[1] << " ]" << std::endl;

        // Calculate the performance
        P = P + sqrt( std::abs(x[0])*std::abs(x[0]) + std::abs(x[1])*std::abs(x[1]) );
        // std::cout << "Performance is : " << P << std::endl;

        if (results.is_open())
        {
            results << t << "\t" << x[0] << "\t" << x[1] << "\t" << P << "\t" << theta[0] << "\t" << theta[1] << "\t" << theta[2] << std::endl;
        }
        else std::cout << "\nUnable to open file\n";
    }
    return P;
}


double* gradient_descent(double x0[2], double theta[3], double tt[3])
{
    // Initialize result
    double P[4] = { 0,0,0,0 };
    std::cout << "Initial Perf = [ "<< P[0] <<" , " << P[1] <<" , " << P[2] << " , " << P[3] << " ]" << std::endl;
    for(int i=0;i<10;i++)
    {
        std::ofstream results;
        std::string filename = "results_" + std::to_string(i) + ".txt";
        results.open(filename);
        double local_theta[3] = { 0,0,0 };
        local_theta[0] = theta[0];
        local_theta[1] = theta[1];
        local_theta[2] = theta[2];

        // std::cout << "Gradient Descend loop : " << i << "\n" << std::endl;
        results << "\n" << "Current theta : [ " << theta[0] << " , " << theta[1] << " , " << theta[2] << " ] " << std::endl;

        P[0] = perf(x0, local_theta, tt, filename);
        local_theta[0] = theta[0] + dtheta;
        P[1] = perf(x0, local_theta, tt, filename);
        local_theta[0] = theta[0];
        local_theta[1] = theta[1] + dtheta;
        P[2] = perf(x0, local_theta, tt, filename);
        local_theta[1] = theta[1];
        local_theta[2] = theta[2] + dtheta;
        P[3] = perf(x0, local_theta, tt, filename);

        // Calculate new theta
        for (int j = 0;j < 3;j++) {
            theta[j] = theta[j] - h * (P[j+1] - P[j]) / dtheta;
        }

        results.close();

        if (P[0] < 0.1 || isinf<double>(P[0]) || isnan<double>(P[0])) break;
        if (P[1] < 0.1 || isinf<double>(P[1]) || isnan<double>(P[1])) break;
        if (P[2] < 0.1 || isinf<double>(P[2]) || isnan<double>(P[2])) break;
        if (P[3] < 0.1 || isinf<double>(P[3]) || isnan<double>(P[3])) break;
    }

    return P;
}


int main()
{
    // Step 0: Test chart

    // Step 1: Initial conditions
    std::cout << "Start of program ... \n" << std::endl;
    double x0[2] = {10,-10};
    double theta[3] = {2,-4,5};
    // Time parameters for simulation
    double tt[3] = {
        0,  // start
        0.01,// step
        1   // stop
    };
    std::cout << "Initial Postion is x0 = [ "<< x0[0] <<" , " << x0[1] <<" ]" << std::endl;
    std::cout << "Initial Theta is theta = [ " << theta[0] << " , " << theta[1] << " , " << theta[2] << " ]" << std::endl;
    // Step 1: Call the performance function
    double* res;
    res = gradient_descent(x0, theta, tt);
    // Step 3: Print the results
    
    for (int j = 0;j < 4;j++) {
        std::cout << "\n" << std::endl;
        std::cout << "Result P" << j << " : " << res[j] << std::endl;
        std::cout << "\n" << std::endl;
    }
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
