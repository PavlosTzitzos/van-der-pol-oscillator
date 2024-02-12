// THIS IS THE OLD VERSION 
// VanDerPolOscillator.cpp : This file contains the 'main' function. Program execution begins and ends there.
// LaTeX Comments Extension : https://github.com/kindermannhubert/VsTeXCommentsExtension
// Doxygen Comments : https://www.doxygen.nl/manual/docblocks.html
// GnuPlot : https://www.youtube.com/watch?v=gsLIUtmTs8Q

#include <iostream>
#include "VanDerPolOscillatorFunctions.h" // for test purposes all functions moved in here

int main(int argc, char* argv[])
{
    std::cout << "Start of program ... \n" << std::endl;
    // Step 1: Initial conditions
    std::array<double, 2> x0 = { 1,0.1 }; // initial values of state space variables x1 and x2
    // please note that :
    // x1 = position
    // x2 = velocity 
    std::array<double, 3> theta = { -0.1,0.1,1 }; // theta parameters
    std::array<double, 3> temp_sysParameters = { 1,1,1 }; // system parameters (k,m,c)
    std::array<std::array<double, 3>, 8> sysParameters{ { // system parameters (k,m,c)
        { 1, 1, 1},
        { 1, 1,-1},
        { 1,-1, 1},
        { 1,-1,-1},
        {-1, 1, 1},
        {-1, 1,-1},
        {-1,-1, 1},
        {-1,-1,-1}
    } };

    std::cout << "Initial Postion is x0 = [ " << x0[0] << " , " << x0[1] << " ]" << std::endl;
    std::cout << "Initial Theta is theta = [ " << theta[0] << " , " << theta[1] << " , " << theta[2] << " ]" << std::endl;

    std::array<std::array<double, MAX_REPEATS>, 2> res;
    algorithms sel = FD2;
    Gnuplot gp("\"C:\\Program Files\\gnuplot\\bin\\gnuplot.exe\"");
    std::vector<double> v0, v1;

    /*
    // Step 1: Calculate using Finite Differences with 2 theta and m=c=k=1
    sel = FD2;
    res = gradientDescent(x0, theta, temp_sysParameters, "step1", sel);
    std::cout << "Final Performance is " << res[0][MAX_REPEATS - 1] << std::endl;
    for (int i = 0; i < MAX_REPEATS;i++)
    {
        v0.push_back(res[0][i]);
    }
    gp << "set title 'Graph of Performance over iterations for FD with 2 theta'\n";
    gp << "plot '-' with lines title 'v0'\n";
    gp.send(v0);

    std::cin.get();
    
    
    std::array<double, 10> constantParameters2 = { 0.01, 0.001, betta, gamma, alpha, A, a, p, gac, dt };
    // Step 2: Calculate using Finite Differences with 3 theta and m=c=k=1
    sel = FD3;
    res = gradientDescent(x0, theta, temp_sysParameters, "step2", sel, constantParameters2);
    std::cout << "Final Performance is " << res[0][MAX_REPEATS - 1] << std::endl;
    for (int i = 0; i < MAX_REPEATS;i++)
    {
        v0.push_back(res[0][i]);
    }
    gp << "set title 'Graph of Performance over iterations for FD with 3 theta'\n";
    gp << "plot '-' with lines title 'v0'\n";
    gp.send(v0);

    std::cin.get();
    
    
    // Step 3: Calculate using SPSA with 2 theta and m=c=k=1
    theta[0] = -0.1;
    theta[1] = 0.1;
    theta[2] = 1;

    sel = SPSA2;
    res = gradientDescent(x0, theta, temp_sysParameters, "step3", sel);
    std::cout << "Final Performance is P+ = " << res[0][MAX_REPEATS - 1] << " P- =  " << res[1][MAX_REPEATS - 1] << std::endl;
    for (int i = 0; i < MAX_REPEATS;i++)
    {
        v0.push_back(res[0][i]);
    }
    gp << "set title 'Graph of Performance over iterations for SPSA with 2 theta'\n";
    gp << "plot '-' with lines title 'v0'\n";
    gp.send(v0);

    std::cin.get();
    */
    /*
    // Step 4: Calculate using SPSA with 3 theta and m=c=k=1
    std::array<double, 10> constantParameters4 = { hetta, dtheta, 2, 0.01, 0.09, 0.1, 0.2, 0.5, gac, dt };
    theta[0] = 0.1;
    theta[1] = 0.2;
    theta[2] = 1;
    sel = SPSA3;
    res = gradientDescent(x0, theta, temp_sysParameters, "step4", sel,constantParameters4);
    std::cout << "Final Performance is P+ = " << res[0][MAX_REPEATS - 1] << " P- =  " << res[1][MAX_REPEATS - 1] << std::endl;
    for (int i = 0; i < MAX_REPEATS;i++)
    {
        v0.push_back(res[0][i]);
    }
    gp << "set title 'Graph of Performance over iterations for SPSA with 3 theta'\n";
    gp << "plot '-' with lines title 'v0'\n";
    gp.send(v0);

    std::cin.get();
    

    
    // Step 5: Calculate using Finite Differences with 2 theta and (m,c,k) = sysParameters
    for (int i = 0;i < 8;i++)
    {
        for (int j = 0;j < 3; j++)
        {
            temp_sysParameters[j] = sysParameters[i][j];
        }
        sel = FD2;
        res = gradientDescent(x0, theta, temp_sysParameters, "step5_" + std::to_string(i), sel);
        // Print the result
        std::cout << "Final Performance is " << res[0][MAX_REPEATS - 1] << std::endl;
        for (int k = 0; k < MAX_REPEATS;k++)
        {
            v0.push_back(res[0][k]);
        }
        gp << "set title 'Graph of Performance over iterations for FD with 2 theta'\n";
        gp << "plot '-' with lines title 'v0'\n";
        gp.send(v0);

        std::cin.get();
    }
    

    // Step 6: Calculate using Finite Differences with 3 theta and (m,c,k) = sysParameters
    for (int i = 0;i < 8;i++)
    {
        for (int j = 0;j < 3; j++)
        {
            temp_sysParameters[j] = sysParameters[i][j];
        }
        sel = FD3;
        res = gradientDescent(x0, theta, temp_sysParameters, "step6_" + std::to_string(i), sel);
        std::cout << "Final Performance is " << res[0][MAX_REPEATS - 1] << std::endl;
        for (int k = 0; k < MAX_REPEATS;k++)
        {
            v0.push_back(res[0][k]);
        }
        gp << "set title 'Graph of Performance over iterations for FD with 3 theta'\n";
        gp << "plot '-' with lines title 'v0'\n";
        gp.send(v0);

        std::cin.get();
    }
    
    
    // Step 7: Calculate using SPSA with 2 theta and (m,c,k) = sysParameters
    for (int i = 0;i < 8;i++)
    {
        for (int j = 0;j < 3; j++)
        {
            temp_sysParameters[j] = sysParameters[i][j];
        }
        sel = SPSA2;
        res = gradientDescent(x0, theta, temp_sysParameters, "step7_" + std::to_string(i), sel);
        std::cout << "Final Performance is P+ = " << res[0][MAX_REPEATS - 1] << " P- =  " << res[1][MAX_REPEATS - 1] << std::endl;
        for (int k = 0; k < MAX_REPEATS;k++)
        {
            v0.push_back(res[0][k]);
        }
        gp << "set title 'Graph of Performance over iterations for SPSA with 2 theta'\n";
        gp << "plot '-' with lines title 'v0'\n";
        gp.send(v0);

        std::cin.get();
    }
    

    // Step 8: Calculate using SPSA with 3 theta and (m,c,k) = sysParameters
    for (int i = 0;i < 8;i++)
    {
        for (int j = 0;j < 3; j++)
        {
            temp_sysParameters[j] = sysParameters[i][j];
        }
        sel = SPSA3;
        res = gradientDescent(x0, theta, temp_sysParameters, "step8_" + std::to_string(i), sel);
        std::cout << "Final Performance is P+ = " << res[0][MAX_REPEATS - 1] << " P- =  " << res[1][MAX_REPEATS - 1] << std::endl;
        for (int k = 0; k < MAX_REPEATS;k++)
        {
            v0.push_back(res[0][k]);
        }
        gp << "set title 'Graph of Performance over iterations for SPSA with 3 theta'\n";
        gp << "plot '-' with lines title 'v0'\n";
        gp.send(v0);

        std::cin.get();
    }
    */
    /*
    // Step 9: Calculate using Adaptive Control
    sel = AC;
    temp_sysParameters[0] = sysParameters[0][0];
    temp_sysParameters[0] = sysParameters[0][1];
    temp_sysParameters[0] = sysParameters[0][2];

    res = gradientDescent(x0, theta, temp_sysParameters, "step9", sel);
    std::cout << "Final Performance is P+ = " << res[0][MAX_REPEATS - 1] << " P- = " << res[1][MAX_REPEATS - 1] << std::endl;
    for (int i = 0; i < MAX_REPEATS;i++)
    {
        v0.push_back(res[0][i]);
    }
    gp << "set title 'Graph of Performance over iterations for AC'\n";
    gp << "plot '-' with lines title 'v0'\n";
    gp.send(v0);

    std::cin.get();
    */
    // Step 10: Calculate using Linear Quandratic Control
    /*
    x0 = { 0.1,0.1 };
    std::array<double, timeFinalLQR + 1> resLQR = lqrTop(x0, theta, temp_sysParameters, "step10", sel);
    for (int i = 0; i < MAX_REPEATS;i++)
    {
        v0.push_back(resLQR[i]);
    }
    gp << "set title 'Graph of Performance over iterations for LQR'\n";
    gp << "plot '-' with lines title 'v0'\n";
    gp.send(v0);

    std::cin.get();
    */
    /*
    // Step 11: Calculate using Adaptive Control with (m,c,k) = sysParameters
    for (int i = 0;i < 8;i++)
    {
        for (int j = 0;j < 3; j++)
        {
            temp_sysParameters[j] = sysParameters[i][j];
        }
        sel = AC;
        res = gradientDescent(x0, theta, temp_sysParameters, "step10", sel);
        std::cout << "Final Performance is P0 = " << res[0][MAX_REPEATS - 1] << " P0 = " << res[1][MAX_REPEATS - 1] << std::endl;
        for (int k = 0; k < MAX_REPEATS;k++)
        {
            v0.push_back(res[0][k]);
        }
        gp << "set title 'Graph of Performance over iterations for AC'\n";
        gp << "plot '-' with lines title 'v0'\n";
        gp.send(v0);

        std::cin.get();
    }
    */
    // Step 10: Sensitivity Analysis
    //sensitivityAnalysis();
    /*
    // Step 11: Plots
    Gnuplot gp("\"C:\\Program Files\\gnuplot\\bin\\gnuplot.exe\"");
    std::vector<double> v0, v1;
    for (int i = 0; i < MAX_REPEATS;i++)
    {
        v0.push_back(res[0][i]);
    }
    //std::partial_sum(v0.begin(), v0.end(), v0.begin());

    gp << "set title 'Graph of Performance over iterations'\n";
    gp << "plot '-' with lines title 'v0'\n";
    gp.send(v0);

    std::cin.get();
    */
    std::cout << "End of program ... \n" << std::endl;
    return 0;
}