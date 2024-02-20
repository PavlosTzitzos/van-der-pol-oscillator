
#include "VanDerPolOscillatorFunctions.h"

std::array<double, 2> f(std::array<double, 2> x, double u_local, double k = 1, double m = 1, double c = 1)
{

    //tex:
    //$\begin{align*} \dot{\vec{x}} = f(\vec{x},u) \end{align*}$

    //tex:
    //$\begin{align*} \left[ \begin{matrix} \dot{x_1} \\ \dot{x_2} \end{matrix} \right] = \begin{bmatrix} x_2 \\ - \frac{c}{m} \cdot(x_1 ^ 2 - 1) \cdot x_2 - \frac{k}{m} \cdot x_1 + \frac{u}{m} \end{bmatrix} \end{align*}$


    std::array<double, 2> d_x{ 0,0 }; // derivative of x

    d_x[0] = x[1];
    d_x[1] = -(c / m) * (std::pow(x[0], 2) - 1) * x[1] - (k / m) * x[0] + (u_local / m);

    return d_x;
}

std::array<double, 2> fLQR(std::array<double, 2> x, double u_local)
{

    //tex:
    //$\begin{align*} \dot{\vec{x}} = f(\vec{x},u) \end{align*}$

    //tex:
    //$\begin{align*} \left[ \begin{matrix} \dot{x_1} \\ \dot{x_2} \end{matrix} \right] = \bar{A} \cdot \vec{x} + \bar{B} \cdot \bar{u} = \left[ \begin{matrix} 0 && 1 \\ 0 && 0 \end{matrix} \right] \cdot \left[ \begin{matrix} x_1 \\ x_2 \end{matrix} \right] + \left[ \begin{matrix} 0 \\ 1 \end{matrix} \right] \cdot \bar{u} \end{align*}$


    std::array<double, 2> d_x{ 0,0 }; // derivative of x

    d_x[0] = x[1];
    d_x[1] = u_local;

    return d_x;
}

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

double u3(std::array<double, 2> x, std::array<double, 3> theta)
{
    //tex:
    //$\begin{align*} u(\vec{\theta} ,\vec{x}) = \theta_1 \cdot x_1 + \theta_2 \cdot x_2 + \theta_3 \cdot x_2 \cdot (x_1 ^2 - 1) \end{align*}$

    double u = theta[0] * x[0] + theta[1] * x[1] - theta[2] * x[1] * (std::pow(x[0], 2) - 1);

    return u;
}

double u2(std::array<double, 2> x, std::array<double, 3> theta)
{
    //tex:
    //$\begin{align*} u(\vec{\theta} ,\vec{x}) = \theta_1 \cdot x_1 + \theta_2 \cdot x_2 \end{align*}$

    double u = theta[0] * x[0] + theta[1] * x[1];

    return u;
}

double uLQR(std::array<double, 2> x, std::array<double, 2> K)
{
    //tex:
    //$\begin{align*} u(\vec{K} ,\vec{x}) = k_1 \cdot x_1 + k_2 \cdot x_2 \end{align*}$

    double u = K[0] * x[0] + K[1] * x[1];

    return u;
}

double uAC(std::array<double, 2> x, std::array<double, 2> thetaApprox, double m = 1)
{
    //tex:
    //$\begin{align*} u(\vec{x},\vec{\hat{\theta}}) = \hat{\theta}_1 \cdot x_2 \cdot (x_1 - 1)^2 + \hat{\theta}_2 \cdot x_1 + k_1 \cdot x_1 + k_2 \cdot x_2 \end{align*}$

    double u = m * (thetaApprox[0] * x[1] * (x[0] - 1) * (x[0] - 1) + thetaApprox[1] * x[0] + k1 * x[0] + k2 * x[1]);

    return u;
}

std::array<std::array<double, timeFinal + 1>, 3> performance(std::array<double, 2> x_old, std::array<double, 3> theta, int theta_sel, std::array<double, 3> systemParameters = { 1,1,1 }, std::string filename = "dump.txt")
{
    //std::ofstream results;
    //results.open(filename, std::ofstream::app);
    //results << "\n" << std::endl;
    //results << "t" << "\t" << "x[0]" << "\t" << "x[1]" << "\t" << "P" << "\t" << "theta[0]" << "\t" << "theta[1]" << "\t" << "theta[2]" << std::endl;

    double P = 0;
    std::array<double, 2> x_new;
    x_new[0] = x_old[0];
    x_new[1] = x_old[1];

    std::array<double, timeFinal> norm;

    std::array<std::array<double, timeFinal + 1>, 3> res;

    for (int i = 0; i < timeFinal; i++)
        norm[i] = 0;

    res[0][0] = P;
    res[1][0] = x_old[0];
    res[2][0] = x_old[1];

    for (int t = 0; t < timeFinal; t++)
    {
        // Store x values into vectors to plot them later.

        // Calculate the next vector x_new:
        //tex:
        //$\begin{align*} \vec{x}_{new} = \vec{x}_{old} + dt \cdot \dot{\vec{x}} = \vec{x}_{old} + dt \cdot f(\vec{x}_{old} , u(\vec{x}_{old} , \vec{\theta} ) ) \end{align*}$

        if (theta_sel == 0)
        {
            x_new[0] = x_old[0] - timeStep * f(x_old, u2(x_old, theta), systemParameters[0], systemParameters[1], systemParameters[2])[0];
            x_new[1] = x_old[1] - timeStep * f(x_old, u2(x_old, theta), systemParameters[0], systemParameters[1], systemParameters[2])[1];
        }
        else if (theta_sel == 1)
        {
            x_new[0] = x_old[0] - timeStep * f(x_old, u3(x_old, theta), systemParameters[0], systemParameters[1], systemParameters[2])[0];
            x_new[1] = x_old[1] - timeStep * f(x_old, u3(x_old, theta), systemParameters[0], systemParameters[1], systemParameters[2])[1];
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


        //if (results.is_open())
        //{
        //    results << t << "\t" << x_new[0] << "\t" << x_new[1] << "\t" << norm[t] << "\t" << P << "\t" << theta[0] << "\t" << theta[1] << "\t" << theta[2] << std::endl;
        //}
        //else std::cout << "\nUnable to open file\n";

        res[0][t + 1] = P;
        res[1][t + 1] = x_old[0];
        res[2][t + 1] = x_old[1];
    }

    return res;
}

std::array<std::array<double, timeFinalLQR + 1>, 3> performanceLQR(std::array<double, 2> x_old, std::array<double, 2> K, std::string filename = "dump.txt")
{
    std::ofstream results;
    results.open(filename, std::ofstream::app);
    results << "\n" << std::endl;
    results << "t" << "\t" << "x[0]" << "\t" << "x[1]" << "\t" << "P" << "\t" << "theta[0]" << "\t" << "theta[1]" << "\t" << "theta[2]" << std::endl;

    double P = 0;
    std::array<double, 2> x_new;
    x_new[0] = x_old[0];
    x_new[1] = x_old[1];

    std::array<double, timeFinalLQR> norm;

    std::array<std::array<double, timeFinalLQR + 1>, 3> res;

    for (int i = 0; i < timeFinalLQR; i++)
        norm[i] = 0;

    res[0][0] = P;
    res[1][0] = x_old[0];
    res[2][0] = x_old[1];

    for (int t = 0; t < timeFinalLQR; t++)
    {
        // Store x values into vectors to plot them later.

        // Calculate the next vector x_new:
        //tex:
        //$\begin{align*} \vec{x}_{new} = \vec{x}_{old} + dt \cdot \dot{\vec{x}} = \vec{x}_{old} + dt \cdot f(\vec{x}_{old} , u(\vec{x}_{old} , \vec{\theta} ) ) \end{align*}$

        x_new[0] = x_old[0] - timeStep * fLQR(x_old, uLQR(x_old, K))[0];
        x_new[1] = x_old[1] - timeStep * fLQR(x_old, uLQR(x_old, K))[1];

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
        res[0][t + 1] = P;
        res[1][t + 1] = x_old[0];
        res[2][t + 1] = x_old[1];
    }
    return res;
}

std::array<double, 4> riccati2(std::array<double, 4> matA, std::array<double, 2> matB, std::array<double, 4> matQ, std::array<double, 4> matPold, double R = 1)
{

    std::array<double, 4> transA;
    transA[0] = matA[0];
    transA[1] = matA[2];
    transA[2] = matA[1];
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
    //tex:
    //$\begin{align*} A^T \cdot P_{old} \cdot A \end{align*}$

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
    for (int i = 0; i < 4; i++)
    {
        matPnew[i] = matQ[i] + mat1[i] - mat5[i];
        //std::cout << matPnew[i] << " ";
    }
    //std::cout << "]\n";
    return matPnew;
}

std::array<double, 2> lqr(std::array<double, 2>x_old, double q, double r)
{
    double matR = r;
    double invR = 1 / r;

    std::array<double, 4> matQ = { q, 0, 0, q };

    std::array<double, 4> tempP, matP;
    tempP[0] = matQ[0];
    tempP[1] = matQ[1];
    tempP[2] = matQ[2];
    tempP[3] = matQ[3];

    std::array<double, 4> matA = { 0, 1, 0, 0 };

    std::array<double, 2> matB = { 0 , 1 };

    std::array<double, 2> transB = { 0, 1 };

    // Step 1 : Calculate the P matrix backwards using algebraic riccati equation
    //tex:
    //$\begin{align*} P =  Q + A^T P A - A^T P B (R + B^T P B)^{-1} B^T P A\end{align*}$
    for (int i = N - 1; i >= 0; i--) // riccati iterations
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

    //matK[0] = 1/(matR + (transB[0] * matP[0] + transB[1] * matP[2]) * matB[0]) * ( (transB[0] * matP[0] + transB[1] * matP[2]) * matA[0] + (transB[0] * matP[1] + transB[1] * matP[3]) * matA[1] );
    //matK[1] = 1/(matR + (transB[0] * matP[1] + transB[1] * matP[3]) * matB[1]) * ( (transB[0] * matP[0] + transB[1] * matP[2]) * matA[2] + (transB[0] * matP[1] + transB[1] * matP[3]) * matA[3] );


    std::array<double, 2> matK = { 0,0 };

    matK[0] = invR * (transB[0] * matP[0] + transB[1] * matP[2]);
    matK[1] = invR * (transB[0] * matP[1] + transB[1] * matP[3]);

    return matK;
}

std::array<std::array<double, timeFinal + 3>, 2> value(std::array<double, 2> x, std::array<double, 2> thetaA, std::array<double, 2> thetaR, std::array<double, 2> dThetaE, double m = 1)
{
    std::array<std::array<double, timeFinal + 3>, 2> res;
    int counter = 0;
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
    while (std::abs(dW) > 0.01 && counter < timeFinal)
    {
        // Step 1: Calculate A1 and A2
        //tex:
        //$\begin{align*} A_1 = x_2 \cdot (x_1^2 -1 ) \cdot (x_2 \frac{1 - k_1}{k_1 k_2} - \frac{x_1}{k_1}) \end{align*}$


        A1 = -x[1] * (x[0] - 1) * (x[0] - 1) * (x[1] * (1 - k1) / (k1 * k2) - x[0] / k1);

        //tex:
        //$\begin{align*} A_2 = x_1 ( x_2 \frac{1 - k_1}{k_1 k_2} - \frac{x_1}{k_1} ) \end{align*}$


        A2 = x[0] * (x[1] * (1 - k1) / (k1 * k2) - x[0] / k1);

        // Step 2: Calculate dW
        //tex:
        //$\begin{align*} \dot{V} = - x_1 ^2 - x_2 ^2 + A_1 \tilde{\theta}_1 + A_2 \tilde{\theta}_2 \end{align*}$

        dV = -1 * x[0] * x[0] - x[1] * x[1] + theta_error[0] * A1 + theta_error[1] * A2;

        //tex:
        //$\begin{align*} \dot{W} = \dot{V} + \frac{1}{\gamma} \tilde{\theta}_1 \dot{\tilde{\theta}}_1 + \frac{1}{\gamma} \tilde{\theta}_2 \dot{\tilde{\theta}}_2 \end{align*}$


        dW = dV + (theta_error[0] * dThetaE[0]) / gac + (theta_error[1] * dThetaE[1]) / gac;

        // Step 3: Calculate the next vector x_new:
        //tex:
        //$\begin{align*} \vec{x}_{new} = \vec{x}_{old} + dt \cdot \dot{\vec{x}} = \vec{x}_{old} + dt \cdot f(\vec{x}_{old} , u(\vec{x}_{old} , \vec{\theta} ) ) \end{align*}$

        thetaA[0] = thetaR[0] - theta_error[0];
        thetaA[1] = thetaR[1] - theta_error[1];

        x_new[0] = x[0] + timeStep * fAC(x, uAC(x, thetaA), thetaR, m)[0];
        x_new[1] = x[1] + timeStep * fAC(x, uAC(x, thetaA), thetaR, m)[1];

        // NOTE : maybe x_new is needed in the future
        x[0] = x_new[0];
        x[1] = x_new[1];

        res[0][counter] = x[0];
        res[1][counter] = x[1];
        counter++;
    }
    res[0][counter + 1] = A1;
    res[1][counter + 1] = A2;
    res[0][timeFinal + 2] = counter;
    return res;
}

std::array<double, timeFinalLQR + 1> lqrTop(std::array<double, 2> x0, std::array<double, 3> theta, std::array<double, 3> systemParameters = { 1,1,1 }, std::string st = "", algorithms algo_sel = FD2, std::array<double, 10> constantParameters = { hetta, dtheta, betta, gamma, alpha, A, a, p, gac, dt })
{

    std::array<double, timeFinalLQR> v0, v1;

    std::array<std::array<double, timeFinalLQR + 1>, 3> resPerf;
    double Perf = 0.01;
    int counter = 0;
    std::array<double, 2> local_x, K;
    local_x[0] = x0[0];
    local_x[1] = x0[1];


    std::array<double, timeFinalLQR + 1> ret;

    while (std::abs(Perf) < 10 && counter < MAX_REPEATS)
    {
        std::ofstream results;
        std::string filename = "results_lqr_" + std::to_string(counter) + ".txt";
        results.open(filename);
        results << "Something " << std::endl;

        // Step 1 : Calculate K matrix
        K = lqr(local_x, 1, 1);
        //results << "\n" << "Current K : [ " << K[0] << " , " << K[1] << " ] " << std::endl;

        // Step 2 : Calculate performance
        resPerf = performanceLQR(x0, K, filename);

        // Prepare data for diagram :
        for (int i = 0; i < timeFinalLQR; i++)
        {
            ret[i] = resPerf[0][i];
            if (std::abs(resPerf[1][i]) > 1000 || isnan<double>(resPerf[1][i]))
                v0[i] = 0; // x1
            else
                v0[i] = resPerf[1][i];
            if (std::abs(resPerf[2][i]) > 1000 || isnan<double>(resPerf[2][i]))
                v1[i] = 0; // x2
            else
                v1[i] = resPerf[2][i];
        }

        Perf = resPerf[0][timeFinalLQR];
        // Close file
        results.close();

        // Check for anomalies
        if (isinf<double>(Perf) || isnan<double>(Perf)) break;

        counter += 1;
    }
    Gnuplot gp("\"C:\\Program Files\\gnuplot\\bin\\gnuplot.exe\"");

    gp << "set title 'Graph of x1 and x2 over iterations for LQR'\n";
    gp << "plot '-' with lines title 'x1',"
        << "'-' with lines title 'x2'\n";
    gp.send(v0);
    gp.send(v1);

    std::cin.get();
    return ret;
}

std::array<std::array<double, MAX_REPEATS>, 2> gradientDescent(std::array<double, 2> x0, std::array<double, 3> theta, std::array<double, 3> systemParameters = { 1,1,1 }, std::string st = "", algorithms algo_sel = FD2, std::array<double, 10> constantParameters = { hetta, dtheta, betta, gamma, alpha, A, a, p, gac, dt })
{
    double local_hetta = constantParameters[0];
    double local_dtheta = constantParameters[1];
    double local_betta = constantParameters[2];
    double local_gamma = constantParameters[3];
    double local_alpha = constantParameters[4];
    double local_A = constantParameters[5];
    double local_a = constantParameters[6];
    double local_p = constantParameters[7];
    double local_gac = constantParameters[8];
    double local_dt = constantParameters[9];

    std::array<std::array<double, MAX_REPEATS>, 2> P_res;

    std::array<double, timeFinal> v0, v1;

    std::array<std::array<double, timeFinal + 1>, 3> resPerf;

    /** Used by AC
    * column 0: x1
    * column 1: x2
    * column 2: A1
    * column 3: A2
    * column 4: only one element a counter (how many iterations the value function made)
    */

    std::array<std::array<double, timeFinal + 3>, 2> resPerfAC;

    for (int i = 0; i < 2; i++)
        for (int j = 0; j < MAX_REPEATS; j++)
            P_res[i][j] = 0;

    switch (algo_sel)
    {
    case FD2: // Finite Differences Algorithm with 2 theta
    {

        std::array<double, 4> P = { 0,0,0,0 };
        double Perf = 100;

        std::cout << "Initial Perf = [ " << P[0] << " , " << P[1] << " , " << P[2] << " , " << P[3] << " ]" << std::endl;

        int counter = 0;
        while (std::abs(Perf) > 10 && counter < MAX_REPEATS)
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

            resPerf = performance(x0, local_theta, 0, systemParameters, filename);
            local_theta[0] = theta[0] + local_dtheta;
            for (int i = 0; i < timeFinal; i++)
            {
                v0[i] = resPerf[1][i];
                v1[i] = resPerf[2][i];
            }
            P[0] = resPerf[0][timeFinal];

            resPerf = performance(x0, local_theta, 0, systemParameters, filename);
            local_theta[0] = theta[0];
            local_theta[1] = theta[1] + local_dtheta;
            P[1] = resPerf[0][timeFinal];

            resPerf = performance(x0, local_theta, 0, systemParameters, filename);
            local_theta[1] = theta[1];
            P[2] = resPerf[0][timeFinal];

            // Calculate new theta :
            //tex:
            //$\begin{align*} \theta_{i+1} = \theta_i - \eta \cdot \frac{P_i(x_0 , \theta_i + \Delta \theta) - P_i(x_0 , \theta_i)}{\Delta \theta} \end{align*}$


            theta[0] = theta[0] - local_hetta * (P[1] - P[0]) / local_dtheta;
            theta[1] = theta[1] - local_hetta * (P[2] - P[0]) / local_dtheta;

            // Save performance
            P_res[0][counter] = Perf = P[0];

            // Close file
            results.close();

            // Check for anomalies
            if (isinf<double>(P[0]) || isnan<double>(P[0])) break;
            if (isinf<double>(P[1]) || isnan<double>(P[1])) break;
            if (isinf<double>(P[2]) || isnan<double>(P[2])) break;
            if (isinf<double>(P[3]) || isnan<double>(P[3])) break;

            counter += 1;
        }
        Gnuplot gp("\"C:\\Program Files\\gnuplot\\bin\\gnuplot.exe\"");

        gp << "set title 'Graph of x1 and x2 over iterations for FD with 2 theta'\n";
        gp << "plot '-' with lines title 'x1',"
            << "'-' with lines title 'x2'\n";
        gp.send(v0);
        gp.send(v1);

        std::cin.get();
        return P_res;
    }
    case FD3: // Finite Differences Algorithm with 3 theta
    {
        std::array<double, 4> P = { 0,0,0,0 };
        double Perf = 100;

        std::cout << "Initial Perf = [ " << P[0] << " , " << P[1] << " , " << P[2] << " , " << P[3] << " ]" << std::endl;

        int counter = 0;
        while (std::abs(Perf) > 10 && counter < MAX_REPEATS)
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

            resPerf = performance(x0, local_theta, 1, systemParameters, filename);
            local_theta[0] = theta[0] + local_dtheta;
            for (int i = 0; i < timeFinal; i++)
            {
                if (std::abs(resPerf[1][i]) > 1000)
                    v0[i] = 0;
                else
                    v0[i] = resPerf[1][i];
                if (std::abs(resPerf[2][i]) > 1000)
                    v1[i] = 0;
                else
                    v1[i] = resPerf[2][i];
            }
            P[0] = resPerf[0][timeFinal];
            resPerf = performance(x0, local_theta, 1, systemParameters, filename);
            local_theta[0] = theta[0];
            local_theta[1] = theta[1] + local_dtheta;
            P[1] = resPerf[0][timeFinal];
            resPerf = performance(x0, local_theta, 1, systemParameters, filename);
            local_theta[1] = theta[1];
            local_theta[2] = theta[2] + local_dtheta;
            P[2] = resPerf[0][timeFinal];
            resPerf = performance(x0, local_theta, 1, systemParameters, filename);
            local_theta[2] = theta[2];
            P[3] = resPerf[0][timeFinal];
            // Calculate new theta :
            //tex:
            //$\begin{align*} \theta_{i+1} = \theta_i - \eta \cdot \frac{P_i(x_0 , \theta_i + \Delta \theta) - P_i(x_0 , \theta_i)}{\Delta \theta} \end{align*}$


            theta[0] = theta[0] - local_hetta * (P[1] - P[0]) / local_dtheta;
            theta[1] = theta[1] - local_hetta * (P[2] - P[0]) / local_dtheta;
            theta[2] = theta[2] - local_hetta * (P[3] - P[0]) / local_dtheta;

            // Save performance
            P_res[0][counter] = Perf = P[0];

            // Close file
            results.close();

            // Check for anomalies
            if (isinf<double>(P[0]) || isnan<double>(P[0])) break;
            if (isinf<double>(P[1]) || isnan<double>(P[1])) break;
            if (isinf<double>(P[2]) || isnan<double>(P[2])) break;
            if (isinf<double>(P[3]) || isnan<double>(P[3])) break;

            counter += 1;
        }
        Gnuplot gp("\"C:\\Program Files\\gnuplot\\bin\\gnuplot.exe\"");

        gp << "set title 'Graph of x1 and x2 over iterations for FD with 3 theta'\n";
        gp << "plot '-' with lines title 'x1',"
            << "'-' with lines title 'x2'\n";
        gp.send(v0);
        gp.send(v1);

        std::cin.get();
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

        for (int i = 0; i < MAX_REPEATS; i++)
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

            ck = local_betta / std::powl(i + 1, local_gamma);

            //Calculate gain a_k :
            //tex:
            //$\begin{align*} a_k = \frac{a}{(A + k)^\alpha} \end{align*}$

            ak = local_a / std::powl(local_A + i + 1, local_alpha);

            // Step 2 : Generation of the simultaneous perturbation vector
            std::random_device rd;
            std::mt19937 gen(rd());
            std::bernoulli_distribution d(local_p);
            for (int i = 0; i < 2; i++)
                Dk[i] = d(gen);

            // Step 3 : Calculate Performance
            //tex:
            //$\begin{align*} P_i = perfomrnace(x_0 , \theta_i + \Delta \theta) \end{align*}$

            // Calculate Positive P_i ( aka y_plus ) :
            local_theta[0] = theta[0] + ck * Dk[0];
            resPerf = performance(x0, local_theta, 0, systemParameters, filename);
            local_theta[0] = theta[0];
            local_theta[1] = theta[1] + ck * Dk[1];

            // Prepare data for diagram :
            for (int i = 0; i < timeFinal; i++)
            {
                if (std::abs(resPerf[1][i]) > 10000000 || isnan<double>(resPerf[1][i]))
                    v0[i] = 0;
                else
                    v0[i] = resPerf[1][i];
                if (std::abs(resPerf[2][i]) > 10000000 || isnan<double>(resPerf[2][i]))
                    v1[i] = 0;
                else
                    v1[i] = resPerf[2][i];
            }
            P[0] = resPerf[0][timeFinal];
            resPerf = performance(x0, local_theta, 0, systemParameters, filename);
            local_theta[1] = theta[1];

            // Calculate Negative P_i ( aka y_minus ) :
            local_theta[0] = theta[0] - ck * Dk[0];
            P[1] = resPerf[0][timeFinal];
            resPerf = performance(x0, local_theta, 0, systemParameters, filename);
            local_theta[0] = theta[0];
            local_theta[1] = theta[1] - ck * Dk[1];
            P[3] = resPerf[0][timeFinal];
            resPerf = performance(x0, local_theta, 0, systemParameters, filename);
            local_theta[1] = theta[1];
            P[4] = resPerf[0][timeFinal];

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
            if (isinf<double>(P[0]) || isnan<double>(P[0])) break;
            if (isinf<double>(P[1]) || isnan<double>(P[1])) break;
            if (isinf<double>(P[2]) || isnan<double>(P[2])) break;
            if (isinf<double>(P[3]) || isnan<double>(P[3])) break;
            if (isinf<double>(P[4]) || isnan<double>(P[4])) break;
            if (isinf<double>(P[5]) || isnan<double>(P[5])) break;

            // Save Performance
            P_res[0][i] = Perf = P[0];
            P_res[1][i] = Perf = P[3];

            if (end_creteria[0] < SPSA_END || end_creteria[1] < SPSA_END)
                break;
        }
        Gnuplot gp("\"C:\\Program Files\\gnuplot\\bin\\gnuplot.exe\"");

        gp << "set title 'Graph of x1 and x2 over iterations for SPSA with 2 theta'\n";
        gp << "plot '-' with lines title 'x1',"
            << "'-' with lines title 'x2'\n";
        gp.send(v0);
        gp.send(v1);

        std::cin.get();
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

        for (int i = 0; i < MAX_REPEATS; i++)
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

            ck = local_betta / powl(i + 1, local_gamma);

            //Calculate gain a_k :
            //tex:
            //$\begin{align*} a_k = \frac{a}{(A + k)^\alpha} \end{align*}$

            ak = local_a / powl(local_A + i + 1, local_alpha);

            // Step 2 : Generation of the simultaneous perturbation vector
            std::random_device rd;
            std::mt19937 gen(rd());
            std::bernoulli_distribution d(local_p);
            for (int i = 0; i < 3; i++)
                Dk[i] = d(gen);

            // Step 3 : Calculate Performance
            //tex:
            //$\begin{align*} P_i = perfomrnace(x_0 , \theta_i + \Delta \theta) \end{align*}$

            // Calculate Positive P_i ( aka y_plus ) :
            local_theta[0] = theta[0] + ck * Dk[0];
            resPerf = performance(x0, local_theta, 1, systemParameters, filename);
            local_theta[0] = theta[0];
            local_theta[1] = theta[1] + ck * Dk[1];

            // Prepare data for diagram :
            for (int i = 0; i < timeFinal; i++)
            {
                if (std::abs(resPerf[1][i]) > 1000 || isnan<double>(resPerf[1][i]))
                    v0[i] = 0;
                else
                    v0[i] = resPerf[1][i];
                if (std::abs(resPerf[2][i]) > 1000 || isnan<double>(resPerf[2][i]))
                    v1[i] = 0;
                else
                    v1[i] = resPerf[2][i];
            }
            P[0] = resPerf[0][timeFinal];
            resPerf = performance(x0, local_theta, 1, systemParameters, filename);
            local_theta[1] = theta[1];
            local_theta[2] = theta[2] + ck * Dk[2];
            P[1] = resPerf[0][timeFinal];
            resPerf = performance(x0, local_theta, 1, systemParameters, filename);
            local_theta[2] = theta[2];
            P[2] = resPerf[0][timeFinal];

            // Calculate Negative P_i ( aka y_minus ) :
            local_theta[0] = theta[0] - ck * Dk[0];
            resPerf = performance(x0, local_theta, 1, systemParameters, filename);
            local_theta[0] = theta[0];
            local_theta[1] = theta[1] - ck * Dk[1];
            P[3] = resPerf[0][timeFinal];
            resPerf = performance(x0, local_theta, 1, systemParameters, filename);
            local_theta[1] = theta[1];
            local_theta[2] = theta[2] - ck * Dk[2];
            P[4] = resPerf[0][timeFinal];
            resPerf = performance(x0, local_theta, 1, systemParameters, filename);
            local_theta[2] = theta[2];
            P[5] = resPerf[0][timeFinal];

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
            P_res[0][i] = Perf = P[0];
            P_res[1][i] = Perf = P[3];

            if (end_creteria[0] < SPSA_END || end_creteria[1] < SPSA_END || end_creteria[2] < SPSA_END)
                break;
        }
        Gnuplot gp("\"C:\\Program Files\\gnuplot\\bin\\gnuplot.exe\"");

        gp << "set title 'Graph of x1 and x2 over iterations for SPSA with 3 theta'\n";
        gp << "plot '-' with lines title 'x1',"
            << "'-' with lines title 'x2'\n";
        gp.send(v0);
        gp.send(v1);

        std::cin.get();
        return P_res;
    }
    case LQR: // Linear Quadratic Regulator Algorithm
    {
        // DO NOT USE THIS SWITCH CASE
        return P_res;
    }
    case AC: // Adaptive Control ALgorithm
    {
        std::array<double, 2> P = { 0,0 };
        std::array<double, 2> thetaHat = { theta[0],theta[1] };
        std::array<double, 2> dThetaError = { 0,0 };
        std::array<double, 3> end_creteria;

        // System Parameters :
        double m = systemParameters[1];
        double c = systemParameters[2];
        double k = systemParameters[0];
        std::array<double, 2> thetaR{ -c / m, -k / m }; // theta Real

        // NOTE :

        // Theoretical symbol :
        //tex:
        // $\begin{align*} \hat{\theta}\end{align*}$ 

        //is actually the variable theta

        for (int i = 0; i < MAX_REPEATS; i++)
        {
            // Filename to save data
            std::ofstream results;
            std::string filename = "results_ac_" + std::to_string(i) + ".txt";
            results.open(filename);

            dThetaError[0] = local_gac * P[0];
            dThetaError[1] = local_gac * P[1];

            // Step 1 : Calculate Gradient of theta hat
            resPerfAC = value(x0, thetaHat, thetaR, dThetaError, m);

            int counter = resPerfAC[0][timeFinal + 2];
            P[0] = resPerfAC[0][counter + 1];
            P[1] = resPerfAC[1][counter + 1];

            // Step 2 : Calculate New theta hat
            //tex:
            // $\begin{align*} \vec{\hat{\theta}} = \vec{\hat{\theta}} - d \theta \cdot \gamma \cdot A \end{align*}$

            theta[0] = theta[0] - local_dt * local_gac * P[0];
            theta[1] = theta[1] - local_dt * local_gac * P[1];

            // Step 4 : Iteration or termination

            end_creteria[0] = std::abs(local_dt * local_gac * P[0]);
            end_creteria[1] = std::abs(local_dt * local_gac * P[1]);

            // Check for anomalies
            if (isinf<double>(P[0]) || isnan<double>(P[0])) break;
            if (isinf<double>(P[1]) || isnan<double>(P[1])) break;

            // Save Performance value
            //P_res[0][i] = P[0];
            //P_res[1][i] = P[1];


            if (end_creteria[0] > AC_END || end_creteria[1] > AC_END)
                break;
        }
        for (int i = 0; i < resPerfAC[0][timeFinal + 2]; i++)
        {
            if (std::abs(resPerfAC[0][i]) > 10000000 || isnan<double>(resPerfAC[0][i]))
                v0[i] = 0;
            else
                v0[i] = P[i];
            if (std::abs(resPerfAC[1][i]) > 10000000 || isnan<double>(resPerfAC[1][i]))
                v1[i] = 0;
            else
                v1[i] = resPerf[2][i];
        }
        Gnuplot gp("\"C:\\Program Files\\gnuplot\\bin\\gnuplot.exe\"");

        gp << "set title 'Graph of x1 and x2 over iterations for SPSA with 2 theta'\n";
        gp << "plot '-' with lines title 'x1',"
            << "'-' with lines title 'x2'\n";
        gp.send(v0);
        gp.send(v1);

        std::cin.get();
        return P_res;
    }
    default:
        return P_res;
    }
}

void sensitivityAnalysis(std::array<double, 2> xInit = { 1,1 }, std::array<double, 3> theta = { 1,1,0 }, std::array<double, 3> sysPar = { 1,1,1 }, double max = 1, double step = 1, double min = 0)
{
    std::array<std::array<double, MAX_REPEATS>, 2> res; //the performances
    algorithms selectAlgorithm = FD2;
    double i = min;
    // FD
    // Vary hetta only in FD2
    double localVar = 0; // if you want leave default the min, max, step and play with localVar only eg using rand instead of i
    std::array<double, 10> constPar = { hetta, dtheta, betta, gamma, alpha, A, a, p, gac, dt };
    for (i = min; i < max; i += step)
    {
        selectAlgorithm = FD2;
        localVar = i;
        constPar[0] = localVar;
        res = gradientDescent(xInit, theta, sysPar, "", selectAlgorithm, constPar);
    }
    constPar[0] = hetta;
    // Vary dtheta only in FD2
    for (i = 0; i < max; i += step)
    {
        selectAlgorithm = FD2;
        localVar = i;
        constPar[1] = localVar;
        res = gradientDescent(xInit, theta, sysPar, "", selectAlgorithm, constPar);
    }
    constPar[1] = dtheta;
    // Vary hetta only in FD3
    for (i = 0; i < max; i += step)
    {
        selectAlgorithm = FD3;
        localVar = i;
        constPar[0] = localVar;
        res = gradientDescent(xInit, theta, sysPar, "", selectAlgorithm, constPar);
    }
    constPar[0] = hetta;
    // Vary dtheta only in FD3
    for (i = 0; i < max; i += step)
    {
        selectAlgorithm = FD3;
        localVar = i;
        constPar[1] = localVar;
        res = gradientDescent(xInit, theta, sysPar, "", selectAlgorithm, constPar);
    }
    constPar[1] = dtheta;

    // SPSA
    // Vary betta only in SPSA2
    for (i = 0; i < max; i += step)
    {
        selectAlgorithm = SPSA2;
        localVar = i;
        constPar[2] = localVar;
        res = gradientDescent(xInit, theta, sysPar, "", selectAlgorithm, constPar);
    }
    constPar[2] = betta;
    // Vary gamma only in SPSA2
    for (i = 0; i < max; i += step)
    {
        selectAlgorithm = SPSA2;
        localVar = i;
        constPar[3] = localVar;
        res = gradientDescent(xInit, theta, sysPar, "", selectAlgorithm, constPar);
    }
    constPar[3] = gamma;
    // Vary alpha only in SPSA2
    for (i = 0; i < max; i += step)
    {
        selectAlgorithm = SPSA2;
        localVar = i;

        // do gradientDescent SPSA2
    }
    // Vary A only in SPSA2
    for (i = 0; i < max; i += step)
    {
        selectAlgorithm = SPSA2;
        localVar = i;

        // do gradientDescent SPSA2
    }
    // Vary a only in SPSA2
    for (i = 0; i < max; i += step)
    {
        selectAlgorithm = SPSA2;
        localVar = i;

        // do gradientDescent SPSA2
    }
    // Vary p only in SPSA2
    for (i = 0; i < max; i += step)
    {
        selectAlgorithm = SPSA2;
        localVar = i;

        // do gradientDescent SPSA2
    }

    // Vary betta only in SPSA3
    for (i = 0; i < max; i += step)
    {
        selectAlgorithm = SPSA3;
        localVar = i;

        // do gradientDescent SPSA3
    }
    // Vary gamma only in SPSA3
    for (i = 0; i < max; i += step)
    {
        selectAlgorithm = SPSA3;
        localVar = i;

        // do gradientDescent SPSA3
    }
    // Vary alpha only in SPSA3
    for (i = 0; i < max; i += step)
    {
        selectAlgorithm = SPSA3;
        localVar = i;

        // do gradientDescent SPSA3
    }
    // Vary A only in SPSA3
    for (i = 0; i < max; i += step)
    {
        selectAlgorithm = SPSA3;
        localVar = i;

        // do gradientDescent SPSA3
    }
    // Vary a only in SPSA3
    for (i = 0; i < max; i += step)
    {
        selectAlgorithm = SPSA3;
        localVar = i;

        // do gradientDescent SPSA3
    }
    // Vary p only in SPSA3
    for (i = 0; i < max; i += step)
    {
        selectAlgorithm = SPSA3;
        localVar = i;

        // do gradientDescent SPSA3
    }

    // AC
    // Vary gamma (gac) only in AC
    for (i = 0; i < max; i += step)
    {
        selectAlgorithm = AC;
        localVar = i;

        // do gradientDescent AC
    }
    // Vary dt only in AC
    for (i = 0; i < max; i += step)
    {
        selectAlgorithm = AC;
        localVar = i;

        // do gradientDescent AC
    }
}
