
// About header guards: https://learn.microsoft.com/en-us/cpp/cpp/header-files-cpp?view=msvc-170#include-guards
#pragma once

#include "enumerators.h"
#include "systemModel.h"
#include "displayData.h"
#include <vector>
#include <random>
#include <stdexcept>

namespace vdpo {

    /*
    * The available Algorithms to control the system.
    * Contains Time Accessors.
    */
    class controlAlgorithm
    {
    public:
        bool displayGraphs = true;

        double x[2];

        double thetaVar[3];

        int numberOfThetaLocal;

        theta numberOfThetaType;

        bool sensitivityAnalysis;

        double simulationIterations = 0;

        controlAlgorithm();

        // Create an Algorithm instance. Choose algorithm and Number of theta parameters. Run the Sensitivity Analysis ?
        controlAlgorithm(algorithm selectAlgorithm, theta numberOfTheta, bool sensitivity);

        // Time Parameters Accessors

        // Set the time the simulation begins.
        void setStartTime(int setValue);

        // Set the time step of the simulation.
        void setStepTime(double setValue);

        // Set the time the simulation stops.
        void setFinalTime(int setValue);

        // Set the maximum iterations of the algorithm - A stop condition of the algorithm.
        void setMaxIterations(int setValue);

        // The time the simulation starts.
        int     getStartTime();

        // The time step of the simulation.
        double  getStepTime();

        // The time the simulation will stop.
        int     getFinalTime();

        // The maximum number of iterations - The stop condition of the algorithm.
        int     getMaxIterations();

        // The iterations of the simulation
        void iterationsCalculate();

        // Special Accessor
        // Set from the outside a system to use
        void setSystemModel(vdpo::systemModel explicitModel);

    private:
        vdpo::algorithm selectedAlgorithm;
    protected:
        // a plot object with default parameters
        vdpo::displayData plotSSV = vdpo::displayData::displayData();
        vdpo::displayData plotPerformance = vdpo::displayData::displayData();

        // system model to be used by the algorithms
        vdpo::systemModel localModel = vdpo::systemModel::systemModel(1.0,1.0,1.0,theta::two);

        // Setted by setSystemModel - whether to use user-instatiated model or let the program instatiate it
        bool useExplicitModel = false;

        // The STOP condition
        int maxRepeats = 100;

        // Simulation Start
        int startTime = 0;

        // Simulation Step
        double stepTime = 0.5;

        // Simulation Stop
        int finalTime = 100;

        // Used for End Creteria inside the algorithms
        double performanceThreshold = 1;
        double costThreshold = 1;

        // Perfomrance Storage - Used for diagrams only
        std::vector<double> performanceValue0;
        std::vector<double> performanceValue1;
        std::vector<double> performanceValue2;
        std::vector<double> performanceValue3;
        std::vector<double> performanceValue4;
        std::vector<double> performanceValue5;

        std::vector<double> costValue;

        // SSV Storage - Used for diagrams only
        std::vector<double> x1;
        std::vector<double> x2;
        std::vector<double> timeVector;
    };

    // Finite Differences with Performance
    class FD : public controlAlgorithm
    {
    public:
        // Constructors

        FD();

        FD(theta numberOfTheta, bool sensitivity);

        // Access Methods - Getters
        
        // Get slope
        double  getHetta();

        // Get theta change (Delta Theta)
        double  getDtheta();

        // Access Methods - Setters
        
        // set all parameters h , dtheta
        void setParameters(double parHetta, double parDtheta);
        
        // set all parameters [h,dtheta]
        void setParameters(double parameters[2]);

        // Set slope
        void    setHetta(double setValue);

        // Set theta change (Delta Theta)
        void    setDtheta(double setValue);

        // Performance Result
        double P = 0;

        void runAlgorithm();

    protected:
        // Algorithm Parameters

        // The slope value of the algorithm
        double hetta = 0.01;

        // The theta change (Delta Theta)
        double dtheta = 0.001;
    private:
        // Algorithm Implementation
        void finiteDifferences();

        // Calculate Performance - Simulation
        void performance();

        // Sensitivity Analysis Implementation
        void sensitivityAnalyzer();
    };

    // Simultaneous Perturbation Stochastic Approximation with Performance
    class SPSA : public controlAlgorithm
    {
    public:
        // Constructors

        SPSA();

        SPSA(theta numberOfTheta, bool sensitivity);

        // Access Methods

        // Set betta value - used for ck
        void setBetta(double setValue);

        // Set gamma value - used for ck
        void setGamma(double setValue);

        // set A value - used for ak
        void setAlpha(double setValue);

        // Set the A value - used for ak
        void setABig(double setValue);

        // Set the a value - used for ak
        void setASmall(double setValue);

        // Set the probability of Delta k
        void setProbability(double setValue);

        // set all parameters [betta,gamma,alpha,A,a,p]
        void setParameters(double parameters[6]);

        // The betta value - used for ck
        double getBetta();

        // The gamma value - used for ck
        double getGamma();

        // The alpha value - used for ak
        double getAlpha();

        // The A value - used for ak
        double getABig();

        // The a value - used for ak
        double getASmall();

        // The probability p - used for Deltak
        double getProbability();

        // Performance result
        double P = 0;

        void runAlgorithm();
    protected:
        // Algorithm Parameters
        double betta = 1;
        double gamma = 0.1;
        double alpha = 1;
        double A = 0.1;
        double a = 0.1;
        double p = 0.5;
        double ck = 0;
        double ak = 0;
        double Dk[3] = { 0,0,0 };
    private:
        // Algorithm Implementation
        void spsa();

        // Calculate Performance
        void performance();

        // Sensitivity Analysis Implementation
        void sensitivityAnalyzer();
    };

    // Linear Quadratic Regulator with DARE and Cost Function
    class LQR : public controlAlgorithm
    {
    public:
        // Constructors

        LQR();

        LQR(double Q[4], double R);

        // NOTE:
        // 
        // Due to already known dimensions of the system dx = Ax+Bu we can omit the 
        // difficulty of general DARE and general LQR as well as NxM matrices.
        // In this way the code is simpler and more straight forward.
        // Therefore the getters and setters are limited to support this system.
        // The R matrix is only one element.
        // The Q matrix is 4 elements.
        // Neither of them have the flexibility to change to higher dimensions.
        // Additionally, the riccati function is already solved and the calculations
        // are made explicity. 
        //
        
        // Access Methods - Getters

        double* getQ();
        double  getR();
        double* getK();

        // Final calculated cost
        double getJ();

        // Final calculated performance
        double getP();

        // Access Methods - Setters

        // Input Q matrix elements like: 
        // Q11 = mat[0] , Q12 = mat[1] , Q21 = mat[2] , Q22 = mat[3] .
        void setQ(double matrix[4]);
        
        void setR(double val);

        // set both Q 2x2 and R element 
        void setMatrices(double matQ[4], double rValue);

        // Cost result
        double J = 0;

        // Performance result
        double P = 0;

        // run the implementation
        void runAlgorithm();
    protected:
        // Algorithm Parameters
        // Matrices
        double q = 1;
        double r = 1;
        double K[2];
        double R = r;
        double Q[4] = { q, 0, 0, q };
    private:

        // Algorithm Implementation
        void riccati();

        // Calculate Cost
        void cost();

        // Sensitivity Analysis Implementation
        void sensitivityAnalyzer();
    };

    // Adaptive Control - NOT WORKING
    class AC :public controlAlgorithm
    {
    public:
        // Constructors

        AC();

        AC(theta numberOfTheta, bool sensitivity);

        // Access Methods
        void setGamma(double setValue);
        void setK1(double setValue);
        void setK2(double setValue);

        double getGamma();
        double getk1();
        double getk2();

        // Value result
        double V = 0;

        void runAlgorithm();
    protected:
        // Algorithm Parameters
        double gamma = 0.01;
        double k1 = 0.5;
        double k2 = 0.5;
    private:
        // Algorithm Implementation
        void adaptiveControl();

        // Calculate Performance
        void value();

        // Sensitivity Analysis Implementation
        void sensitivityAnalyzer();
    };
}
