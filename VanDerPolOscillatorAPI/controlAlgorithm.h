
// About header guards: https://learn.microsoft.com/en-us/cpp/cpp/header-files-cpp?view=msvc-170#include-guards
#pragma once

//#ifndef CONTROL_ALGORITHM_INCLUDE
//#define CONTROL_ALGORITHM_INCLUDE

#include "enumerators.h"
#include "systemModel.h"
#include "displayData.h"
#include <vector>

namespace vdpo {

    /*
    * The available Algorithms to control the system.
    * Contains Time Accessors.
    */
    class controlAlgorithm
    {
    public:
        //std::array<double,2> x;

        double x[2];

        double thetaVar[3];

        int numberOfThetaLocal;

        theta numberOfThetaType;

        bool sensitivityAnalysis;

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
        double stepTime = 0.01;

        // Simulation Stop
        int finalTime = 1000;
    };

    // Finite Differences Algorithm
    class FD : public controlAlgorithm
    {
    public:
        // Constructor

        FD();

        FD(theta numberOfTheta, bool sensitivity);

        // Access Methods
        
        // Perfomrance Storage - Used for diagrams only
        std::vector<double> performanceValue0;
        std::vector<double> performanceValue1;
        std::vector<double> performanceValue2;
        std::vector<double> performanceValue3;

        // Get slope
        double  getHetta();

        // Get theta change (Delta Theta)
        double  getDtheta();

        // Set slope
        void    setHetta(double setValue);

        // Set theta change (Delta Theta)
        void    setDtheta(double setValue);

        double P = 0;

        void iterationsCalculate();

        int simulationIterations = 0;

        void runAlgorithm();

    protected:
        double performanceThreshold = 1;
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

    class SPSA : public controlAlgorithm
    {
    public:
        void runAlgorithm();
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
    protected:
        // Algorithm Parameters
        double betta = 2.1;
        double gamma = 0.1;
        double alpha = 0.1;
        double A = 0.1;
        double a = 0.1;
        double p = 0.5;
        double ck = 0;
        double ak = 0;
        double Dk2[2] = { 0,0 };
        double Dk3[3] = { 0,0,0 };
    private:
        // Algorithm Implementation
        void spsa();

        // Calculate Performance
        void performance();

        // Sensitivity Analysis Implementation
        void sensitivityAnalyzer();
    };

    class LQR : public controlAlgorithm
    {
    public:
        void runAlgorithm();
        // Access Methods
    protected:
        // Algorithm Parameters
    private:
        // Algorithm Implementation
        void lqr();

        // Calculate Performance
        void cost();
    };

    class AC :public controlAlgorithm
    {
    public:
        void runAlgorithm();
        // Access Methods
        void setGamma(double setValue);
        void setK1(double setValue);
        void setK2(double setValue);

        double getGamma();
        double getk1();
        double getk2();
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

//#endif