
// About header guards: https://learn.microsoft.com/en-us/cpp/cpp/header-files-cpp?view=msvc-170#include-guards
#pragma once

//#ifndef CONTROL_ALGORITHM_INCLUDE
//#define CONTROL_ALGORITHM_INCLUDE

#include "enumerators.h"

namespace vdpo {

    /*
    * The available Algorithms to control the system
    */
    class controlAlgorithm
    {
    public:
        
        vdpo::algorithm selectedAlgorithm;
        int numberOfThetaLocal;
        bool sensitivityAnalysis;
        // Constructor
        
        controlAlgorithm(algorithm selectAlgorithm = vdpo::algorithm::FD, theta numberOfTheta = vdpo::theta::two, bool sensitivity = false);

        void setStartTime(int setValue);
        void setStepTime(double setValue);
        void setFinalTime(int setValue);
        void setMaxIterations(int setValue);

        int     getStartTime();
        double  getStepTime();
        int     getFinalTime();
        int     getMaxIterations();
    private:
        int maxRepeats = 100;
        int startTime = 0;
        double stepTime = 0.01;
        int finalTime = 1000;
    };

    class FD :public controlAlgorithm
    {
    public:
        // Access Methods
        double  getHetta();
        double  getDtheta();
        void    setHetta(double setValue);
        void    setDtheta(double setValue);
    protected:
        // Algorithm Parameters
        double hetta = 0.01;
        double dtheta = 0.001;
    private:
        // Algorithm Implementation
        template<int thPar>
        void finiteDifferences();
        // Calculate Performance
        void performance();
    };

    class SPSA :public controlAlgorithm
    {
    public:
        // Access Methods
        void setBetta(double setValue);
        void setGamma(double setValue);
        void setAlpha(double setValue);
        void setABig(double setValue);
        void setASmall(double setValue);
        void setProbability(double setValue);

        double getBetta();
        double getGamma();
        double getAlpha();
        double getABig();
        double getASmall();
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
        template<int thPar>
        void spsa();
        // Calculate Performance
        void performance();
    };

    class LQR :public controlAlgorithm
    {
    public:
        // Access Methods
    protected:
        // Algorithm Parameters
    private:
        // Algorithm Implementation
        template<int thPar>
        void lqr();
        // Calculate Performance
        void cost();
    };

    class AC :public controlAlgorithm
    {
    public:
        // Access Methods
        void setGamma(double setValue);
        void setk1(double setValue);
        void setk2(double setValue);

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
        template<int thPar>
        void adaptiveControl();
        // Calculate Performance
        void value();
    };
}

//#endif