#include "controlAlgorithm.h"
#include <array>

// Base class functions

vdpo::controlAlgorithm::controlAlgorithm()
{
    selectedAlgorithm = algorithm::FD;
    numberOfThetaLocal = 2;
    numberOfThetaType = theta::two;
    sensitivityAnalysis = false;
    thetaVar[0] = 0.0;
    thetaVar[1] = 0.0;
    thetaVar[2] = 0.0;
    x[0] = 0.0;
    x[1] = 0.0;
}

vdpo::controlAlgorithm::controlAlgorithm(algorithm selectAlgorithm, theta numberOfTheta, bool sensitivity)
{
    selectedAlgorithm = selectAlgorithm;
    numberOfThetaLocal = (numberOfTheta == theta::two ? 2 : (numberOfTheta == theta::three ? 3 : 0));
    numberOfThetaType = numberOfTheta;
    sensitivityAnalysis = sensitivity;
    thetaVar[0] = 0.0;
    thetaVar[1] = 0.0;
    thetaVar[2] = 0.0;
    x[0] = 0.0;
    x[1] = 0.0;
}

void vdpo::controlAlgorithm::setStartTime(int setValue)     { this->startTime = setValue; }
void vdpo::controlAlgorithm::setStepTime(double setValue)   { this->stepTime = setValue; }
void vdpo::controlAlgorithm::setFinalTime(int setValue)     { this->finalTime = setValue; }
void vdpo::controlAlgorithm::setMaxIterations(int setValue) { this->maxRepeats = setValue; }
void vdpo::controlAlgorithm::setSystemModel(vdpo::systemModel explicitModel)
{
    useExplicitModel = true;

    localModel.setSysParK(explicitModel.getSysParK());
    localModel.setSysParM(explicitModel.getSysParM());
    localModel.setSysParC(explicitModel.getSysParC());
    localModel.setThetaNumber1(explicitModel.getThetaNumber());
}

int     vdpo::controlAlgorithm::getStartTime()      { return this->startTime; }
double  vdpo::controlAlgorithm::getStepTime()       { return this->stepTime; }
int     vdpo::controlAlgorithm::getFinalTime()      { return this->finalTime; }
int     vdpo::controlAlgorithm::getMaxIterations()  { return this->maxRepeats; }

// Access parameters of FD

double vdpo::FD::getHetta()                 { return hetta; }
double vdpo::FD::getDtheta()                { return dtheta; }
void   vdpo::FD::setHetta(double setValue)  { this->hetta = setValue; }
void   vdpo::FD::setDtheta(double setValue) { this->dtheta = setValue; }

void vdpo::FD::finiteDifferences() 
{
    // Local Variables
    double local_performance = 100;

    std::array<double, 3> local_theta;
    local_theta[0] = thetaVar[0];
    local_theta[1] = thetaVar[1];
    local_theta[2] = (numberOfThetaLocal==2)?0:thetaVar[2];

    for (int i = 0; i < maxRepeats; i++)
    {
        // Calculate performance
        //performanceValues[0][i] = performance();
        local_theta[0] = thetaVar[0] + dtheta;

        //performanceValues[1][i] = performance();
        local_theta[0] = thetaVar[0];
        local_theta[1] = thetaVar[1] + dtheta;

        //performanceValues[2][i] = performance();
        local_theta[1] = thetaVar[1];

        //thetaVar[0] = thetaVar[0] - hetta * (performanceValues[1][i] - performanceValues[0][i]) / dtheta;
        //thetaVar[1] = thetaVar[1] - hetta * (performanceValues[2][i] - performanceValues[0][i]) / dtheta;

        // Check End Creteria
        //if (std::abs(performanceValue) > 10)
            break;
    }
}

void vdpo::FD::performance () 
{
    int i = 0;
    for (double t = startTime; t < finalTime; t += stepTime)
    {
        i++;
        this->localModel.setSSV(this->x);
        this->localModel.setTheta2(this->thetaVar);
        this->localModel.dxCalculate();
        //x[0][i] = x[0][i] - stepTime * (localModel.getDx()); // please fix this line to make it work
        
    }
}

void vdpo::FD::sensitivityAnalyzer()
{
    double min = 0;
    double step = 1;
    double max = 10;
    double ll = hetta;
    // Vary hetta only
    for (double i = min; i < max; i+= step)
    { 
        hetta = i; 
    }
    hetta = ll;
    ll = dtheta;
    // Vary dtheta only
    for (double i = min; i < max; i += step) { dtheta = i; }
    dtheta = ll;
}

// Access parameters of SPSA

void vdpo::SPSA::setBetta(double setValue)       { this->betta = setValue; }
void vdpo::SPSA::setGamma(double setValue)       { this->gamma = setValue; }
void vdpo::SPSA::setAlpha(double setValue)       { this->alpha = setValue; }
void vdpo::SPSA::setABig(double setValue)        { this->A = setValue; }
void vdpo::SPSA::setASmall(double setValue)      { this->a = setValue; }
void vdpo::SPSA::setProbability(double setValue) { this->p = setValue; }

double vdpo::SPSA::getBetta()       { return this->betta; }
double vdpo::SPSA::getGamma()       { return this->gamma; }
double vdpo::SPSA::getAlpha()       { return this->alpha; }
double vdpo::SPSA::getABig()        { return this->A; }
double vdpo::SPSA::getASmall()      { return this->a; }
double vdpo::SPSA::getProbability() { return this->p; }

void spsa()
{
    // Add implementation here
}
void performance()
{
    // Add implementation here
}
void lqr()
{
    // Add implementation here
}
void cost()
{
    // Add implementation here
}

// Adaptive Control

void vdpo::AC::setGamma(double setValue) { this->gamma = setValue; }
void vdpo::AC::setK1(double setValue)    { this->k1 = setValue; }
void vdpo::AC::setK2(double setValue)    { this->k2 = setValue; }

double vdpo::AC::getGamma() { return this->gamma; }
double vdpo::AC::getk1()    { return this->k1; }
double vdpo::AC::getk2()    { return this->k2; }

void adaptiveControl()
{
    // Add implementation here
}
void value()
{
    // Add implementation here
}







