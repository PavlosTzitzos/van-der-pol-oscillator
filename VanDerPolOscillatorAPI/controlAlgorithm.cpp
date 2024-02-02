#include "controlAlgorithm.h"

// Base class functions

vdpo::controlAlgorithm::controlAlgorithm(algorithm selectAlgorithm = vdpo::algorithm::FD, theta numberOfTheta = vdpo::theta::two, bool sensitivity = false)
{
    selectedAlgorithm = selectAlgorithm;
    numberOfThetaLocal = (numberOfTheta == theta::two ? 2 : (numberOfTheta == theta::three ? 3 : 0));
    numberOfThetaType = numberOfTheta;
    sensitivityAnalysis = sensitivity;
}

void vdpo::controlAlgorithm::setStartTime(int setValue)     { this->startTime = setValue; }
void vdpo::controlAlgorithm::setStepTime(double setValue)   { this->stepTime = setValue; }
void vdpo::controlAlgorithm::setFinalTime(int setValue)     { this->finalTime = setValue; }
void vdpo::controlAlgorithm::setMaxIterations(int setValue) { this->maxRepeats = setValue; }

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
    // Add implementation here
}

void vdpo::FD::performance () 
{ 
    //
    vdpo::systemModel currentModel = vdpo::systemModel::systemModel(1.0,1.1,1.1,theta::two);

    for (double t = startTime; t < finalTime; t += stepTime)
    {
        
        currentModel.setSSV(this->x);
        currentModel.setTheta2(this->thetaVar);
        currentModel.dxCalculate();
        x[0] = x[0] - stepTime * (*currentModel.getDx()); // please fix this line to make it work
    }
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







