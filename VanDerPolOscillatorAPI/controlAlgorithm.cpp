#include "controlAlgorithm.h"


vdpo::controlAlgorithm::controlAlgorithm(algorithm selectAlgorithm = vdpo::algorithm::FD, theta numberOfTheta = vdpo::theta::two, bool sensitivity = false)
{
    selectedAlgorithm = selectAlgorithm;
    numberOfThetaLocal = (numberOfTheta == theta::two ? 2 : (numberOfTheta == theta::three ? 3 : 0));
    sensitivityAnalysis = sensitivity;
}

// Time Parameters
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

void vdpo::FD::finiteDifferences() { }

void performance () { }

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

// SPSA Algorithm Implementation
template<int thPar>
void spsa(){}
// Calculate Performance
void performance(){}

// LQR Algorithm Implementation
template<int thPar>
void lqr(){}
// Calculate Performance
void cost(){}

// Adaptive Control
// Access Methods
void vdpo::AC::setGamma(double setValue) { this->gamma = setValue; }
void vdpo::AC::setk1(double setValue)    { this->k1 = setValue; }
void vdpo::AC::setk2(double setValue)    { this->k2 = setValue; }

double vdpo::AC::getGamma() { return this->gamma; }
double vdpo::AC::getk1()    { return this->k1; }
double vdpo::AC::getk2()    { return this->k2; }

// AC Algorithm Implementation
template<int thPar>
void adaptiveControl(){}
// Calculate Performance
void value(){}







