#include "controlAlgorithm.h"
#include <array>

// Base class functions

vdpo::controlAlgorithm::controlAlgorithm()
{
    this->selectedAlgorithm = algorithm::FD;
    this->numberOfThetaLocal = 2;
    this->numberOfThetaType = theta::two;
    this->sensitivityAnalysis = false;
    this->thetaVar[0] = 0.0;
    this->thetaVar[1] = 0.0;
    this->thetaVar[2] = 0.0;
    this->x[0] = 0.0;
    this->x[1] = 0.0;
}

vdpo::controlAlgorithm::controlAlgorithm(algorithm selectAlgorithm, theta numberOfTheta, bool sensitivity)
{
    this->selectedAlgorithm = selectAlgorithm;
    this->numberOfThetaLocal = (numberOfTheta == theta::two ? 2 : (numberOfTheta == theta::three ? 3 : 0));
    this->numberOfThetaType = numberOfTheta;
    this->sensitivityAnalysis = sensitivity;
    this->thetaVar[0] = 0.0;
    this->thetaVar[1] = 0.0;
    this->thetaVar[2] = 0.0;
    this->x[0] = 0.0;
    this->x[1] = 0.0;
}

void vdpo::controlAlgorithm::setStartTime(int setValue)     { this->startTime = setValue; }
void vdpo::controlAlgorithm::setStepTime(double setValue)   { this->stepTime = setValue; }
void vdpo::controlAlgorithm::setFinalTime(int setValue)     { this->finalTime = setValue; }
void vdpo::controlAlgorithm::setMaxIterations(int setValue) { this->maxRepeats = setValue; }
void vdpo::controlAlgorithm::setSystemModel(vdpo::systemModel explicitModel)
{
    this->useExplicitModel = true;

    this->localModel.setSysParK(explicitModel.getSysParK());
    this->localModel.setSysParM(explicitModel.getSysParM());
    this->localModel.setSysParC(explicitModel.getSysParC());
    this->localModel.setThetaNumber1(explicitModel.getThetaNumber());
}

int     vdpo::controlAlgorithm::getStartTime()      { return this->startTime; }
double  vdpo::controlAlgorithm::getStepTime()       { return this->stepTime; }
int     vdpo::controlAlgorithm::getFinalTime()      { return this->finalTime; }
int     vdpo::controlAlgorithm::getMaxIterations()  { return this->maxRepeats; }

// FD Constructor

vdpo::FD::FD()
{
    this->numberOfThetaLocal = 2;
    this->numberOfThetaType = theta::two;
    this->sensitivityAnalysis = false;
    this->thetaVar[0] = 0.0;
    this->thetaVar[1] = 0.0;
    this->thetaVar[2] = 0.0;
    this->x[0] = 0.0;
    this->x[1] = 0.0;
}

vdpo::FD::FD(theta numberOfTheta, bool sensitivity)
{
    this->numberOfThetaLocal = (numberOfTheta == theta::two ? 2 : (numberOfTheta == theta::three ? 3 : 0));;
    this->numberOfThetaType = numberOfTheta;
    this->sensitivityAnalysis = sensitivity;
    this->thetaVar[0] = 0.0;
    this->thetaVar[1] = 0.0;
    this->thetaVar[2] = 0.0;
    this->x[0] = 0.0;
    this->x[1] = 0.0;
}

// Access parameters of FD

double vdpo::FD::getHetta()                 { return this->hetta; }
double vdpo::FD::getDtheta()                { return this->dtheta; }
void   vdpo::FD::setHetta(double setValue)  { this->hetta = setValue; }
void   vdpo::FD::setDtheta(double setValue) { this->dtheta = setValue; }

void vdpo::FD::iterationsCalculate()
{
    simulationIterations = ((this->finalTime - this->startTime) / this->stepTime) + 1;
}

void vdpo::FD::runAlgorithm()
{
    if (this->sensitivityAnalysis)
        sensitivityAnalyzer();
    else
        performance();
        //finiteDifferences();
}

void vdpo::FD::finiteDifferences() 
{
    // Local Variables
    double local_performance = 100;

    std::array<double, 3> local_theta;
    local_theta[0] = this->thetaVar[0];
    local_theta[1] = this->thetaVar[1];
    local_theta[2] = (this->numberOfThetaLocal==2)?0: this->thetaVar[2];

    for (int i = 0; i < maxRepeats; i++)
    {
        // Calculate performance
        //performanceValues[0][i] = performance();
        local_theta[0] = this->thetaVar[0] + this->dtheta;

        //performanceValues[1][i] = performance();
        local_theta[0] = this->thetaVar[0];
        local_theta[1] = this->thetaVar[1] + this->dtheta;

        //performanceValues[2][i] = performance();
        local_theta[1] = this->thetaVar[1];

        //thetaVar[0] = thetaVar[0] - hetta * (performanceValues[1][i] - performanceValues[0][i]) / dtheta;
        //thetaVar[1] = thetaVar[1] - hetta * (performanceValues[2][i] - performanceValues[0][i]) / dtheta;

        // Check End Creteria
        //if (std::abs(this->performanceValue) > 10)
            break;
        // Do something when NaN or inf appears
    }
}

void vdpo::FD::performance() 
{
    double norm = 0;
    int i = 0;
    for (double t = this->startTime; t < this->finalTime; t += this->stepTime)
    {
        i++;
        this->localModel.setSSV(this->x);
        this->localModel.setTheta2(this->thetaVar);
        this->localModel.dxCalculate();
        x[0] = x[0] - stepTime * (localModel.getDx()[0]);
        x[1] = x[1] - stepTime * (localModel.getDx()[1]);

        norm = std::sqrt(x[0] * x[0] + x[1] * x[1]);

        this->P += norm;

        // Do something when a NaN or inf appears.
    }
}

void vdpo::FD::sensitivityAnalyzer()
{
    double min = 0;
    double step = 1;
    double max = 10;
    double local_var = this->hetta;
    // Vary hetta only
    for (double i = min; i < max; i+= step)
    { 
        // Step the parameter
        this->hetta = i;
        // Calculate
        this->finiteDifferences();
        // Save results
    }
    this->hetta = local_var;

    local_var = this->dtheta;
    // Vary dtheta only
    for (double i = min; i < max; i += step)
    {
        // Step the parameter
        this->dtheta = i;
        // Calculate
        this->finiteDifferences();
        // Save the results
    }
    this->dtheta = local_var;
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

void vdpo::SPSA::runAlgorithm()
{
    if (this->sensitivityAnalysis)
        sensitivityAnalyzer();
    else
        performance();
        //spsa();
}

void vdpo::SPSA::spsa()
{
    // Add implementation here
}
void vdpo::SPSA::performance()
{
    // Add implementation here
}

void vdpo::SPSA::sensitivityAnalyzer()
{
    //
}

void vdpo::LQR::runAlgorithm()
{
    lqr();
}

void vdpo::LQR::lqr()
{
    // Add implementation here
}
void vdpo::LQR::cost()
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

void vdpo::AC::runAlgorithm()
{
    if (this->sensitivityAnalysis)
        sensitivityAnalyzer();
    else
        value();
        //adaptiveControl();
}

void vdpo::AC::adaptiveControl()
{
    // Add implementation here
}
void vdpo::AC::value()
{
    // Add implementation here
}
void vdpo::AC::sensitivityAnalyzer()
{
    //
}






