#include "controlAlgorithm.h"
#include <array>
#include <stdexcept>

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
void vdpo::controlAlgorithm::iterationsCalculate()
{
    simulationIterations = ((this->finalTime - this->startTime) / this->stepTime) + 1;
}

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
    bool plotFlag = false;

    std::array<double, 3> localTheta;

    for (int i = 0; i < maxRepeats; i++)
    {
        try
        {
            localTheta[0] = this->thetaVar[0];
            localTheta[1] = this->thetaVar[1];
            localTheta[2] = (this->numberOfThetaLocal == 2) ? 0 : this->thetaVar[2];
            // Calculate performance
            performance();
            performanceValue0.push_back(this->P);
            this->thetaVar[0] = localTheta[0] + this->dtheta;

            performance();
            performanceValue1.push_back(this->P);
            this->thetaVar[0] = localTheta[0];
            this->thetaVar[1] = localTheta[1] + this->dtheta;

            performance();
            performanceValue2.push_back(this->P);
            this->thetaVar[1] = localTheta[1];
            this->thetaVar[2] = localTheta[2] + this->dtheta;

            performance();
            performanceValue2.push_back(this->P);
            this->thetaVar[2] = localTheta[2];

            this->thetaVar[0] = this->thetaVar[0] - this->hetta * (performanceValue1.back() - performanceValue0.back()) / dtheta;
            this->thetaVar[1] = this->thetaVar[1] - this->hetta * (performanceValue2.back() - performanceValue0.back()) / dtheta;
            this->thetaVar[2] = this->thetaVar[2] - this->hetta * (performanceValue3.back() - performanceValue0.back()) / dtheta;

            // Do something when NaN or inf appears - Exception and Error handling
            if (isnan(performanceValue0.back()) || isnan(performanceValue1.back()) || isnan(performanceValue2.back()) || isnan(performanceValue3.back()))
            {
                // Remove the last element
                performanceValue0.pop_back();
                performanceValue1.pop_back();
                performanceValue2.pop_back();
                performanceValue3.pop_back();
                throw std::runtime_error("101 - Calculated Performance is NaN!");
            }
            if (isinf(performanceValue0.back()) || isinf(performanceValue1.back()) || isinf(performanceValue2.back()) || isinf(performanceValue3.back()))
            {
                // Remove the last element
                performanceValue0.pop_back();
                performanceValue1.pop_back();
                performanceValue2.pop_back();
                performanceValue3.pop_back();
                throw std::runtime_error("102 - Calculated Performance is infinity!");
            }
            if (performanceValue0.back() > 1e7 || performanceValue1.back() > 1e7 || performanceValue2.back() > 1e7 || performanceValue3.back() > 1e7)
            {
                // Remove the last element
                performanceValue0.pop_back();
                performanceValue1.pop_back();
                performanceValue2.pop_back();
                performanceValue3.pop_back();
                throw std::overflow_error("105 - Calculated Performance is too big!");
            }
            // Check End Creteria
            if(i > 0)
            {
                if (std::abs(this->performanceValue0[i] - this->performanceValue0[i - 1]) < this->performanceThreshold)
                {
                    // Plot 
                    plotFlag = true;
                    // Stop Iterations
                    break;
                }
            }
            // Plot last x0,x1 (ie at i before break or end for-loop)
            if (i == maxRepeats - 1)
                plotFlag = true;
        }
        catch (const std::exception& e)
        {
            std::cout << "Inside Finite Differencies method at iteration: "<< i << " an exception occured. More details are shown below: " << std::endl;
            std::cout << "Exception " << e.what() << std::endl;
            std::cout << "Stoping execution..." << std::endl;
            exit(100);
        }
    }
    if (plotFlag)
    {
        this->plotSSV.setVector1(performanceValue0);
        this->plotSSV.plotData1();
    }
}
void vdpo::FD::performance() 
{
    double norm = 0;
    int i = 0;
    double localX[2] = { this->x[0],this->x[1] };
    this->x1.push_back(localX[0]);
    this->x2.push_back(localX[1]);
    for (double t = this->startTime; t < this->finalTime; t += this->stepTime)
    {
        try
        {
            i++;
            this->localModel.setSSV(localX);
            this->localModel.setTheta2(this->thetaVar);
            this->localModel.dxCalculate();
            localX[0] = localX[0] - this->stepTime * (this->localModel.getDx()[0]);
            localX[1] = localX[1] - this->stepTime * (this->localModel.getDx()[1]);

            this->x1.push_back(localX[0]);
            this->x2.push_back(localX[1]);
            // Do something when NaN or inf appears - Exception and Error handling
            if (isnan(localX[0]) || isnan(localX[1]))
            {
                throw std::runtime_error("103 - Calculated x is NaN!");
            }
            if (isinf(localX[0]) || isinf(localX[1]))
            {
                throw std::runtime_error("104 - Calculated x is infinity!");
            }
            if (std::abs(localX[0]) > 1e7 || std::abs(localX[1]) > 1e7)
            {
                throw std::runtime_error("104 - Calculated x is infinity!");
            }
            if (std::abs(localX[0]) < 1e-7 || std::abs(localX[1]) < 1e-7)
            {
                std::cout << "Zero reached successfully at time " << t << " with performance P = " << this->P << std::endl;
                break;
            }
            norm = std::sqrt(localX[0] * localX[0] + localX[1] * localX[1]);
            this->P += norm;
        }
        catch (const std::overflow_error& e)
        {
            std::cout << "Inside Finite Differencies Performance method at iteration: " << i << " an overflow has occured. More details are shown below: " << std::endl;
            std::cout << "Exception " << e.what() << std::endl;
            std::cout << "Stoping execution..." << std::endl;
            exit(100);
        }
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

// SPSA Constructors

vdpo::SPSA::SPSA()
{
    //
}
vdpo::SPSA::SPSA(theta numberOfTheta, bool sensitivity)
{
    //
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

// LQR

vdpo::LQR::LQR()
{
    //
    this->numberOfThetaType = theta::none;
    this->x[0] = 0.0;
    this->x[1] = 0.0;
    this->K[0] = 0.0;
    this->K[1] = 0.0;
}
vdpo::LQR::LQR(double Q[4], double R)
{
    //
    this->numberOfThetaType = theta::none;
    this->x[0] = 0.0;
    this->x[1] = 0.0;
    this->Q[0] = Q[0];
    this->Q[1] = Q[1];
    this->Q[2] = Q[2];
    this->Q[3] = Q[3];
    this->r = R;
    this->K[0] = 0.0;
    this->K[1] = 0.0;
}
void vdpo::LQR::runAlgorithm()
{
    riccati();
}
void vdpo::LQR::riccati()
{
    bool plotFlag = false;

    for (int i = 0; i < maxRepeats; i++)
    {
        //
        try
        {
            // Solve the DARE equation
            double invR = 1 / this->r;

            double tempP[4];
            double matP[4];
            tempP[0] = this->Q[0];
            tempP[1] = this->Q[1];
            tempP[2] = this->Q[2];
            tempP[3] = this->Q[3];

            double matA[4] = {0, 1, 0, 0};

            double matB[2] = {0 , 1};

            double transB[2] = {0, 1};

            // tempP = riccati2(matA, matB, matQ, tempP, matR);
            
            double transA[4];
            transA[0] = matA[0];
            transA[1] = matA[2];
            transA[2] = matA[1];
            transA[3] = matA[3];

            double temp[4];
            double mat1[4];
            double mat2[2];
            double mat3;
            double invMat3;
            double mat4[2];
            double mat5[4];

            temp[0] = transA[0] * tempP[0] + transA[1] * tempP[1];
            temp[1] = transA[0] * tempP[2] + transA[1] * tempP[3];
            temp[2] = transA[2] * tempP[0] + transA[3] * tempP[1];
            temp[3] = transA[2] * tempP[2] + transA[3] * tempP[3];

            mat1[0] = temp[0] * matA[0] + temp[1] * matA[1];
            mat1[1] = temp[0] * matA[2] + temp[1] * matA[3];
            mat1[2] = temp[2] * matA[0] + temp[3] * matA[1];
            mat1[3] = temp[2] * matA[2] + temp[3] * matA[3];

            mat2[0] = temp[0] * matB[0] + temp[1] * matB[1];
            mat2[1] = temp[2] * matB[0] + temp[3] * matB[1];

            mat3 = (transB[0] * tempP[0] + transB[1] * tempP[2]) * matB[0] + (transB[0] * tempP[1] + transB[1] * tempP[3]) * matB[1];

            mat4[0] = (transB[0] * tempP[0] + transB[1] * tempP[2]) * matA[0] + (transB[0] * tempP[1] + transB[1] * tempP[3]) * matA[1];
            mat4[1] = (transB[0] * tempP[0] + transB[1] * tempP[2]) * matA[2] + (transB[0] * tempP[1] + transB[1] * tempP[3]) * matA[3];

            invMat3 = 1 / (mat3 + R);

            mat5[0] = mat2[0] * invMat3 * mat4[0];
            mat5[1] = mat2[0] * invMat3 * mat4[1];
            mat5[2] = mat2[1] * invMat3 * mat4[0];
            mat5[3] = mat2[1] * invMat3 * mat4[1];

            // Solution P of DARE equation

            matP[0] = this->Q[0] + mat1[0] - mat5[0];
            matP[1] = this->Q[1] + mat1[1] - mat5[1];
            matP[2] = this->Q[2] + mat1[2] - mat5[2];
            matP[3] = this->Q[3] + mat1[3] - mat5[3];

            // Calculate K matrix
            
            this->K[0] = invR * (transB[0] * matP[0] + transB[1] * matP[2]);
            this->K[1] = invR * (transB[0] * matP[1] + transB[1] * matP[3]);

            // Calculate Cost (Performance)
            cost();

            // Do something when NaN or inf appears - Exception and Error handling
            if (isnan(costValue.back()) )
            {
                // Remove the last element
                costValue.pop_back();
                throw std::runtime_error("101 - Calculated Performance is NaN!");
            }
            if (isinf(costValue.back()) )
            {
                // Remove the last element
                costValue.pop_back();
                throw std::runtime_error("102 - Calculated Performance is infinity!");
            }
            if (costValue.back() > 1e7  )
            {
                // Remove the last element
                costValue.pop_back();
                throw std::overflow_error("105 - Calculated Performance is too big!");
            }
            // Check End Creteria
            if (i > 0)
            {
                if (std::abs(this->costValue[i] - this->costValue[i - 1]) < this->costThreshold)
                {
                    // Plot 
                    plotFlag = true;
                    // Stop Iterations
                    break;
                }
            }
            // Plot last x0,x1 (ie at i before break or end for-loop)
            if (i == maxRepeats - 1)
                plotFlag = true;
        }
        catch (const std::overflow_error& e)
        {
            std::cout << "Inside LQR Cost method at iteration: " << i << " an overflow has occured. More details are shown below: " << std::endl;
            std::cout << "Exception " << e.what() << std::endl;
            std::cout << "Stoping execution..." << std::endl;
            exit(100);
        }
    }
    if (plotFlag)
    {
        this->plotPerformance.setVector1(costValue);
        this->plotPerformance.plotData1();

        this->plotSSV.setVector1(x1);
        this->plotSSV.setVector2(x2);
        this->plotSSV.plotData2();
    }
}
void vdpo::LQR::performance()
{
    double norm = 0;
    int i = 0;
    double localX[2] = { this->x[0],this->x[1] };
    this->x1.push_back(localX[0]);
    this->x2.push_back(localX[1]);
    for (double t = this->startTime; t < this->finalTime; t += this->stepTime)
    {
        try
        {
            i++;
            this->localModel.setSSV(localX);
            this->localModel.setTheta2(this->K);
            this->localModel.dxCalculate();
            localX[0] = localX[0] - this->stepTime * (this->localModel.getDx()[0]);
            localX[1] = localX[1] - this->stepTime * (this->localModel.getDx()[1]);

            this->x1.push_back(localX[0]);
            this->x2.push_back(localX[1]);

            // Do something when NaN or inf appears - Exception and Error handling
            if (isnan(localX[0]) || isnan(localX[1]))
            {
                throw std::runtime_error("103 - Calculated x is NaN!");
            }
            if (isinf(localX[0]) || isinf(localX[1]))
            {
                throw std::runtime_error("104 - Calculated x is infinity!");
            }
            if (std::abs(localX[0]) > 1e7 || std::abs(localX[1]) > 1e7)
            {
                throw std::runtime_error("104 - Calculated x is infinity!");
            }

            if (std::abs(localX[0]) < 1e-7 || std::abs(localX[1]) < 1e-7)
            {
                std::cout << "Zero reached successfully at time " << t << " with performance P = " << this->P << std::endl;
                break;
            }
            norm = std::sqrt(localX[0] * localX[0] + localX[1] * localX[1]);
            this->P += norm;
        }
        catch (const std::overflow_error& e)
        {
            std::cout << "Inside Finite Differencies Performance method at iteration: " << i << " an overflow has occured. More details are shown below: " << std::endl;
            std::cout << "Exception " << e.what() << std::endl;
            std::cout << "Stoping execution..." << std::endl;
            exit(100);
        }
    }
}
void vdpo::LQR::cost()
{
    double sum = 0;
    int i = 0;
    double localX[2] = {this->x[0],this->x[1]};
    this->x1.push_back(localX[0]);
    this->x2.push_back(localX[1]);
    for (double t = this->startTime; t < this->finalTime; t += this->stepTime)
    {
        try
        {
            i++;
            this->localModel.setSSV(localX);
            this->localModel.setTheta2(this->K);
            this->localModel.dxCalculate();

            localX[0] = localX[0] - this->stepTime * (this->localModel.getDx()[0]);
            localX[1] = localX[1] - this->stepTime * (this->localModel.getDx()[1]);

            this->x1.push_back(localX[0]);
            this->x2.push_back(localX[1]);

            // Do something when NaN or inf appears - Exception and Error handling
            if (isnan(localX[0]) || isnan(localX[1]))
            {
                throw std::runtime_error("103 - Calculated x is NaN!");
            }
            if (isinf(localX[0]) || isinf(localX[1]))
            {
                throw std::runtime_error("104 - Calculated x is infinity!");
            }
            if (std::abs(localX[0]) > 1e7 || std::abs(localX[1]) > 1e7)
            {
                throw std::runtime_error("104 - Calculated x is infinity!");
            }

            if (std::abs(localX[0]) < 1e-7 || std::abs(localX[1]) < 1e-7)
            {
                std::cout << "Zero reached successfully at time " << t << " with cost J = " << this->J << std::endl;
                break;
            }
            
            sum = localX[0] * localX[0] * this->Q[0]
                + localX[0] * localX[1] * this->Q[2]
                + localX[0] * localX[1] * this->Q[1]
                + localX[1] * localX[1] * this->Q[3];

            this->J += sum;
        }
        catch (const std::overflow_error& e)
        {
            std::cout << "Inside LQR Cost method at iteration: " << i << " an overflow has occured. More details are shown below: " << std::endl;
            std::cout << "Exception " << e.what() << std::endl;
            std::cout << "Stoping execution..." << std::endl;
            exit(100);
        }
    }
    this->costValue.push_back(this->J);
}

// Adaptive Control

vdpo::AC::AC()
{
    //
}
vdpo::AC::AC(theta numberOfTheta, bool sensitivity)
{
    //
}

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






