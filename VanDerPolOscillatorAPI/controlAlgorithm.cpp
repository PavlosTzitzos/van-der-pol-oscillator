#include "controlAlgorithm.h"

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
void    vdpo::controlAlgorithm::setStartTime(int setValue)     { this->startTime = setValue; }
void    vdpo::controlAlgorithm::setStepTime(double setValue)   { this->stepTime = setValue; }
void    vdpo::controlAlgorithm::setFinalTime(int setValue)     { this->finalTime = setValue; }
void    vdpo::controlAlgorithm::setMaxIterations(int setValue) { this->maxRepeats = setValue; }
void    vdpo::controlAlgorithm::setSystemModel(vdpo::systemModel explicitModel)
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
void    vdpo::controlAlgorithm::iterationsCalculate()
{
    simulationIterations = ((this->finalTime - this->startTime) / this->stepTime) + 1;
}

// FD Class

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
double  vdpo::FD::getHetta()                 { return this->hetta; }
double  vdpo::FD::getDtheta()                { return this->dtheta; }
void    vdpo::FD::setParameters(double parHetta, double parDtheta)
{ 
    this->hetta = parHetta;
    this->dtheta = parDtheta;
}
void    vdpo::FD::setParameters(double parameters[2])
{ 
    this->hetta = parameters[0];
    this->dtheta = parameters[1];
}
void    vdpo::FD::setHetta(double setValue)  { this->hetta = setValue; }
void    vdpo::FD::setDtheta(double setValue) { this->dtheta = setValue; }
void    vdpo::FD::runAlgorithm()
{
    if (this->sensitivityAnalysis)
        this->sensitivityAnalyzer();
    else
        // this->performance(); // for debugging and testing uncomment this line and comment the next
        this->finiteDifferences();
}
void    vdpo::FD::finiteDifferences() 
{
    if (!this->performanceValue0.empty())
        this->performanceValue0.clear();
    if (!this->performanceValue1.empty())
        this->performanceValue1.clear();
    if (!this->performanceValue2.empty())
        this->performanceValue2.clear();
    if (!this->performanceValue3.empty())
        this->performanceValue3.clear();
    if (!this->x1.empty())
        this->x1.clear();
    if (!this->x2.empty())
        this->x2.clear();

    bool plotFlag = false;

    double localTheta[3];

    for (int i = 0; i < this->maxRepeats; i++)
    {
        try
        {
            localTheta[0] = this->thetaVar[0];
            localTheta[1] = this->thetaVar[1];
            localTheta[2] = (this->numberOfThetaLocal == 2) ? 0 : this->thetaVar[2];
            // Calculate performance
            performance();
            this->performanceValue0.push_back(this->P);
            this->thetaVar[0] = localTheta[0] + this->dtheta;

            performance();
            this->performanceValue1.push_back(this->P);
            this->thetaVar[0] = localTheta[0];
            this->thetaVar[1] = localTheta[1] + this->dtheta;

            performance();
            this->performanceValue2.push_back(this->P);
            this->thetaVar[1] = localTheta[1];
            this->thetaVar[2] = localTheta[2] + this->dtheta;

            performance();
            this->performanceValue3.push_back(this->P);
            this->thetaVar[2] = localTheta[2];

            this->thetaVar[0] = this->thetaVar[0] - this->hetta * (this->performanceValue1.back() - this->performanceValue0.back()) / this->dtheta;
            this->thetaVar[1] = this->thetaVar[1] - this->hetta * (this->performanceValue2.back() - this->performanceValue0.back()) / this->dtheta;
            this->thetaVar[2] = this->thetaVar[2] - this->hetta * (this->performanceValue3.back() - this->performanceValue0.back()) / this->dtheta;

            // Do something when NaN or inf appears - Exception and Error handling
            if (isnan(this->performanceValue0.back()) || isnan(this->performanceValue1.back()) || isnan(this->performanceValue2.back()) || isnan(this->performanceValue3.back()))
            {
                // Remove the last element
                this->performanceValue0.pop_back();
                this->performanceValue1.pop_back();
                this->performanceValue2.pop_back();
                this->performanceValue3.pop_back();
                throw std::runtime_error("101 - Calculated Performance is NaN!");
            }
            if (isinf(this->performanceValue0.back()) || isinf(this->performanceValue1.back()) || isinf(this->performanceValue2.back()) || isinf(this->performanceValue3.back()))
            {
                // Remove the last element
                this->performanceValue0.pop_back();
                this->performanceValue1.pop_back();
                this->performanceValue2.pop_back();
                this->performanceValue3.pop_back();
                throw std::runtime_error("102 - Calculated Performance is infinity!");
            }
            // Check End Creteria
            if (this->performanceValue0.back() <= 1.0 )
            {
                // Plot 
                plotFlag = true;
                // Stop Iterations
                break;
            }
            // Plot last x0,x1 (ie at i before break or end for-loop)
            if (i == this->maxRepeats - 1)
                plotFlag = true;
        }
        catch (const std::exception& e)
        {
            std::cout << "Inside Finite Differencies method at iteration: "<< i << " an exception occured. More details are shown below: " << std::endl;
            std::cout << "Exception " << e.what() << std::endl;
            std::cout << "Stoping execution..." << std::endl;
            return;
        }
    }
    if (plotFlag && this->displayGraphs)
    {
        this->plotPerformance.setVector1(this->performanceValue0);
        this->plotPerformance.graphTitle = "Performance over iterations";
        this->plotPerformance.axisYlabel = "Performance";
        this->plotPerformance.axisXlabel = "Iterations";
        this->plotPerformance.label1 = "P";
        this->plotPerformance.plotData1();

        this->plotSSV.setVector1(this->x1);
        this->plotSSV.setVector2(this->x2);
        this->plotSSV.label1 = "x1";
        this->plotSSV.label2 = "x2";
        this->plotSSV.axisYlabel = "State Space Variables";
        this->plotSSV.axisXlabel = "Time";
        this->plotSSV.plotData2();
        this->plotSSV.setPair(true);
        this->plotSSV.graphTitle = "x1 vs x2";
        this->plotSSV.axisYlabel = "x2";
        this->plotSSV.axisXlabel = "x1";
        this->plotSSV.labelPair = "(x1, x2)";
        this->plotSSV.setXRange(-1, 1);
        this->plotSSV.setYRange(-1, 1);
        this->plotSSV.plotPair();
    }
}
void    vdpo::FD::performance() 
{
    double norm = 0;
    int i = 0;
    double localX[2] = { this->x[0],this->x[1] };
    this->x1.clear();
    this->x2.clear();
    this->timeVector.clear();
    this->P = 0.0;

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
            this->timeVector.push_back(t);
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
                throw std::runtime_error("105 - Calculated x is infinity!");
            }
            if (std::abs(localX[0]) < 1e-7 || std::abs(localX[1]) < 1e-7)
            {
                std::cout << "Zero reached successfully at time " << t << " with performance P = " << this->P << std::endl;
                break;
            }
            norm = std::sqrt(localX[0] * localX[0] + localX[1] * localX[1]);
            this->P += norm;
        }
        catch (const std::exception& e)
        {
            std::cout << "Inside Finite Differencies Performance method at iteration: " << i << " an overflow has occured. More details are shown below: " << std::endl;
            std::cout << "Exception " << e.what() << std::endl;
            std::cout << "Stoping execution..." << std::endl;
            return;
        }
    }
}
void    vdpo::FD::sensitivityAnalyzer()
{
    if (!this->parameter0.empty())
        this->parameter0.clear();
    if (!this->parameter1.empty())
        this->parameter1.clear();
    // this might be unnecessary , but for now the values will be stored and then loaded again

    double localVar1 = this->hetta;
    double localVar2 = this->dtheta;

    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution0(0.0001, 0.002);
    // Vary hetta only
    double step = 0.001;
    double min = 0.005;
    double max = 0.015;
    for (double i = min; i < max; i+= step)
    { 
        // Step the parameter
        this->hetta = i;
        // Give random values to all other parameters
        this->dtheta = distribution0(generator);
        // Calculate
        this->finiteDifferences();
        // Save results
        this->parameter0.push_back(this->hetta);
        this->parameter1.push_back(this->dtheta);
    }

    std::uniform_real_distribution<double> distribution1(0.001,0.02);
    // Vary dtheta only
    step = 0.0001;
    min = 0.0005;
    max = 0.0015;
    for (double i = min; i < max; i += step)
    {
        // Step the parameter
        this->dtheta = i;
        // Give random values to all other parameters
        this->hetta = distribution1(generator);
        // Calculate
        this->finiteDifferences();
        // Save results
        this->parameter0.push_back(this->hetta);
        this->parameter1.push_back(this->dtheta);
    }
    this->hetta  = localVar1;
    this->dtheta = localVar2;
}

// SPSA Class

vdpo::SPSA::SPSA()
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
vdpo::SPSA::SPSA(theta numberOfTheta, bool sensitivity)
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
void    vdpo::SPSA::setBetta(double setValue)       { this->betta = setValue; }
void    vdpo::SPSA::setGamma(double setValue)       { this->gamma = setValue; }
void    vdpo::SPSA::setAlpha(double setValue)       { this->alpha = setValue; }
void    vdpo::SPSA::setABig(double setValue)        { this->A = setValue; }
void    vdpo::SPSA::setASmall(double setValue)      { this->a = setValue; }
void    vdpo::SPSA::setProbability(double setValue) { this->p = setValue; }
void    vdpo::SPSA::setParameters(double parameters[6])
{
    this->betta = parameters[0];
    this->gamma = parameters[1];
    this->alpha = parameters[2];
    this->A = parameters[3];
    this->a = parameters[4];
    this->p = parameters[5];
}
double  vdpo::SPSA::getBetta()       { return this->betta; }
double  vdpo::SPSA::getGamma()       { return this->gamma; }
double  vdpo::SPSA::getAlpha()       { return this->alpha; }
double  vdpo::SPSA::getABig()        { return this->A; }
double  vdpo::SPSA::getASmall()      { return this->a; }
double  vdpo::SPSA::getProbability() { return this->p; }
void    vdpo::SPSA::runAlgorithm()
{
    if (this->sensitivityAnalysis)
        sensitivityAnalyzer();
    else
        //performance(); // for debugging and testing
        this->spsa();
}
void    vdpo::SPSA::spsa()
{
    // make sure the vectors are empty
    if (!this->performanceValue0.empty())
        this->performanceValue0.clear();
    if (!this->performanceValue1.empty())
        this->performanceValue1.clear();
    if (!this->performanceValue2.empty())
        this->performanceValue2.clear();
    if (!this->performanceValue3.empty())
        this->performanceValue3.clear();
    if (!this->performanceValue4.empty())
        this->performanceValue4.clear();
    if (!this->performanceValue5.empty())
        this->performanceValue5.clear();

    bool plotFlag = false;

    double localTheta[3];

    for (int i = 0; i < this->maxRepeats; i++)
    {
        try
        {
            localTheta[0] = this->thetaVar[0];
            localTheta[1] = this->thetaVar[1];
            localTheta[2] = (this->numberOfThetaLocal == 2) ? 0 : this->thetaVar[2];

            this->ck = this->betta / powl(i + 1, this->gamma);
            this->ak = this->a / powl(this->A + i + 1, this->alpha);
            std::random_device rd;
            std::mt19937 gen(rd());
            std::bernoulli_distribution d(this->p);
            for (int i = 0; i < 3; i++)
                this->Dk[i] = d(gen) ? 1.0 : -1.0;

            // Calculate performance
            // positive P
            this->thetaVar[0] = localTheta[0] + this->ck * this->Dk[0];
            performance();
            this->performanceValue0.push_back(this->P);
            this->thetaVar[0] = localTheta[0];

            this->thetaVar[1] = localTheta[1] + this->ck * this->Dk[1];
            performance();
            this->performanceValue1.push_back(this->P);
            this->thetaVar[1] = localTheta[1];
            if (this->numberOfThetaType == vdpo::theta::three)
            {
                this->thetaVar[2] = localTheta[2] + this->ck * this->Dk[2];
                performance();
                this->performanceValue2.push_back(this->P);
                this->thetaVar[2] = localTheta[2];
            }
            // negative P
            this->thetaVar[0] = localTheta[0] - this->ck * this->Dk[0];
            performance();
            this->performanceValue3.push_back(this->P);
            this->thetaVar[0] = localTheta[0];

            this->thetaVar[1] = localTheta[1] - this->ck * this->Dk[1];
            performance();
            this->performanceValue4.push_back(this->P);
            this->thetaVar[1] = localTheta[1];

            if (this->numberOfThetaType == vdpo::theta::three)
            {
                this->thetaVar[2] = localTheta[2] - this->ck * this->Dk[2];
                performance();
                this->performanceValue5.push_back(this->P);
                this->thetaVar[2] = localTheta[2];
            }

            this->thetaVar[0] = this->thetaVar[0] - this->ak * (this->performanceValue0.back() - this->performanceValue3.back()) / (2 * this->ck * this->Dk[0]);
            this->thetaVar[1] = this->thetaVar[1] - this->ak * (this->performanceValue1.back() - this->performanceValue4.back()) / (2 * this->ck * this->Dk[1]);
            if (this->numberOfThetaType == vdpo::theta::three)
                this->thetaVar[2] = this->thetaVar[2] - this->ak * (this->performanceValue2.back() - this->performanceValue5.back()) / (2 * this->ck * this->Dk[2]);
            else
                this->thetaVar[2] = 0.0;

            // Do something when NaN or inf appears - Exception and Error handling
            if (this->numberOfThetaType == vdpo::theta::three)
            {
                if (isnan(this->performanceValue0.back()) || isnan(this->performanceValue1.back()) || isnan(this->performanceValue2.back()) || isnan(this->performanceValue3.back()) || isnan(this->performanceValue4.back()) || isnan(this->performanceValue5.back()))
                {
                    // Remove the last element
                    this->performanceValue0.pop_back();
                    this->performanceValue1.pop_back();
                    this->performanceValue2.pop_back();
                    this->performanceValue3.pop_back();
                    this->performanceValue4.pop_back();
                    this->performanceValue5.pop_back();
                    throw std::runtime_error("101 - Calculated Performance is NaN!");
                }
                if (isinf(this->performanceValue0.back()) || isinf(this->performanceValue1.back()) || isinf(this->performanceValue2.back()) || isinf(this->performanceValue3.back()) || isinf(this->performanceValue4.back()) || isinf(this->performanceValue5.back()))
            {
                // Remove the last element
                this->performanceValue0.pop_back();
                this->performanceValue1.pop_back();
                if (this->numberOfThetaType == vdpo::theta::three) this->performanceValue2.pop_back();
                this->performanceValue3.pop_back();
                this->performanceValue4.pop_back();
                if (this->numberOfThetaType == vdpo::theta::three) this->performanceValue5.pop_back();
                throw std::runtime_error("102 - Calculated Performance is infinity!");
            }
            }
            else
            {
                if (isnan(this->performanceValue0.back()) || isnan(this->performanceValue1.back()) || isnan(this->performanceValue3.back()) || isnan(this->performanceValue4.back()) )
                {
                    // Remove the last element
                    this->performanceValue0.pop_back();
                    this->performanceValue1.pop_back();
                    this->performanceValue3.pop_back();
                    this->performanceValue4.pop_back();
                    throw std::runtime_error("103 - Calculated Performance is NaN!");
                }
                if (isinf(this->performanceValue0.back()) || isinf(this->performanceValue1.back()) || isinf(this->performanceValue3.back()) || isinf(this->performanceValue4.back()) )
                {
                    // Remove the last element
                    this->performanceValue0.pop_back();
                    this->performanceValue1.pop_back();
                    this->performanceValue3.pop_back();
                    this->performanceValue4.pop_back();
                    throw std::runtime_error("104 - Calculated Performance is infinity!");
                }
            }
            // Check End Creteria
            if (this->performanceValue0.back() <= 1.0)
            {
                // Plot 
                plotFlag = true;
                // Stop Iterations
                break;
            }
            // Plot last x0,x1 (ie at i before break or end for-loop)
            if (i == this->maxRepeats - 1)
                plotFlag = true;
        }
        catch (const std::exception& e)
        {
            std::cout << "Inside SPSA method at iteration: " << i << " an exception occured. More details are shown below: " << std::endl;
            std::cout << "Exception " << e.what() << std::endl;
            std::cout << "Stoping execution..." << std::endl;
            return;
        }
    }
    if (plotFlag && this->displayGraphs)
    {
        this->plotPerformance.setVector1(this->performanceValue0);
        this->plotPerformance.setVector2(this->performanceValue1);
        this->plotPerformance.setVector3(this->performanceValue3);
        this->plotPerformance.setVector4(this->performanceValue4);
        this->plotPerformance.label1 = "Performance \\theta_{0,+} ";
        this->plotPerformance.label2 = "Performance \\theta_{1,+} ";
        this->plotPerformance.label3 = "Performance \\theta_{0,-} ";
        this->plotPerformance.label4 = "Performance \\theta_{2,-} ";
        this->plotPerformance.graphTitle = "Performance over iterations";
        this->plotPerformance.axisYlabel = "Performance";
        this->plotPerformance.axisXlabel = "Iterations";
        this->plotPerformance.plotData4();

        this->plotSSV.setVector1(this->x1);
        this->plotSSV.setVector2(this->x2);
        this->plotSSV.label1 = "x1";
        this->plotSSV.label2 = "x2";
        this->plotSSV.axisYlabel = "State Space Variables";
        this->plotSSV.axisXlabel = "Iterations";
        this->plotSSV.plotData2();
        this->plotSSV.setPair(true);
        this->plotSSV.graphTitle = "x1 vs x2";
        this->plotSSV.axisYlabel = "x2";
        this->plotSSV.axisXlabel = "x1";
        this->plotSSV.labelPair = "(x1, x2)";
        this->plotSSV.setXRange(-1, 1);
        this->plotSSV.setYRange(-1, 1);
        this->plotSSV.plotPair();
    }
}
void    vdpo::SPSA::performance()
{
    // Add implementation here
    double norm = 0;
    int i = 0;
    double localX[2] = { this->x[0],this->x[1] };
    this->x1.clear();
    this->x2.clear();
    this->timeVector.clear();
    this->P = 0.0;

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
            this->timeVector.push_back(t);
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
            return;
        }
    }
}
void    vdpo::SPSA::sensitivityAnalyzer()
{
    if (!this->parameter0.empty())
        this->parameter0.clear();
    if (!this->parameter1.empty())
        this->parameter1.clear();
    if (!this->parameter2.empty())
        this->parameter2.clear();
    if (!this->parameter3.empty())
        this->parameter3.clear();
    if (!this->parameter4.empty())
        this->parameter4.clear();

    double localVars[5] = { this->betta, this->gamma, this->alpha, this->A, this->a};

    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution0(0.9, 1.1);  // this is for betta and alpha only
    std::uniform_real_distribution<double> distribution1(0.099, 0.11);// this is for gamma, A and a only
    // Vary betta only
    double min = 0.9;
    double step = 0.01;
    double max = 1.1;
    for (double i = min; i < max; i += step)
    {
        // Step the parameter
        this->betta = i;
        // Give random values to all other parameters
        this->gamma = distribution1(generator);
        this->alpha = distribution0(generator);
        this->A = distribution1(generator);
        this->a = distribution1(generator);
        // Calculate
        this->spsa();
        // Save results
        this->parameter0.push_back(this->betta);
        this->parameter1.push_back(this->gamma);
        this->parameter2.push_back(this->alpha);
        this->parameter3.push_back(this->A);
        this->parameter4.push_back(this->a);
    }
    // plot diagram of P for every value
    
    this->parameter0.clear();
    this->parameter1.clear();
    this->parameter2.clear();
    this->parameter3.clear();
    this->parameter4.clear();
    min = 0.05;
    step = 0.01;
    max = 0.15;
    // Vary gamma only
    for (double i = min; i < max; i += step)
    {
        // Step the parameter
        this->gamma = i;
        // Give random values to all other parameters
        this->betta = distribution0(generator);
        this->alpha = distribution0(generator);
        this->A = distribution1(generator);
        this->a = distribution1(generator);
        // Calculate
        this->spsa();
        // Save the results
        this->parameter0.push_back(this->betta);
        this->parameter1.push_back(this->gamma);
        this->parameter2.push_back(this->alpha);
        this->parameter3.push_back(this->A);
        this->parameter4.push_back(this->a);
    }

    this->parameter0.clear();
    this->parameter1.clear();
    this->parameter2.clear();
    this->parameter3.clear();
    this->parameter4.clear();
    min = 0.9;
    step = 0.01;
    max = 1.1;
    // Vary alpha only
    for (double i = min; i < max; i += step)
    {
        // Step the parameter
        this->alpha = i;
        // Give random values to all other parameters
        this->betta = distribution0(generator);
        this->gamma = distribution1(generator);
        this->A = distribution1(generator);
        this->a = distribution1(generator);
        // Calculate
        this->spsa();
        // Save the results
        this->parameter0.push_back(this->betta);
        this->parameter1.push_back(this->gamma);
        this->parameter2.push_back(this->alpha);
        this->parameter3.push_back(this->A);
        this->parameter4.push_back(this->a);
    }

    this->parameter0.clear();
    this->parameter1.clear();
    this->parameter2.clear();
    this->parameter3.clear();
    this->parameter4.clear();
    min = 0.099;
    step = 0.001;
    max = 0.11;
    // Vary A only
    for (double i = min; i < max; i += step)
    {
        // Step the parameter
        this->A = i;
        // Give random values to all other parameters
        this->betta = distribution0(generator);
        this->gamma = distribution1(generator);
        this->alpha = distribution0(generator);
        this->a = distribution1(generator);
        // Calculate
        this->spsa();
        // Save the results
        this->parameter0.push_back(this->betta);
        this->parameter1.push_back(this->gamma);
        this->parameter2.push_back(this->alpha);
        this->parameter3.push_back(this->A);
        this->parameter4.push_back(this->a);
    }

    this->parameter0.clear();
    this->parameter1.clear();
    this->parameter2.clear();
    this->parameter3.clear();
    this->parameter4.clear();
    // Vary a only
    for (double i = min; i < max; i += step)
    {
        // Step the parameter
        this->a = i;
        // Give random values to all other parameters
        this->betta = distribution0(generator);
        this->gamma = distribution1(generator);
        this->alpha = distribution0(generator);
        this->A = distribution1(generator);
        // Calculate
        this->spsa();
        // Save the results
        this->parameter0.push_back(this->betta);
        this->parameter1.push_back(this->gamma);
        this->parameter2.push_back(this->alpha);
        this->parameter3.push_back(this->A);
        this->parameter4.push_back(this->a);
    }

    this->parameter0.clear();
    this->parameter1.clear();
    this->parameter2.clear();
    this->parameter3.clear();
    this->parameter4.clear();
    this->betta = localVars[0];
    this->gamma = localVars[1];
    this->alpha = localVars[2];
    this->A     = localVars[3];
    this->a     = localVars[4];
}

// LQR Class

vdpo::LQR::LQR()
{
    this->sensitivityAnalysis = false;
    this->numberOfThetaType = theta::none;
    this->x[0] = 0.0;
    this->x[1] = 0.0;
    this->K[0] = 0.0;
    this->K[1] = 0.0;
}
vdpo::LQR::LQR(double Q[4], double R, bool sensitivityAnalysis)
{
    this->sensitivityAnalysis = sensitivityAnalysis;
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
double* vdpo::LQR::getQ() { return this->Q; }
double  vdpo::LQR::getR() { return this->R; }
double* vdpo::LQR::getK() { return this->K; }
double  vdpo::LQR::getJ() { return this->J; }
void    vdpo::LQR::setQ(double matrix[4])
{ 
    this->Q[0] = matrix[0];
    this->Q[1] = matrix[1];
    this->Q[2] = matrix[2];
    this->Q[3] = matrix[3];
}
void    vdpo::LQR::setR(double val) { this->R; }
void    vdpo::LQR::setMatrices(double matQ[4], double rValue)
{
    this->Q[0] = matQ[0];
    this->Q[1] = matQ[1];
    this->Q[2] = matQ[2];
    this->Q[3] = matQ[3];
    this->R = rValue;
}
void    vdpo::LQR::runAlgorithm()
{
    if (this->sensitivityAnalysis)
        this->sensitivityAnalyzer();
    else
        //this->cost();// for debugging and testing
        this->riccati();
}
void    vdpo::LQR::riccati()
{
    if (!this->x1.empty())
        this->x1.clear();
    if (!this->x2.empty())
        this->x2.clear();
    if (!this->costValue.empty())
        this->costValue.clear();
    
    bool plotFlag = false;

    for (int i = 0; i < this->maxRepeats; i++)
    {
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

            invMat3 = 1 / (mat3 + this->R);

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
            this->cost();

            // Do something when NaN or inf appears - Exception and Error handling
            if (isnan(this->costValue.back()) )
            {
                // Remove the last element
                this->costValue.pop_back();
                throw std::runtime_error("101 - Calculated Performance is NaN!");
            }
            if (isinf(this->costValue.back()) )
            {
                // Remove the last element
                this->costValue.pop_back();
                throw std::runtime_error("102 - Calculated Performance is infinity!");
            }
            if (this->costValue.back() > 1e7  )
            {
                // Remove the last element
                this->costValue.pop_back();
                throw std::overflow_error("103 - Calculated Performance is too big!");
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
            if (i == this->maxRepeats - 1)
                plotFlag = true;
        }
        catch (const std::overflow_error& e)
        {
            std::cout << "Inside LQR method at iteration: " << i << " an overflow has occured. More details are shown below: " << std::endl;
            std::cout << "Exception " << e.what() << std::endl;
            std::cout << "Stoping execution..." << std::endl;
            return;
        }
    }
    if (plotFlag && this->displayGraphs)
    {
        this->plotPerformance.setVector1(this->costValue);
        this->plotPerformance.setVector2(this->timeVector);
        this->plotPerformance.setPair(false);
        this->plotPerformance.graphTitle = "Cost over time";
        this->plotPerformance.axisYlabel = "Cost";
        this->plotPerformance.axisXlabel = "Time";
        this->plotPerformance.labelPair = "J(t)";
        this->plotPerformance.setXRange(this->timeVector.front()-0.1, this->timeVector.back());
        this->plotPerformance.setYRange(this->costValue.front(), this->costValue.back()+0.1);
        this->plotPerformance.plotPair();

        this->plotSSV.setVector1(this->x1);
        this->plotSSV.setVector2(this->x2);
        this->plotSSV.label1 = "x1";
        this->plotSSV.label2 = "x2";
        this->plotSSV.axisYlabel = "State Space Variables";
        this->plotSSV.axisXlabel = "Iterations";
        this->plotSSV.plotData2();
        this->plotSSV.setPair(true);
        this->plotSSV.graphTitle = "x1 vs x2";
        this->plotSSV.axisYlabel = "x2";
        this->plotSSV.axisXlabel = "x1";
        this->plotSSV.labelPair = "(x1, x2)";
        this->plotSSV.setXRange(-1, 1);
        this->plotSSV.setYRange(-1, 1);
        this->plotSSV.plotPair();
    }
}
void    vdpo::LQR::cost()
{
    double sum = 0;
    int i = 0;
    double localX[2] = {this->x[0],this->x[1]};
    this->x1.clear();
    this->x2.clear();
    this->J = 0.0;
    this->timeVector.clear();
    this->costValue.clear();

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
            this->timeVector.push_back(t);
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
            this->costValue.push_back(this->J);
        }
        catch (const std::overflow_error& e)
        {
            std::cout << "Inside LQR Cost method at iteration: " << i << " an overflow has occured. More details are shown below: " << std::endl;
            std::cout << "Exception " << e.what() << std::endl;
            std::cout << "Stoping execution..." << std::endl;
            return;
        }
    }
    
}
void    vdpo::LQR::sensitivityAnalyzer()
{
    if (!this->parameter0.empty())
        this->parameter0.clear();
    if (!this->parameter1.empty())
        this->parameter1.clear();
    if (!this->parameter2.empty())
        this->parameter2.clear();
    if (!this->parameter3.empty())
        this->parameter3.clear();
    double localVars[5] = { this->Q[0], this->Q[1], this->Q[2], this->Q[3], this->R};
    
    double min = 0.5;
    double step = 0.1;
    double max = 1.5;
    // Vary R only from 0 to 1
    for (double i = min; i < max; i += step)
    {
        // Step the parameter
        this->R = i;
        // Calculate
        this->riccati();
        // Save results
        this->parameter0.push_back(i);
        // plot diagram of P for every value
    }
    this->R = localVars[4];

    
    this->parameter0.clear();
    min = 0.0;
    step = 0.1;
    max = 0.5;
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(-1, 1); //doubles from -1 to 1
    // Vary Q only around their initial values (init +- variation) with random sign
    for (double i = min; i < max; i += step)
    {
        // Step the parameter
        this->Q[0] = (distribution(generator) > 0) ? (localVars[0] + i) : (localVars[0] - i);
        this->Q[1] = (distribution(generator) > 0) ? (localVars[1] + i) : (localVars[1] - i);
        this->Q[2] = (distribution(generator) > 0) ? (localVars[2] + i) : (localVars[2] - i);
        this->Q[3] = (distribution(generator) > 0) ? (localVars[3] + i) : (localVars[3] - i);
        // Calculate
        this->riccati();
        // Save results
        this->parameter0.push_back(this->Q[0]);
        this->parameter1.push_back(this->Q[1]);
        this->parameter2.push_back(this->Q[2]);
        this->parameter3.push_back(this->Q[3]);
        // plot diagram of P for every value
    }
    this->parameter0.clear();
    this->parameter1.clear();
    this->parameter2.clear();
    this->parameter3.clear();
}

// AC Class - NOT WORKING

vdpo::AC::AC()
{
    //
}
vdpo::AC::AC(theta numberOfTheta, bool sensitivity)
{
    //
}
void    vdpo::AC::setGamma(double setValue) { this->gamma = setValue; }
void    vdpo::AC::setK1(double setValue)    { this->k1 = setValue; }
void    vdpo::AC::setK2(double setValue)    { this->k2 = setValue; }
double  vdpo::AC::getGamma() { return this->gamma; }
double  vdpo::AC::getk1()    { return this->k1; }
double  vdpo::AC::getk2()    { return this->k2; }
void    vdpo::AC::runAlgorithm()
{
    if (this->sensitivityAnalysis)
        sensitivityAnalyzer();
    else
        value();
        //adaptiveControl();
}
void    vdpo::AC::adaptiveControl()
{
    // Add implementation here
}
void    vdpo::AC::value()
{
    // Add implementation here
}
void    vdpo::AC::sensitivityAnalyzer()
{
    // Add implementation
}

