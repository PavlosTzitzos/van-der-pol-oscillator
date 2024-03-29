#include "plant.h"

// Constructors

vdpo::plant::plant()
{
	this->x0[0] = 1;
	this->x0[1] = 1;
	this->k = 1;
	this->m = 1;
	this->c = 1;
	this->numberOfTheta = theta::two;
	this->selectedAlgorithm = algorithm::FD;
	this->sensitivityAnalysis = false;
}
vdpo::plant::plant(bool sensitivityAnalysis)
{
	this->x0[0] = 1;
	this->x0[1] = 1;
	this->k = 1;
	this->m = 1;
	this->c = 1;
	this->numberOfTheta = theta::two;
	this->selectedAlgorithm = algorithm::FD;
	this->sensitivityAnalysis = sensitivityAnalysis;
}
vdpo::plant::plant(double kValue, double mValue, double cValue, algorithm selectAlgorithm, bool sensitivityAnalysis)
{
	this->x0[0] = 1;
	this->x0[1] = 1;
	this->k = kValue;
	this->m = mValue;
	this->c = cValue;
	this->numberOfTheta = theta::two;
	this->selectedAlgorithm = selectAlgorithm;
	this->sensitivityAnalysis = sensitivityAnalysis;
}
vdpo::plant::plant(double systemParameters[3], algorithm selectAlgorithm, bool sensitivityAnalysis)
{
	this->x0[0] = 1;
	this->x0[1] = 1;
	this->k = systemParameters[0];
	this->m = systemParameters[1];
	this->c = systemParameters[2];
	this->numberOfTheta = theta::two;
	this->selectedAlgorithm = selectAlgorithm;
	this->sensitivityAnalysis = sensitivityAnalysis;
}

// Accessors

void vdpo::plant::setInitialX(double setValue[2])
{
	this->x0[0] = setValue[0];
	this->x0[1] = setValue[1];
}
void vdpo::plant::setInitialTheta2(double setValue[2])
{
	this->theta0[0] = setValue[0];
	this->theta0[1] = setValue[1];
	this->theta0[2] = 0;
	this->numberOfTheta = theta::two;
}
void vdpo::plant::setInitialTheta3(double setValue[3])
{
	this->theta0[0] = setValue[0];
	this->theta0[1] = setValue[1];
	this->theta0[2] = setValue[2];
	this->numberOfTheta = theta::three;
}

// Execute-Run

void vdpo::plant::simulatePlant()
{
	std::chrono::high_resolution_clock::time_point t_start = std::chrono::high_resolution_clock::now();
	std::cout << "\n Simulation Started ... \n" << std::endl;
	vdpo::systemModel MySystem = vdpo::systemModel::systemModel(this->k, this->m, this->c, this->numberOfTheta);
	MySystem.setSSV(this->x0);
	if(this->selectedAlgorithm == algorithm::FD)
	{
		vdpo::FD MyAlgorithm = vdpo::FD::FD(this->numberOfTheta,this->sensitivityAnalysis);
		MyAlgorithm.x[0] = this->x0[0];
		MyAlgorithm.x[1] = this->x0[1];
		MyAlgorithm.thetaVar[0] = this->theta0[0];
		MyAlgorithm.thetaVar[1] = this->theta0[1];
		MyAlgorithm.thetaVar[2] = this->theta0[2];
		MyAlgorithm.setSystemModel(MySystem);
		MyAlgorithm.runAlgorithm();
	}
	else if (this->selectedAlgorithm == algorithm::SPSA)
	{
		vdpo::SPSA MyAlgorithm = vdpo::SPSA::SPSA(this->numberOfTheta, this->sensitivityAnalysis);
		MyAlgorithm.x[0] = this->x0[0];
		MyAlgorithm.x[1] = this->x0[1];
		MyAlgorithm.thetaVar[0] = this->theta0[0];
		MyAlgorithm.thetaVar[1] = this->theta0[1];
		MyAlgorithm.thetaVar[2] = this->theta0[2];
		MyAlgorithm.setSystemModel(MySystem);
		MyAlgorithm.runAlgorithm();
	}
	else if (this->selectedAlgorithm == algorithm::LQR)
	{
		vdpo::LQR MyAlgorithm = vdpo::LQR::LQR();
		MyAlgorithm.x[0] = this->x0[0];
		MyAlgorithm.x[1] = this->x0[1];
		MyAlgorithm.setSystemModel(MySystem);
		MyAlgorithm.runAlgorithm();
	}
	else if (this->selectedAlgorithm == algorithm::AC)
	{
		vdpo::AC MyAlgorithm = vdpo::AC::AC(this->numberOfTheta, this->sensitivityAnalysis);
		MyAlgorithm.x[0] = this->x0[0];
		MyAlgorithm.x[1] = this->x0[1];
		MyAlgorithm.thetaVar[0] = this->theta0[0];
		MyAlgorithm.thetaVar[1] = this->theta0[1];
		MyAlgorithm.thetaVar[2] = this->theta0[2];
		MyAlgorithm.setSystemModel(MySystem);
		MyAlgorithm.runAlgorithm();
	}
	else
	{
		// Default FD
		std::cout << "... Default simulation ..." << std::endl;
		vdpo::FD MyAlgorithm = vdpo::FD::FD(this->numberOfTheta, this->sensitivityAnalysis);
		MyAlgorithm.x[0] = this->x0[0];
		MyAlgorithm.x[1] = this->x0[1];
		MyAlgorithm.thetaVar[0] = this->theta0[0];
		MyAlgorithm.thetaVar[1] = this->theta0[1];
		MyAlgorithm.thetaVar[2] = this->theta0[2];
		MyAlgorithm.setSystemModel(MySystem);
		MyAlgorithm.runAlgorithm();
	}
	std::chrono::high_resolution_clock::time_point t_end = std::chrono::high_resolution_clock::now();
	std::cout << std::fixed << std::setprecision(2) << "\n Simulation Finished after " << std::chrono::duration<double, std::milli>(t_end - t_start).count() << " s. Press Enter to exit...";
	std::cin.get();
}

vdpo::plant::~plant() { }
