#include "displayData.h"

void vdpo::displayData::thetaTypes()
{
	std::cout << "The number of theta parameters can be 2 or 3" << std::endl;
	std::cout << "Example for 2 parameters: " << std::endl;
	std::cout << "vdpo::theta::two " << std::endl;
}

void vdpo::displayData::availableAlgorithms()
{
	std::cout << "Currently available algorithms to control the van der pol system are:" << std::endl;
	std::cout << "FD   - Finite Differences" << std::endl;
	std::cout << "SPSA - Simultaneous Perturbation Stochastic Approximation" << std::endl;
	std::cout << "LQR  - Linear Quadratic Regulator using Riccati ODE" << std::endl;
	std::cout << "AC   - Indirect Adaptive Control" << std::endl;
}

void vdpo::displayData::systemParameters()
{
	std::cout << "The system has 3 constant (over time) which are:" << std::endl;
	std::cout << "m = " << this->m << std::endl;
	std::cout << "c = " << this->c << std::endl;
	std::cout << "k = " << this->k << std::endl;
}

template<typename T>
void vdpo::displayData::printData(T var)
{
	std::cout << var << std::endl;
}

void vdpo::FDParameters::parameters()
{
	std::cout << "The parameters of the FD algorithm are:" << std::endl;
	std::cout << "h " << std::endl;
	std::cout << "dtheta " << std::endl;
}

