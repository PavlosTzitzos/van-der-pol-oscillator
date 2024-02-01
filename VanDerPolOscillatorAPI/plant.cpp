#include "plant.h"

vdpo::plant::plant()
{
	x0[0] = 1;
	x0[1] = 1;
	k = 1;
	m = 1;
	c = 1;
	numOfTheta = theta::two;
	selAlgo = algorithm::FD;
	sensAnal = false;
}

// Execute-Run
void vdpo::plant::simulatePlant()
{
	std::cout << "\n Simulation Started ... \n" << std::endl;
	vdpo::systemModel MySystem = vdpo::systemModel::systemModel(1.0,1.0,1.0,vdpo::theta::two);
	vdpo::controlAlgorithm MyAlgorithm = vdpo::controlAlgorithm::controlAlgorithm(selAlgo, numOfTheta);
	std::cout << "\n Simulation Finished ... \n" << std::endl;
}

vdpo::plant::~plant() { }
