#pragma once

//#ifndef PLANT_INCLUDE
//#define PLANT_INCLUDE

#include <iostream>
//#include "gnuplot-iostream.h"

//#include "bigNumberHandler.h"
#include "enumerators.h"
#include "systemModel.h"
//#include "displayData.h"
#include "controlAlgorithm.h"
//#include "sensitivityAnalyzer.h"

namespace vdpo {
	/*
	* Top Model for the system and the controller.
	*/
	class plant
	{
	public:
		// x0, theta0
		double x0[2] = { 0,0 };
		// k,m,c
		double k = 0;
		double m = 0;
		double c = 0;
		// number of theta
		theta numOfTheta = theta::two;
		// algorithm select
		algorithm selAlgo = algorithm::FD;
		// if you want sensitivity analysis
		bool sensAnal = false;

		// Constructor
		plant();
		plant(bool sensitivityAnalysis = true);
		plant(double x0[2], double theta0[2], double k, double m, double c, algorithm selectAlgorithm = algorithm::FD, bool sensitivityAnalysis = false);
		plant(double x0[2], double theta0[3], double k, double m, double c, algorithm selectAlgorithm = algorithm::FD, bool sensitivityAnalysis = false);
		plant(double x0[2], double theta0[2], double systemParameters[3], algorithm selectAlgorithm = algorithm::FD, bool sensitivityAnalysis = false);
		plant(double x0[2], double theta0[3], double systemParameters[3], algorithm selectAlgorithm = algorithm::FD, bool sensitivityAnalysis = false);

		// Execute-Run
		void simulatePlant();
		
		// Destructor
		~plant();
	};
}

//#endif
