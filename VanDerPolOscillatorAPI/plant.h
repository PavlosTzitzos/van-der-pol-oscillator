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
	protected:
		// Initial value of SSV x1,x2
		double x0[2] = { 1,1 };

		// Initial estimate of theta parameters
		double theta0[3] = { 0,0,0 };

		// System Parameters (check notes for more explaination)

		// k system parameter (k = spring constant or k = 1/capacitance(C) )
		double k = 0;

		// m system parameter (m = mass or k = inductance(L) )
		double m = 0;

		// c system parameter (c = damping factor or c = resistance(R) )
		double c = 0;

		// number of theta
		theta numberOfTheta = theta::two;

		// algorithm select
		algorithm selectedAlgorithm = algorithm::FD;

		// if you want sensitivity analysis
		bool sensitivityAnalysis = false;
	public:
		// Accessors

		// Set initial value of SSVs
		void setInitialX(double setValue[2]);

		// Set initial value of SSVs
		void setInitialTheta2(double setValue[2]);

		// Set initial value of SSVs
		void setInitialTheta3(double setValue[3]);

		// Constructors
		
		// Empty Constructor
		plant();
		
		// Use if you need only sensitivity analysis
		plant(bool sensitivityAnalysis = true);
		
		// For k,m,c seperate variables
		plant(double k, double m, double c, algorithm selectAlgorithm = algorithm::FD, bool sensitivityAnalysis = false);
		
		// For [k,m,c] vector
		plant(double systemParameters[3], algorithm selectAlgorithm = algorithm::FD, bool sensitivityAnalysis = false);

		// Execute-Run
		void simulatePlant();
		
		// Destructor
		~plant();
	};
}

//#endif
