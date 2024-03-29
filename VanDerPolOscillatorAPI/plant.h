// About header guards: https://learn.microsoft.com/en-us/cpp/cpp/header-files-cpp?view=msvc-170#include-guards

#pragma once

#include <iostream>
#include <chrono>
#include <ctime>

#include "enumerators.h"
#include "systemModel.h"
#include "displayData.h"
#include "controlAlgorithm.h"

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
		plant(bool sensitivityAnalysis);
		
		// For k,m,c seperate variables
		plant(double k, double m, double c, algorithm selectAlgorithm, bool sensitivityAnalysis);
		
		// For [k,m,c] vector
		plant(double systemParameters[3], algorithm selectAlgorithm, bool sensitivityAnalysis);

		// Execute-Run
		void simulatePlant();
		
		void help();

		// Destructor
		~plant();
	};
}
