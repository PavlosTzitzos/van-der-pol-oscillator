
// About header guards: https://learn.microsoft.com/en-us/cpp/cpp/header-files-cpp?view=msvc-170#include-guards
#pragma once

//#ifndef SYSTEM_MODEL_INCLUDE
//#define SYSTEM_MODEL_INCLUDE

#include "enumerators.cpp"

namespace vdpo {
	/*
	* The Van der Pol System model
	* 
	*/
	class systemModel
	{
	protected:
		// System Parameters
		double k = 1; // System Parameter k
		double m = 1; // System Parameter m
		double c = 1; // System Parameter c
		// State Space Variables
		double x[2] = { 0,0 };
		// Derivative of State Space Variables
		double dx[2] = { 0,0 };
		// Model Parameters
		double thetaVar[3]; // Parameters
		// Theta Parameters Number
		theta numTheta = theta::two;
	public:
		// Gets system parameter k value
		double getSysParK();
		// Gets system parameter m value
		double getSysParM();
		// Gets system parameter c value
		double getSysParC();
		// Get state space variables x values
		double* getSSV();
		// Get theta parameters values (2 theta)
		double* getTheta2();
		// Get theta parameters values (3 theta)
		double* getTheta3();
		theta getThetaNumber();

		// Set state space variables x values
		void setSSV(double setValues[2]);
		// Set theta parameters values (2 theta)
		void setTheta2(double setValues[2]);
		// Set theta parameters values (3 theta)
		void setTheta3(double setValues[3]);
				
		// Constructor with k,m,c system parameters as seperate variables
		systemModel(double kk = 1, double mm = 1, double cc = 1, theta numOfTheta = theta::two);
		
		// Constructor with [k,m,c] system parameters as array
		systemModel(double systemParameters[3], theta numOfTheta);

		// Calculate dx
		void dxCalculate();

		virtual ~systemModel();
	};

	// Control Signal u
	class u : public systemModel
	{
	public:
		u(theta num = theta::two);

		// Control Signal using 2 theta parameters
		double u2();
		// Control Signal using 3 theta parameters
		double u3();
		~u();
	};
}
//#endif
