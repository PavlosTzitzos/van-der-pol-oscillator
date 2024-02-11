
// About header guards: https://learn.microsoft.com/en-us/cpp/cpp/header-files-cpp?view=msvc-170#include-guards
#pragma once

//#ifndef SYSTEM_MODEL_INCLUDE
//#define SYSTEM_MODEL_INCLUDE

#include "enumerators.cpp"

namespace vdpo {
	
	// The Van der Pol System model
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
		double thetaVar[3] = { 0,0,0 }; // Parameters

		// Theta Parameters Number
		theta numTheta = theta::two;

	public:

		double uTest = 0; // for debugging

		// Accessors

		// Gets dx data
		double* getDx(); // ???? please FIX this to return either the array or a pointer

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

		// Get theta number - enumerator type
		theta getThetaNumber();

		// Sets system parameter k value
		void setSysParK(double setValue);

		// Sets system parameter m value
		void setSysParM(double setValue);

		// Sets system parameter c value
		void setSysParC(double setValue);

		// Sets system parameters k,m,c values
		void setSysParKMC(double setValues[3]);

		// Set state space variables x values
		void setSSV(double setValues[2]);

		// Set theta parameters values (2 theta)
		void setTheta2(double setValues[2]);

		// Set theta parameters values (3 theta)
		void setTheta3(double setValues[3]);
		
		// Set theta value using enumerator
		void setThetaNumber1(theta setValue);

		// Set theta value using number (2 or 3)
		void setThetaNumber2(int setValue);

		// Default Constructor
		systemModel();

		// Parametrized Constructor with k,m,c system parameters as seperate variables
		systemModel(double kk, double mm, double cc, theta numOfTheta);
		
		// Parametrized Constructor with [k,m,c] system parameters as array
		systemModel(double systemParameters[3], theta numOfTheta);

		// Control Signal using 2 theta parameters
		double u2();

		// Control Signal using 3 theta parameters
		double u3();

		// Calculate dx
		void dxCalculate();

		virtual ~systemModel();
	};
}
//#endif
