#pragma once

//#ifndef DISPLAY_DATA_INCLUDE
//#define DISPLAY_DATA_INCLUDE

#include <vector> // required for gnuplot's method send
#include "enumerators.h"
#include "systemModel.h"
#include "controlAlgorithm.h"
#include "gnuplot-iostream.h"

namespace vdpo {
	// Can plot data on gnuplot window and can print on console data
	class displayData : protected systemModel, controlAlgorithm
	{
	public:
		std::string graphTitle;
		std::string label1;
		std::string label2;
		std::vector<double> vector1;
		std::vector<double> vector2;

		std::string filename;

		displayData();
		
		displayData(std::string title, std::string labelA, std::string labelB, std::string resultsFileName);

		//Load Data on Vector
		void setVector1(double v);
		void setVector2(double v);
		
		template<int N>
		void setVector1(double v[N]);
		template<int N>
		void setVector2(double v[N]);
		
		void setVector1(std::vector<double> v);
		void setVector2(std::vector<double> v);

		// Prints data on console.
		void thetaTypes();

		// A helper method to display the available algorithms
		void availableAlgorithms();

		// A helper method to display the values of system parameters
		void systemParameters();

		// Prints all data of the vector1
		void consoleWrite1();

		// Prints all data of the vector2
		void consoleWrite2();

		// Plots data on GNU window.
		void plotData1();
		
		// Plots data on GNU window.
		void plotData2();

		// Exports data in a .txt file.
		void exportData();

		//~displayData();
	};

	//-------------------------------------------//
	// About the following code
	// Please fix it or delete it
	// It will be too comblicated if this stays here

	class FDParameters : public displayData
	{
	public:
		void parameters();
	};
	class SPSAParameters : public displayData
	{
		//
		void parameters();
	};
	class LQRParameters : public displayData
	{
		//
		void parameters();
	};
	class ACParameters : public displayData
	{
		//
		void parameters();
	};

}

//#endif
