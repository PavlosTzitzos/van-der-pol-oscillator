#pragma once

//#ifndef DISPLAY_DATA_INCLUDE
//#define DISPLAY_DATA_INCLUDE

#include <vector> // required for gnuplot's method send
#include "enumerators.h"
#include "gnuplot-iostream.h"

namespace vdpo {
	// Can plot data on gnuplot window and can print on console data
	class displayData
	{
	public:
		std::string graphTitle;
		std::string label1;
		std::string label2;
		std::string label3;
		std::string label4;
		std::string label5;
		std::string label6;
		std::vector<double> vector1;
		std::vector<double> vector2;
		std::vector<double> vector3;
		std::vector<double> vector4;
		std::vector<double> vector5;
		std::vector<double> vector6;

		std::string filename;

		displayData();
		
		displayData(std::string title, std::string labelA, std::string labelB, std::string resultsFileName);

		//Load Data on Vector
		void setVector1(double v);
		void setVector2(double v);
		void setVector3(double v);
		void setVector4(double v);
		void setVector5(double v);
		void setVector6(double v);

		template<int N>
		void setVector1(double v[N]);
		template<int N>
		void setVector2(double v[N]);
		template<int N>
		void setVector3(double v[N]);
		template<int N>
		void setVector4(double v[N]);
		template<int N>
		void setVector5(double v[N]);
		template<int N>
		void setVector6(double v[N]);

		void setVector1(std::vector<double> v);
		void setVector2(std::vector<double> v);
		void setVector3(std::vector<double> v);
		void setVector4(std::vector<double> v);
		void setVector5(std::vector<double> v);
		void setVector6(std::vector<double> v);

		// Prints data on console.
		void thetaTypes();

		// A helper method to display the available algorithms
		void availableAlgorithms();

		// Prints all data of the vector1
		void consoleWrite1();

		// Prints all data of the vector2
		void consoleWrite2();

		// ToDo: Add consoleWrite3,4,5,6
		// Additionally: figure out a way to write only 1 function instead of 6 !

		// Plots data on GNU window.
		void plotData1();
		
		// Plots data on GNU window.
		void plotData2();

		void plotData3();

		void plotData4();

		void plotData5();

		void plotData6();

		// Exports data in a .txt file.
		void exportData();

		//~displayData();
	};
}

//#endif
