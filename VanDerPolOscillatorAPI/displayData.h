#pragma once

//#ifndef DISPLAY_DATA_INCLUDE
//#define DISPLAY_DATA_INCLUDE

#include <algorithm>
#include <map>
#include <cmath>
#include <array>
#include <vector> // required for gnuplot's method send
#include "enumerators.h"
#include "gnuplot-iostream.h"

namespace vdpo {
	// Can plot data on gnuplot window and can print on console data
	class displayData
	{
	private:
		// Finds the minimum and maximum values in the vectorId for x
		// 0 - none
		// 1 - vector1
		// ...
		// 6 - vector6
		void calcXRange(int id);

		// Finds the minimum and maximum values in the vectorId for y
		// 0 - none
		// 1 - vector1
		// ...
		// 6 - vector6
		void calcYRange(int id);
	public:
		bool xRangeAuto = true;
		bool yRangeAuto = true;

		double xRange[2] = { -1.0,1.0 };
		double yRange[2] = { -1.0,1.0 };

		std::string graphTitle;
		std::string axisXlabel;
		std::string axisYlabel;
		std::string label1;
		std::string label2;
		std::string label3;
		std::string label4;
		std::string label5;
		std::string label6;
		std::string labelPair;
		std::vector<double> vector1;
		std::vector<double> vector2;
		std::vector<double> vector3;
		std::vector<double> vector4;
		std::vector<double> vector5;
		std::vector<double> vector6;

		std::vector<std::pair<double, double> > xyPair;

		std::string filename;

		displayData();
		
		displayData(std::string title, std::string labelA, std::string labelB, std::string resultsFileName);

		// Set range of x : min <= x <= max
		void setXRange(double min, double max);
		
		// Set range of y : min <= y <= max
		void setYRange(double min, double max);

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

		// will plot y vs x vectors 2 vs 1
		// If the vectors are loaded as vector1 as x
		// and vector2 as y then set orderxy true
		// If the vectors are loaded as vector1 as y and 
		// vector2 as x then set orderxy false 
		void setPair(bool orderxy);

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

		void plotPair();

		// Exports data in a .txt file.
		void exportData();

		//~displayData();
	};
}

//#endif
