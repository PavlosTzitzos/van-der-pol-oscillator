#pragma once

//#ifndef DISPLAY_DATA_INCLUDE
//#define DISPLAY_DATA_INCLUDE

#include "enumerators.h"
#include"systemModel.h"
#include "controlAlgorithm.h"

namespace vdpo {
	/*
	* Can plot data on gnuplot window and can print on console data
	*/
	class displayData : protected systemModel, controlAlgorithm
	{
	public:
		displayData();
		/*
		* Prints data on console.
		*/
		void thetaTypes();

		void availableAlgorithms();

		void systemParameters();

		template<typename T>
		void printData(T var);

		/*
		* Plots data on GNU window.
		*/
		void plotData();
		/*
		* Exports data in a .txt file.
		*/
		void exportData();

		~displayData();
	};

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
