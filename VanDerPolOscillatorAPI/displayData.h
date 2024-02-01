#pragma once

//#ifndef DISPLAY_DATA_INCLUDE
//#define DISPLAY_DATA_INCLUDE

namespace vdpo {
	/*
	* Can plot data on gnuplot window and can print on console data
	*/
	class displayData
	{
	public:
		displayData();
		/*
		* Prints data on console.
		*/
		void printData();
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
}

//#endif
