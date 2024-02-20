#pragma once

#include <iostream>
#include <vector>
#include <limits>

namespace vdpo{
	/*
	* A handler of Infinite , NaN and other formats bigger than the double type can handle
	* 
	* The idea is to break the original into 2 variables and handle both like one variable.
	* 
	* Only Decimals and only +,-,*,/
	*/
	class bigNum
	{
	public:
		double bigNumber;

		std::vector<int> UpperHalf;

		std::vector<int> LowHalf;

		int length;

		bigNum();

		bigNum(double UnhandledbigNumber);
		
		bigNum(std::vector<int> UnhandledbigNumber);
		
		~bigNum();

		// Operations

		friend bigNum operator + (bigNum& a, bigNum& b);
		friend bigNum operator - (bigNum& a, bigNum& b);
		friend bigNum operator * (bigNum& a, bigNum& b);
		friend bigNum operator / (bigNum& a, bigNum& b);

	private:
		void breakNumber();

		/*
		* When data give inf value
		*/
		void infiniteHandler();

		/*
		* When data give NaN value
		*/
		void nanHandler();

		/*
		* When value is very small and near zero
		*/
		void nearZeroHandler();

		/*
		* When we want more decimals for the number
		*/
		void decimalHnadler();
	};
}

