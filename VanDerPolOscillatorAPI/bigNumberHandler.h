#pragma once
namespace vdpo{
	/*
	* A handler of Infinite , NaN and other formats bigger than the double type can handle
	*/
	class bigNumberHandler
	{
	public:
		double bigNumber;

		bigNumberHandler(double UnhandledbigNumber);
		
		~bigNumberHandler();

	private:
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

