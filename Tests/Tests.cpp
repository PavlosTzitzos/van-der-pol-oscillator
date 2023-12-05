#include "pch.h"
#include "CppUnitTest.h"
#include "../VanDerPolOscillator/VanDerPolOscillator.cpp" // this is not yet created please add this file !!

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace Tests
{
	TEST_CLASS(Tests)
	{
	public:
		
		TEST_METHOD(TestDerivate)
		{
			const int points = 50;
			std::array<double, 3> par = { 1,1,1}; // k,c,m

			// generate position values
			std::array<double, points> x1, x2, d_x1, d_x2;
			double u = 0;
			for (int i = 0;i < points;i++)
			{
				x1[i] = i;
				x2[i] = points - i;
			}

			// generate d_x
			for (int i = 0;i < points;i++)
			{
				d_x1[i] = x2[i];
				d_x2[i] = (par[0]*x2[i]*(x1[i]*x1[i] - 1) * (x1[i] * x1[i] - 1)) / par[2];
			}

			// compare with f function
			std::array<double, 2> x, res, d_x;
			for (int i = 0;i < points;i++)
			{
				x[0] = x1[i];
				x[1] = x2[i];
				res = f(x, u);
				d_x[0] = d_x1[i];
				d_x[1] = d_x2[i];

				// pass or fail ?

				Assert::AreEqual(d_x[0], res[0]);

				Assert::AreEqual(d_x[1], res[1]);
			}
			
		}
		TEST_METHOD(TestControlSignal)
		{
			const int points = 50;
			std::array<double, 3> par = { 1,1,1 }; // k,c,m

			// generate position values
			std::array<double, points> x1, x2, d_x1, d_x2;
			double u = 0;
			for (int i = 0;i < points;i++)
			{
				x1[i] = i;
				x2[i] = points - i;
			}

			// generate d_x
			for (int i = 0;i < points;i++)
			{
				d_x1[i] = x2[i];
				d_x2[i] = (par[0] * x2[i] * (x1[i] * x1[i] - 1) * (x1[i] * x1[i] - 1)) / par[2];
			}

			// compare with f function

			// pass or fail ?
		}
	};
}
