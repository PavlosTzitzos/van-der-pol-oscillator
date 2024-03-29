#include "pch.h"
#include "CppUnitTest.h"
#include "../VanDerPolOscillatorAPI/gnuplot-iostream.h"

// include cpp files only to test the implementations directly
#include "../VanDerPolOscillatorAPI/enumerators.cpp"
#include "../VanDerPolOscillatorAPI/systemModel.cpp"
#include "../VanDerPolOscillatorAPI/controlAlgorithm.cpp"
#include "../VanDerPolOscillatorAPI/displayData.cpp"
#include "../VanDerPolOscillator/VanDerPolOscillatorFunctions.cpp"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace VanDerPolOscillatorTests
{
	TEST_CLASS(VanDerPolOscillatorTests)
	{
	public:
		//-------------------------------------------------------------------------------------------//
		// Test initial functions only
		TEST_METHOD(TestFunctionControlSignal2)
		{
			std::array<double,2> x = { 1, 0.5 };
			std::array<double,3> theta = { 0.1,0.1,0.0 };
			double u = theta[0] * x[0] + theta[1] * x[1];
			Logger::WriteMessage("In TestFunctionControlSignal2");
			Logger::WriteMessage(" - Testing control signal function with 2 theta parameters");
			double uTest = u2(x,theta);
			Assert::AreEqual(u, uTest, L"FAILED - u is not calculated correctly");
		}
		TEST_METHOD(TestFunctionControlSignal3)
		{
			std::array<double, 2> x = { 1, 0.5 };
			std::array<double, 3> theta = { -0.1,0.1,-0.1 };
			double u = theta[0] * x[0] + theta[1] * x[1] + theta[2] * x[1] * (x[0]*x[0] - 1);
			Logger::WriteMessage("In TestFunctionControlSignal3");
			Logger::WriteMessage(" - Testing control signal function with 3 theta parameters");
			double uTest = u2(x, theta);
			Assert::AreEqual(u, uTest, L"FAILED - u is not calculated correctly");
		}
		TEST_METHOD(TestDerivativeXu2)
		{
			std::array<double, 2> x = { 1, 0.5 };
			std::array<double, 3> theta = { 0.1,0.1,0.0 };
			double dx[2] = { 0.0, 0.0 };
			double k = 1;
			double m = 3;
			double c = 2;
			double u = theta[0] * x[0] + theta[1] * x[1];
			dx[0] = x[1];
			dx[1] = -(c / m) * (std::pow(x[0], 2) - 1) * x[1] - (k / m) * x[0] + u / m;
			Logger::WriteMessage("In TestDerivativeXu2");
			std::array<double, 2> dxTest = f(x,u2(x,theta),k,m,c);
			Assert::AreEqual(dxTest[0], dx[0], L"FAILED - Derivative is not calculated correctly for x0");
			Assert::AreEqual(dxTest[1], dx[1], L"FAILED - Derivative is not calculated correctly for x1");
		}
		TEST_METHOD(TestDerivativeXu3)
		{
			std::array<double, 2> x = { 1, 0.5 };
			std::array<double, 3> theta = { -0.1,0.1,-0.1 };
			double dx[2] = { 0.0, 0.0 };
			double k = 1;
			double m = 3;
			double c = 2;
			double u = theta[0] * x[0] + theta[1] * x[1] + theta[2] * x[1] * (x[0] * x[0] - 1);
			dx[0] = x[1];
			dx[1] = -(c / m) * (std::pow(x[0], 2) - 1) * x[1] - (k / m) * x[0] + u / m;
			Logger::WriteMessage("In TestDerivativeXu3");
			std::array<double, 2> dxTest = f(x, u3(x, theta), k, m, c);
			Assert::AreEqual(dxTest[0], dx[0], L"FAILED - Derivative is not calculated correctly for x0");
			Assert::AreEqual(dxTest[1], dx[1], L"FAILED - Derivative is not calculated correctly for x1");
		}
		TEST_METHOD(TestFDAlgorithm2Theta)
		{
			//
			double x[2] = { 1, 0.5 };
			double x0[2] = { 1, 0.5 };
			double theta[3] = { 0.1,0.1, 0.05 };
			double dTheta = dtheta;
			double dx[2] = { 0.0, 0.0 };
			double k = 1;
			double m = 3;
			double c = 2;
			double Perf = 100;
			double P0;
			double P1;
			double P2;
			// add implementation
			int counter = 0;
			double local_theta[3];
			local_theta[0] = theta[0];
			local_theta[1] = theta[1];
			local_theta[2] = theta[2];
			while (std::abs(Perf) > 1 && counter < 100)
			{
				//
				P0 = 0;
				for (double t = 0; t < 10; t += 0.5)
				{
					double u = local_theta[0] * x[0] + local_theta[1] * x[1];
					dx[0] = x[1];
					dx[1] = -(c / m) * (std::pow(x[0], 2) - 1) * x[1] - (k / m) * x[0] + u / m;

					x[0] = x[0] - 0.5 * dx[0];
					x[1] = x[1] - 0.5 * dx[1];

					P0 += std::sqrt(x[0] * x[0] + x[1] * x[1]);
				}
				Perf = P0;
				x[0] = x0[0];
				x[1] = x0[1];
				local_theta[0] += dtheta;
				P1 = 0;
				for (double t = 0; t < 10; t += 0.5)
				{
					double u = local_theta[0] * x[0] + local_theta[1] * x[1];
					dx[0] = x[1];
					dx[1] = -(c / m) * (std::pow(x[0], 2) - 1) * x[1] - (k / m) * x[0] + u / m;

					x[0] = x[0] - 0.5 * dx[0];
					x[1] = x[1] - 0.5 * dx[1];

					P1 += std::sqrt(x[0] * x[0] + x[1] * x[1]);
				}
				x[0] = x0[0];
				x[1] = x0[1];
				local_theta[0] = theta[0];
				local_theta[1] += dtheta;
				P2 = 0;
				for (double t = 0; t < 10; t += 0.5)
				{
					double u = local_theta[0] * x[0] + local_theta[1] * x[1];
					dx[0] = x[1];
					dx[1] = -(c / m) * (std::pow(x[0], 2) - 1) * x[1] - (k / m) * x[0] + u / m;

					x[0] = x[0] - 0.5 * dx[0];
					x[1] = x[1] - 0.5 * dx[1];

					P2 += std::sqrt(x[0] * x[0] + x[1] * x[1]);
				}
				x[0] = x0[0];
				x[1] = x0[1];
				local_theta[1] = theta[1];

				theta[0] = theta[0] - hetta * (P1 - P0) / dTheta;
				theta[1] = theta[1] - hetta * (P2 - P0) / dTheta;

				local_theta[0] = theta[0];
				local_theta[1] = theta[1];

				counter += 1;
			}
			
			std::array<double, 2> X0 = { 1,0.5 };
			std::array<double, 3> Theta = { 0.1,0.1,0.05 };
			std::array<double, 3> temp_sysParameters = { k,m,c }; // system parameters (k,m,c)
			std::array<std::array<double, MAX_REPEATS>, 2> res;
			algorithms sel = FD2;
			sel = FD2;
			res = gradientDescent(X0, Theta, temp_sysParameters, "step1", sel);
			double tempRes = 0.0;
			for (int i = 0; i < MAX_REPEATS; i++)
			{
				if (res[0][i] > 0.0)
					tempRes = res[0][i];
			}
			bool ok = (0.02 > std::abs(Perf - tempRes)) ? true : false;
			Assert::IsTrue(ok, L"FAILED - Final Performances do not match");
		}
		//-------------------------------------------------------------------------------------------//
		// Test Classes only
		TEST_METHOD(TestEnumerators)
		{
			Logger::WriteMessage("In TestEnumerators");
			vdpo::algorithm testValue = vdpo::algorithm::FD;
		}
		TEST_METHOD(TestSystemModelParameterM)
		{
			Logger::WriteMessage("In TestSystemModelParameterM");
			vdpo::systemModel testModel = vdpo::systemModel::systemModel(1.0, 3.0, 2.0, vdpo::theta::three);
			double m = testModel.getSysParM();
			Assert::AreEqual(3.0,m,L"FAILED - m is not equal to 3.0");
		}
		TEST_METHOD(TestSystemModelParameterK)
		{
			Logger::WriteMessage("In TestSystemModelParameterK");
			vdpo::systemModel testModel = vdpo::systemModel::systemModel(1.0, 3.0, 2.0, vdpo::theta::three);
			double k = testModel.getSysParK();
			Assert::AreEqual(2.0, k, L"FAILED - (Should fail) k is not equal to 1.0", LINE_INFO());
		}
		TEST_METHOD(TestSystemModelParameterC)
		{
			Logger::WriteMessage("In TestSystemModelParameterC");
			vdpo::systemModel testModel = vdpo::systemModel::systemModel(1.0, 3.0, 2.0, vdpo::theta::three);
			double c = testModel.getSysParC();
			Assert::AreEqual(2.0, c, L"PASSED - c equals 2.0");
		}
		TEST_METHOD(TestSystemModelAccessors)
		{
			Logger::WriteMessage("In TestSystemModelAccessors");
			vdpo::systemModel testModel = vdpo::systemModel::systemModel(1.0, 3.0, 2.0, vdpo::theta::three);
			double testVector[2] = { 0.0 , 1.0 };
			testModel.setTheta2(testVector);
			Assert::AreEqual(testVector[0], testModel.getTheta2()[0], L"FAILED - (Should pass) Input theta does not match given theta at position 0");
			Assert::AreEqual(testVector[0], testModel.getTheta2()[1], L"FAILED - (Should fail) Input theta does not match given theta at position 1");
		}
		TEST_METHOD(TestSystemModelControlSignalAndDerivativeX)
		{
			double x[2] = { 1, 0.5 };
			double theta[2] = { 0.1,0.1 };
			double dx[2] = { 0.0, 0.0 };
			double k = 1;
			double m = 3;
			double c = 2;
			double u = theta[0] * x[0] + theta[1] * x[1];
			dx[0] = x[1];
			dx[1] = -(c/m)*(std::pow(x[0],2) - 1)*x[1]-(k/m)*x[0] + u/m;
			Logger::WriteMessage("In TestSystemModelControlSignalAndDerivativeX");
			vdpo::systemModel testModel = vdpo::systemModel::systemModel(1.0, 3.0, 2.0, vdpo::theta::two);
			
			testModel.setTheta2(theta);
			Assert::AreEqual(theta[0], testModel.getTheta2()[0], L"FAILED - theta0 is not setted correctly");
			Assert::AreEqual(theta[1], testModel.getTheta2()[1], L"FAILED - theta1 is not setted correctly");
			testModel.setSSV(x);
			Assert::AreEqual(x[0], testModel.getSSV()[0], L"FAILED - x0 is not setted correctly");
			Assert::AreEqual(x[1], testModel.getSSV()[1], L"FAILED - x1 is not setted correctly");

			testModel.dxCalculate();

			Assert::AreEqual(u, testModel.uTest, L"FAILED - u is not calculated correctly");

			Assert::AreEqual(testModel.getDx()[0], dx[0], L"FAILED - Derivative is not calculated correctly for x0");
			Assert::AreEqual(testModel.getDx()[1], dx[1], L"FAILED - Derivative is not calculated correctly for x1");
		}
		TEST_METHOD(TestGnuPlotVectors1)
		{
			Logger::WriteMessage("In TestGnuPlotVectors1");
			vdpo::controlAlgorithm testAlgo = vdpo::controlAlgorithm::controlAlgorithm();
			vdpo::displayData testData = vdpo::displayData::displayData("Test","testSetA","testSetB","dataFile.txt");
			testData.vector1.push_back(1.0);
			testData.vector2.push_back(2.0);
			testData.vector1.push_back(2.0);
			testData.vector2.push_back(3.0);
			testData.vector1.push_back(3.0);
			testData.vector2.push_back(4.0);
			testData.availableAlgorithms();
			Assert::AreEqual(testData.vector1[0], 0.0, L"FAILED - Vector 1 Elements do not match at position 0 should be 0.0");
			Assert::AreEqual(testData.vector1[1], 1.0, L"FAILED - Vector 1 Elements do not match at position 1 should be 1.0");
			Assert::AreEqual(testData.vector1[2], 2.0, L"FAILED - Vector 1 Elements do not match at position 2 should be 2.0");
			Assert::AreEqual(testData.vector1[3], 3.0, L"FAILED - Vector 1 Elements do not match at position 3 should be 3.0");

			Assert::AreEqual(testData.vector2[0], 0.0, L"FAILED - Vector 2 Elements do not match at position 0 should be 0.0");
			Assert::AreEqual(testData.vector2[1], 2.0, L"FAILED - Vector 2 Elements do not match at position 1 should be 2.0");
			Assert::AreEqual(testData.vector2[2], 3.0, L"FAILED - Vector 2 Elements do not match at position 2 should be 3.0");
			Assert::AreEqual(testData.vector2[3], 4.0, L"FAILED - Vector 2 Elements do not match at position 3 should be 4.0");
			testData.exportData();
			//testData.plotData1();
			Logger::WriteMessage("Test completed");
		}
		TEST_METHOD(TestGnuPlotVectors2)
		{
			Logger::WriteMessage("In TestGnuPlotVectors2");
			vdpo::controlAlgorithm testAlgo = vdpo::controlAlgorithm::controlAlgorithm();
			vdpo::displayData testData = vdpo::displayData::displayData("Test", "testSetA", "testSetB", "dataFile.txt");
			testData.vector1.push_back(1.0);
			testData.vector2.push_back(2.0);
			testData.vector1.push_back(2.0);
			testData.vector2.push_back(3.0);
			testData.vector1.push_back(3.0);
			testData.exportData();
			//testData.plotData2();
			Logger::WriteMessage("Test completed");
		}
		TEST_METHOD(TestControlAlgorithm)
		{
			double x[2] = { 1, 0.5 };
			double theta[3] = { 0.1,0.1, 0.05 };
			double dx[2] = { 0.0, 0.0 };
			double k = 1;
			double m = 3;
			double c = 2;
			// Default constructor FD , 2 theta , no sensitivity analysis
			vdpo::controlAlgorithm testAlgorithm = vdpo::controlAlgorithm::controlAlgorithm();
			// Set times for simulation (performance / value / cost functions)
			testAlgorithm.setStartTime(0);
			testAlgorithm.setStepTime(0.5);
			testAlgorithm.setFinalTime(10);
			
			testAlgorithm.setMaxIterations(100);

			testAlgorithm.thetaVar[0] = theta[0];
			testAlgorithm.thetaVar[1] = theta[1];
			Assert::AreEqual(theta[0], testAlgorithm.thetaVar[0], L"FAILED - theta0 is not setted correctly");
			Assert::AreEqual(theta[1], testAlgorithm.thetaVar[1], L"FAILED - theta1 is not setted correctly");
			Assert::AreEqual(theta[2], testAlgorithm.thetaVar[2], L"FAILED - (should fail) theta2 is not setted correctly");
		}
		TEST_METHOD(TestFDPerformanceControlAlgorithm)
		{
			double x[2] = { 1, 0.5 };
			double theta[3] = { 0.1,0.1, 0.05 };
			double dx[2] = { 0.0, 0.0 };
			double k = 1;
			double m = 3;
			double c = 2;
			vdpo::systemModel testModel = vdpo::systemModel::systemModel(1.0, 3.0, 2.0, vdpo::theta::two);

			// Default constructor FD , 2 theta , no sensitivity analysis
			vdpo::FD testFD = vdpo::FD::FD();

			//pass the x and theta values
			testFD.x[0] = x[0];
			testFD.x[1] = x[1];
			testFD.thetaVar[0] = theta[0];
			testFD.thetaVar[1] = theta[1];
			testFD.thetaVar[2] = theta[2];

			// Set times for simulation (performance / value / cost functions)
			testFD.setStartTime(0);
			testFD.setStepTime(0.5);
			testFD.setFinalTime(10);
			testFD.iterationsCalculate();
			Assert::AreEqual(21.0,testFD.simulationIterations, L"FAILED - (should not fail) time iterations are not correctly  calculated");

			testFD.setMaxIterations(100);

			testFD.thetaVar[0] = theta[0];
			testFD.thetaVar[1] = theta[1];
			Assert::AreEqual(theta[0], testFD.thetaVar[0], L"FAILED - theta0 is not setted correctly");
			Assert::AreEqual(theta[1], testFD.thetaVar[1], L"FAILED - theta1 is not setted correctly");
			//Assert::AreEqual(theta[2], testFD.thetaVar[2], L"FAILED - (should fail) theta2 is not setted correctly");
			testFD.setDtheta(0.5);
			testFD.setHetta(1);

			testFD.setSystemModel(testModel);
			//testFD.runAlgorithm();
			double P = 0;
			for (double t = 0; t < 10; t += 0.5)
			{
				double u = theta[0] * x[0] + theta[1] * x[1];
				dx[0] = x[1];
				dx[1] = -(c / m) * (std::pow(x[0], 2) - 1) * x[1] - (k / m) * x[0] + u / m;

				x[0] = x[0] - 0.5 * dx[0];
				x[1] = x[1] - 0.5 * dx[1];

				P += std::sqrt(x[0] * x[0] + x[1] * x[1]);
			}

			Assert::AreEqual(P, testFD.P, L"FAILED - Should fail - Performance is not correct");
		}
		TEST_METHOD(TestFDMethodControlAlgorithm)
		{
			double x[2] = { 1, 0.5 };
			double x0[2] = { 1, 0.5 };
			double theta[3] = { 0.1,0.1, 0.05 };
			double dTheta = dtheta;
			double dx[2] = { 0.0, 0.0 };
			double k = 1;
			double m = 3;
			double c = 2;
			vdpo::systemModel testModel = vdpo::systemModel::systemModel(1.0, 3.0, 2.0, vdpo::theta::two);

			// Default constructor FD , 2 theta , no sensitivity analysis
			vdpo::FD testFD = vdpo::FD::FD();

			//pass the x and theta values
			testFD.x[0] = x[0];
			testFD.x[1] = x[1];
			testFD.thetaVar[0] = theta[0];
			testFD.thetaVar[1] = theta[1];
			testFD.thetaVar[2] = theta[2];

			// Set times for simulation (performance / value / cost functions)
			testFD.setStartTime(0);
			testFD.setStepTime(0.5);
			testFD.setFinalTime(10);
			testFD.iterationsCalculate();
			Assert::AreEqual(21.0, testFD.simulationIterations, L"FAILED - (should not fail) time iterations are not correctly  calculated");

			testFD.setMaxIterations(100);

			testFD.thetaVar[0] = theta[0];
			testFD.thetaVar[1] = theta[1];
			Assert::AreEqual(theta[0], testFD.thetaVar[0], L"FAILED - theta0 is not setted correctly");
			Assert::AreEqual(theta[1], testFD.thetaVar[1], L"FAILED - theta1 is not setted correctly");
			//Assert::AreEqual(theta[2], testFD.thetaVar[2], L"FAILED - (should fail) theta2 is not setted correctly");
			//testFD.setDtheta(0.5);
			//testFD.setHetta(1);

			testFD.setSystemModel(testModel);
			testFD.runAlgorithm();
			double Perf = 100;
			double P0;
			double P1;
			double P2;
			// add implementation
			int counter = 0;
			double local_theta[3];
			local_theta[0] = theta[0];
			local_theta[1] = theta[1];
			local_theta[2] = theta[2];

			while (std::abs(Perf) > 1 && counter < 100)
			{
				//
				P0 = 0;
				for (double t = 0; t < 10; t += 0.5)
				{
					double u = local_theta[0] * x[0] + local_theta[1] * x[1];
					dx[0] = x[1];
					dx[1] = -(c / m) * (std::pow(x[0], 2) - 1) * x[1] - (k / m) * x[0] + u / m;

					x[0] = x[0] - 0.5 * dx[0];
					x[1] = x[1] - 0.5 * dx[1];

					P0 += std::sqrt(x[0] * x[0] + x[1] * x[1]);
				}
				Perf = P0;
				x[0] = x0[0];
				x[1] = x0[1];
				local_theta[0] += dtheta;
				P1 = 0;
				for (double t = 0; t < 10; t += 0.5)
				{
					double u = local_theta[0] * x[0] + local_theta[1] * x[1];
					dx[0] = x[1];
					dx[1] = -(c / m) * (std::pow(x[0], 2) - 1) * x[1] - (k / m) * x[0] + u / m;

					x[0] = x[0] - 0.5 * dx[0];
					x[1] = x[1] - 0.5 * dx[1];

					P1 += std::sqrt(x[0] * x[0] + x[1] * x[1]);
				}
				x[0] = x0[0];
				x[1] = x0[1];
				local_theta[0] = theta[0];
				local_theta[1] += dtheta;
				P2 = 0;
				for (double t = 0; t < 10; t += 0.5)
				{
					double u = local_theta[0] * x[0] + local_theta[1] * x[1];
					dx[0] = x[1];
					dx[1] = -(c / m) * (std::pow(x[0], 2) - 1) * x[1] - (k / m) * x[0] + u / m;

					x[0] = x[0] - 0.5 * dx[0];
					x[1] = x[1] - 0.5 * dx[1];

					P2 += std::sqrt(x[0] * x[0] + x[1] * x[1]);
				}
				x[0] = x0[0];
				x[1] = x0[1];
				local_theta[1] = theta[1];
				
				theta[0] = theta[0] - hetta * (P1 - P0) / dTheta;
				theta[1] = theta[1] - hetta * (P2 - P0) / dTheta;

				local_theta[0] = theta[0];
				local_theta[1] = theta[1];

				counter += 1;
			}
			Assert::AreEqual(Perf, testFD.P, L"FAILED - Should not fail - Performance is not correct");
			Assert::AreEqual(theta[0], testFD.thetaVar[0], L"FAILED - Should not fail - Theta 0 is not correct");
			Assert::AreEqual(theta[1], testFD.thetaVar[1], L"FAILED - Should not fail - Theta 1 is not correct");
		}
		TEST_METHOD(TestSPSAMethodControlAlgorithm)
		{
			double x[2] = { 0.5, 0.1 };
			double x0[2] = { 0.5, 0.1 };
			double theta[3] = { -0.1,0.1, 1 };
			double dTheta = dtheta;
			double dx[2] = { 0.0, 0.0 };
			double k = 1;
			double m = 3;
			double c = 2;
			vdpo::systemModel testModel = vdpo::systemModel::systemModel(k, m, c, vdpo::theta::two);

			// Default constructor FD , 2 theta , no sensitivity analysis
			vdpo::SPSA testSPSA = vdpo::SPSA::SPSA();

			//pass the x and theta values
			testSPSA.x[0] = x[0];
			testSPSA.x[1] = x[1];
			testSPSA.thetaVar[0] = theta[0];
			testSPSA.thetaVar[1] = theta[1];
			testSPSA.thetaVar[2] = theta[2];

			// Set times for simulation (performance / value / cost functions)
			testSPSA.setStartTime(0);
			testSPSA.setStepTime(0.5);
			testSPSA.setFinalTime(20);
			testSPSA.iterationsCalculate();
			Assert::AreEqual(41.0, testSPSA.simulationIterations, L"FAILED - (should not fail) time iterations are not correctly  calculated");

			testSPSA.setMaxIterations(100);

			testSPSA.thetaVar[0] = theta[0];
			testSPSA.thetaVar[1] = theta[1];
			Assert::AreEqual(theta[0], testSPSA.thetaVar[0], L"FAILED - theta0 is not setted correctly");
			Assert::AreEqual(theta[1], testSPSA.thetaVar[1], L"FAILED - theta1 is not setted correctly");
			//Assert::AreEqual(theta[2], testFD.thetaVar[2], L"FAILED - (should fail) theta2 is not setted correctly");
			//testFD.setDtheta(0.5);
			//testFD.setHetta(1);

			testSPSA.setSystemModel(testModel);
			testSPSA.runAlgorithm();
			double Perf = 100;
			double P0, P1, P2, P3;
			// add implementation
			int counter = 0;
			double local_theta[3];
			local_theta[0] = theta[0];
			local_theta[1] = theta[1];
			local_theta[2] = theta[2];

			double ak, ck, Dk[2];

			while (std::abs(Perf) > 1 && counter < 100)
			{
				// Calculate ak, ck, Dk
				ck = betta / std::powl(counter + 1, gamma);
				ak = a / std::powl(A + counter + 1, alpha);
				std::random_device rd;
				std::mt19937 gen(rd());
				std::bernoulli_distribution d(p);
				for (int i = 0; i < 2; i++)
					Dk[i] = d(gen) ? 1.0 : -1.0;
				//
				P0 = 0;
				local_theta[0] += ck * Dk[0];
				for (double t = 0; t < 20; t += 0.5)
				{
					double u = local_theta[0] * x[0] + local_theta[1] * x[1];
					dx[0] = x[1];
					dx[1] = -(c / m) * (std::pow(x[0], 2) - 1) * x[1] - (k / m) * x[0] + u / m;

					x[0] = x[0] - 0.5 * dx[0];
					x[1] = x[1] - 0.5 * dx[1];

					P0 += std::sqrt(x[0] * x[0] + x[1] * x[1]);
				}
				Perf = P0;
				x[0] = x0[0];
				x[1] = x0[1];
				local_theta[0] = theta[0];
				local_theta[1] += ck * Dk[1];
				P1 = 0;
				for (double t = 0; t < 20; t += 0.5)
				{
					double u = local_theta[0] * x[0] + local_theta[1] * x[1];
					dx[0] = x[1];
					dx[1] = -(c / m) * (std::pow(x[0], 2) - 1) * x[1] - (k / m) * x[0] + u / m;

					x[0] = x[0] - 0.5 * dx[0];
					x[1] = x[1] - 0.5 * dx[1];

					P1 += std::sqrt(x[0] * x[0] + x[1] * x[1]);
				}
				x[0] = x0[0];
				x[1] = x0[1];
				local_theta[1] = theta[1];
				local_theta[0] -= ck * Dk[0];
				P2 = 0;
				for (double t = 0; t < 20; t += 0.5)
				{
					double u = local_theta[0] * x[0] + local_theta[1] * x[1];
					dx[0] = x[1];
					dx[1] = -(c / m) * (std::pow(x[0], 2) - 1) * x[1] - (k / m) * x[0] + u / m;

					x[0] = x[0] - 0.5 * dx[0];
					x[1] = x[1] - 0.5 * dx[1];

					P2 += std::sqrt(x[0] * x[0] + x[1] * x[1]);
				}
				x[0] = x0[0];
				x[1] = x0[1];
				local_theta[0] = theta[0];
				local_theta[1] -= ck * Dk[1];
				P3 = 0;
				for (double t = 0; t < 20; t += 0.5)
				{
					double u = local_theta[0] * x[0] + local_theta[1] * x[1];
					dx[0] = x[1];
					dx[1] = -(c / m) * (std::pow(x[0], 2) - 1) * x[1] - (k / m) * x[0] + u / m;

					x[0] = x[0] - 0.5 * dx[0];
					x[1] = x[1] - 0.5 * dx[1];

					P2 += std::sqrt(x[0] * x[0] + x[1] * x[1]);
				}
				x[0] = x0[0];
				x[1] = x0[1];
				local_theta[1] = theta[1];

				theta[0] = theta[0] - ak * (P0 - P2) / (2 * ck * Dk[0]);
				theta[1] = theta[1] - ak * (P1 - P3) / (2 * ck * Dk[1]);

				local_theta[0] = theta[0];
				local_theta[1] = theta[1];

				counter += 1;
			}
			Assert::AreEqual(Perf, testSPSA.P, L"FAILED - Should not fail but always fails - Performance is not correct");
		}
		TEST_METHOD(TestFDSensitivityAnalyzer)
		{
			double x[2] = { 1, 0.5 };
			double x0[2] = { 1, 0.5 };
			double theta[3] = { 0.1,0.1, 0.05 };
			double dTheta = dtheta;
			double dx[2] = { 0.0, 0.0 };
			double k = 1;
			double m = 3;
			double c = 2;
			vdpo::systemModel testModel = vdpo::systemModel::systemModel(1.0, 3.0, 2.0, vdpo::theta::two);

			// Default constructor FD , 2 theta , run sensitivity analysis
			vdpo::FD testFD = vdpo::FD::FD(vdpo::theta::two, true);

			//pass the x and theta values
			testFD.x[0] = x[0];
			testFD.x[1] = x[1];
			testFD.thetaVar[0] = theta[0];
			testFD.thetaVar[1] = theta[1];
			testFD.thetaVar[2] = theta[2];

			// Set times for simulation (performance / value / cost functions)
			testFD.setStartTime(0);
			testFD.setStepTime(0.5);
			testFD.setFinalTime(10);
			testFD.iterationsCalculate();
			Assert::AreEqual(21.0, testFD.simulationIterations, L"FAILED - (should not fail) time iterations are not correctly  calculated");

			testFD.setMaxIterations(100);

			testFD.thetaVar[0] = theta[0];
			testFD.thetaVar[1] = theta[1];
			Assert::AreEqual(theta[0], testFD.thetaVar[0], L"FAILED - theta0 is not setted correctly");
			Assert::AreEqual(theta[1], testFD.thetaVar[1], L"FAILED - theta1 is not setted correctly");
			//Assert::AreEqual(theta[2], testFD.thetaVar[2], L"FAILED - (should fail) theta2 is not setted correctly");
			//testFD.setDtheta(0.5);
			//testFD.setHetta(1);

			testFD.setSystemModel(testModel);
			testFD.runAlgorithm();
			Logger::WriteMessage("Finished testing FD sensitivity analysis.");
		}
		TEST_METHOD(TestSPSASensitivityAnalyzer)
		{
			double x[2] = { 0.5, 0.1 };
			double x0[2] = { 0.5, 0.1 };
			double theta[3] = { -0.1,0.1, 1 };
			double dTheta = dtheta;
			double dx[2] = { 0.0, 0.0 };
			double k = 1;
			double m = 3;
			double c = 2;
			vdpo::systemModel testModel = vdpo::systemModel::systemModel(k, m, c, vdpo::theta::two);

			// Default constructor FD , 2 theta , no sensitivity analysis
			vdpo::SPSA testSPSA = vdpo::SPSA::SPSA(vdpo::theta::two,true);

			//pass the x and theta values
			testSPSA.x[0] = x[0];
			testSPSA.x[1] = x[1];
			testSPSA.thetaVar[0] = theta[0];
			testSPSA.thetaVar[1] = theta[1];
			testSPSA.thetaVar[2] = theta[2];

			// Set times for simulation (performance / value / cost functions)
			testSPSA.setStartTime(0);
			testSPSA.setStepTime(0.5);
			testSPSA.setFinalTime(20);
			testSPSA.iterationsCalculate();
			Assert::AreEqual(41.0, testSPSA.simulationIterations, L"FAILED - (should not fail) time iterations are not correctly  calculated");

			testSPSA.setMaxIterations(100);

			testSPSA.thetaVar[0] = theta[0];
			testSPSA.thetaVar[1] = theta[1];
			Assert::AreEqual(theta[0], testSPSA.thetaVar[0], L"FAILED - theta0 is not setted correctly");
			Assert::AreEqual(theta[1], testSPSA.thetaVar[1], L"FAILED - theta1 is not setted correctly");
			

			testSPSA.setSystemModel(testModel);
			testSPSA.runAlgorithm();
			Logger::WriteMessage("Finished testing SPSA sensitivity analysis.");
		}
		TEST_METHOD(TestLQRSensitivityAnalyzer)
		{
			double x[2] = { 1, 0.5 };
			double dx[2] = { 0.0, 0.0 };
			double k = 1;
			double m = 3;
			double c = 2;
			vdpo::systemModel testModel = vdpo::systemModel::systemModel(k, m, c, vdpo::theta::two);

			double Q[4] = { 1.0, 0.0, 0.0, 1.0 };
			double r = 1.0;
			// constructor LQR , with sensitivity analysis
			vdpo::LQR testLQR = vdpo::LQR::LQR(Q, r, true);

			//pass the x and theta values
			testLQR.x[0] = x[0];
			testLQR.x[1] = x[1];

			// Set times for simulation (performance / value / cost functions)
			testLQR.setStartTime(0);
			testLQR.setStepTime(0.5);
			testLQR.setFinalTime(20);
			testLQR.iterationsCalculate();
			Assert::AreEqual(41.0, testLQR.simulationIterations, L"FAILED - (should not fail) time iterations are not correctly  calculated");

			testLQR.setMaxIterations(100);

			testLQR.setSystemModel(testModel);
			testLQR.runAlgorithm();
			Logger::WriteMessage("Finished testing LQR sensitivity analysis.");
		}
		//-------------------------------------------------------------------------------------------//
		// Compare results from functions and Classes
		TEST_METHOD(FunctionVsApiProjectsDerivativeXu2)
		{
			Logger::WriteMessage("In FunctionVsApiProjectsDerivativeXu2");
			Logger::WriteMessage(" - Combare old project with API version");
			Logger::WriteMessage("\n Results should always be the same");
			// Part 1: Test functions only
			std::array<double, 2> x = { 1, 0.5 };
			std::array<double, 3> theta = { 0.1,0.1,0.0 };
			double xx[2] = {1, 0.5};
			double Ttheta[2] = {0.1,0.1};
			double dx[2] = { 0.0, 0.0 };
			double k = 1;
			double m = 3;
			double c = 2;
			double u = theta[0] * x[0] + theta[1] * x[1];
			dx[0] = x[1];
			dx[1] = -(c / m) * (std::pow(x[0], 2) - 1) * x[1] - (k / m) * x[0] + u / m;
			Logger::WriteMessage("In TestDerivativeXu2");
			std::array<double, 2> dxTest = f(x, u2(x, theta), k, m, c);
			Assert::AreEqual(dxTest[0], dx[0], L"FAILED - Derivative is not calculated correctly for x0");
			Assert::AreEqual(dxTest[1], dx[1], L"FAILED - Derivative is not calculated correctly for x1");
			// Part 2: Test Classes only
			vdpo::systemModel testModel = vdpo::systemModel::systemModel(1.0, 3.0, 2.0, vdpo::theta::two);

			testModel.setTheta2(Ttheta);
			Assert::AreEqual(theta[0], testModel.getTheta2()[0], L"FAILED - theta0 is not setted correctly");
			Assert::AreEqual(theta[1], testModel.getTheta2()[1], L"FAILED - theta1 is not setted correctly");
			testModel.setSSV(xx);
			Assert::AreEqual(x[0], testModel.getSSV()[0], L"FAILED - x0 is not setted correctly");
			Assert::AreEqual(x[1], testModel.getSSV()[1], L"FAILED - x1 is not setted correctly");

			testModel.dxCalculate();

			Assert::AreEqual(u, testModel.uTest, L"FAILED - u is not calculated correctly");

			Assert::AreEqual(testModel.getDx()[0], dx[0], L"FAILED - Derivative is not calculated correctly for x0");
			Assert::AreEqual(testModel.getDx()[1], dx[1], L"FAILED - Derivative is not calculated correctly for x1");
			// Part 3: Test function vs Classes
			Assert::AreEqual(dxTest[0], testModel.getDx()[0], L"FAILED - Derivative x0 does not match please review implementation");
			Assert::AreEqual(dxTest[1], testModel.getDx()[1], L"FAILED - Derivative x1 does not match please review implementation");

		}
		TEST_METHOD(CompareLQRMethods)
		{
			double x[2] = { 1, 0.5 };
			double dx[2] = { 0.0, 0.0 };
			double k = 1;
			double m = 3;
			double c = 2;
			vdpo::systemModel testModel = vdpo::systemModel::systemModel(k, m, c, vdpo::theta::two);

			// Default constructor FD , 2 theta , no sensitivity analysis
			vdpo::LQR testLQR = vdpo::LQR::LQR();

			//pass the x and theta values
			testLQR.x[0] = x[0];
			testLQR.x[1] = x[1];

			// Set times for simulation (performance / value / cost functions)
			testLQR.setStartTime(0);
			testLQR.setStepTime(0.5);
			testLQR.setFinalTime(20);
			testLQR.iterationsCalculate();
			Assert::AreEqual(41.0, testLQR.simulationIterations, L"FAILED - (should not fail) time iterations are not correctly  calculated");

			testLQR.setMaxIterations(100);

			testLQR.setSystemModel(testModel);
			testLQR.runAlgorithm();
			double Perf = 100;
			// add implementation
			std::array<double, 2> x0 = { 0.5, 0.1 };
			std::array<double, 3> theta = { -0.1, 0.1, 1 }; // theta parameters
			std::array<double, 3> temp_sysParameters = { k, m, c }; // system parameters (k,m,c)
			algorithms sel = LQR;
			std::array<double, timeFinalLQR> resLQR = lqrTop(x0, theta, temp_sysParameters, "step10", sel);

			bool okJ = (std::abs(resLQR[timeFinalLQR - 1] - testLQR.J) < 0.001) ? true : false;
			Assert::IsTrue(okJ, L"FAILED - Should fail - Performance is not correct due to different x0");
		}
	};
}
