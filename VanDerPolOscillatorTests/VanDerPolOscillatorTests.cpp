#include "pch.h"
#include "CppUnitTest.h"
#include "../VanDerPolOscillatorAPI/gnuplot-iostream.h"
#include "../VanDerPolOscillatorAPI/enumerators.cpp"
#include "../VanDerPolOscillatorAPI/systemModel.cpp"
#include "../VanDerPolOscillatorAPI/controlAlgorithm.cpp"
#include "../VanDerPolOscillatorAPI/displayData.cpp"

using namespace Microsoft::VisualStudio::CppUnitTestFramework;

namespace VanDerPolOscillatorTests
{
	TEST_CLASS(VanDerPolOscillatorTests)
	{
	public:
		
		TEST_METHOD(TestEnumerators)
		{
			Logger::WriteMessage("In TestEnumerators");
			vdpo::algorithm testValue = vdpo::algorithm::FD;
		}
		//-------------------------------------------------------------------------------------------//
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
			Assert::AreEqual(2.0, k, L"(Should fail) FAILED - k is not equal to 1.0", LINE_INFO());
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
		//-------------------------------------------------------------------------------------------//
		TEST_METHOD(TestGnuPlotVectors1)
		{
			Logger::WriteMessage("In TestGnuPlotVectors");
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
			Logger::WriteMessage("In TestGnuPlotVectors");
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
		//-------------------------------------------------------------------------------------------//
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
		TEST_METHOD(TestFDControlAlgorithm)
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
			Assert::AreEqual(21,testFD.simulationIterations, L"FAILED - (should not fail) time iterations are not correctly  calculated");

			testFD.setMaxIterations(100);

			testFD.thetaVar[0] = theta[0];
			testFD.thetaVar[1] = theta[1];
			Assert::AreEqual(theta[0], testFD.thetaVar[0], L"FAILED - theta0 is not setted correctly");
			Assert::AreEqual(theta[1], testFD.thetaVar[1], L"FAILED - theta1 is not setted correctly");
			//Assert::AreEqual(theta[2], testFD.thetaVar[2], L"FAILED - (should fail) theta2 is not setted correctly");
			testFD.setDtheta(0.5);
			testFD.setHetta(1);

			testFD.setSystemModel(testModel);
			testFD.runAlgorithm();
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

			Assert::AreEqual(P, testFD.P, L"FAILED - Should not fail - Performance is not correct");
		}
	};
}
