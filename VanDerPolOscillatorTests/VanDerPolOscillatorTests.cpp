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
		TEST_METHOD(TestGnuPlotSetVectors)
		{
			Logger::WriteMessage("In TestGnuPlotSetVectors");
			vdpo::controlAlgorithm testAlgo = vdpo::controlAlgorithm::controlAlgorithm();
			vdpo::displayData testData = vdpo::displayData::displayData("Test", "testSetA", "testSetB", "dataFile.txt");
			//
			Logger::WriteMessage("Test completed");
		}
		//-------------------------------------------------------------------------------------------//
	};
}
