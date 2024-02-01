#include "pch.h"
#include "CppUnitTest.h"
#include "../VanDerPolOscillatorAPI/enumerators.cpp"
#include "../VanDerPolOscillatorAPI/systemModel.cpp"

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
	};
}
