#include "displayData.h"

#include <vector>

vdpo::displayData::displayData()
{
	graphTitle = "default";
	label1 = "label1";
	label2 = "label2";
	filename = "deault.txt";
	vector1.push_back(0.0);
	vector1.push_back(0.0);
}

vdpo::displayData::displayData(std::string title, std::string labelA, std::string labelB, std::string resultsFileName)
{
	graphTitle = title;
	label1 = labelA;
	label2 = labelB;
	filename = resultsFileName;
	vector1.push_back(0.0);
	vector1.push_back(0.0);
}

void vdpo::displayData::thetaTypes()
{
	std::cout << "The number of theta parameters can be 2 or 3" << std::endl;
	std::cout << "Example for 2 parameters: " << std::endl;
	std::cout << "vdpo::theta::two " << std::endl;
}

void vdpo::displayData::availableAlgorithms()
{
	std::cout << "Currently available algorithms to control the van der pol system are:" << std::endl;
	std::cout << "FD   - Finite Differences" << std::endl;
	std::cout << "SPSA - Simultaneous Perturbation Stochastic Approximation" << std::endl;
	std::cout << "LQR  - Linear Quadratic Regulator using Riccati ODE" << std::endl;
	std::cout << "AC   - Indirect Adaptive Control" << std::endl;
}

void vdpo::displayData::systemParameters()
{
	std::cout << "The system has 3 constant (over time) which are:" << std::endl;
	std::cout << "m = " << this->m << std::endl;
	std::cout << "c = " << this->c << std::endl;
	std::cout << "k = " << this->k << std::endl;
}

template<typename T>
void vdpo::displayData::printData(T var)
{
	std::cout << var << std::endl;
}

void vdpo::FDParameters::parameters()
{
	std::cout << "The parameters of the FD algorithm are:" << std::endl;
	std::cout << "h " << std::endl;
	std::cout << "dtheta " << std::endl;
}

void vdpo::displayData::plotData1()
{
	Gnuplot gp("\"C:\\Program Files\\gnuplot\\bin\\gnuplot.exe\"");
	
	gp << "set title '" << this->graphTitle << "'\n";
	if (!this->vector1.empty())
	{
		gp << "plot '-' with lines title '" << this->label1 << "'\n";
		gp.send(this->vector1);
	}
	else
	{
		gp << "plot '-' with lines title '" << this->label2 << "'\n";
		gp.send(this->vector2);
	}
	std::cin.get();
}

void vdpo::displayData::plotData2()
{
	Gnuplot gp("\"C:\\Program Files\\gnuplot\\bin\\gnuplot.exe\"");

	gp << "set title '" << this->graphTitle << "'\n";
	gp << "plot '-' with lines title '" << this->label1 << "',"
		<< "'-' with lines title '" << this-> label2 << "'\n";
	gp.send(this->vector1);
	gp.send(this->vector2);

	std::cin.get();
}
void vdpo::displayData::exportData()
{
	std::ofstream results;
	results.open(this->filename);
	results << "\n" << this->label1 << "," << this->label2 << std::endl;
	results << "-----------------------------------------" << std::endl;
	if (vector1.size() > vector2.size())
	{
		for (int i = 0; i < vector1.size(); i++)
		{
			if(i > vector2.size())
				results << "\n" << this->vector1[i] << "0" << std::endl;
			else
				results << "\n" << this->vector1[i] << this->vector2[i] << std::endl;
		}
	}
	else if (vector1.size() < vector2.size())
	{
		for (int i = 0; i < vector2.size(); i++)
		{
			if (i > vector1.size())
				results << "\n" << "0" << this->vector2[i] << std::endl;
			else
				results << "\n" << this->vector1[i] << this->vector2[i] << std::endl;
		}
	}
	else 
	{
		for (int i = 0; i < vector2.size(); i++)
		{
			results << "\n" << this->vector1[i] << this->vector2[i] << std::endl;
		}
	}
}

