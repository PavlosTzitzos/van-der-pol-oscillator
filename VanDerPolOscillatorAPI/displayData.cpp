#include "displayData.h"
#include <array>
#include <vector>

vdpo::displayData::displayData()
{
	this->graphTitle = "default";
	this->label1 = "label1";
	this->label2 = "label2";
	this->filename = "deault.txt";
	this->vector1.push_back(0.0);
	this->vector2.push_back(0.0);
}

vdpo::displayData::displayData(std::string title, std::string labelA, std::string labelB, std::string resultsFileName)
{
	this->graphTitle = title;
	this->label1 = labelA;
	this->label2 = labelB;
	this->filename = resultsFileName;
	this->vector1.push_back(0.0);
	this->vector2.push_back(0.0);
}

void vdpo::displayData::setVector1(double v)
{
	this->vector1.push_back(v);
}

void vdpo::displayData::setVector2(double v)
{
	this->vector2.push_back(v);
}

template<int N>
void vdpo::displayData::setVector1(double v[N])
{
	for (int i=0;i<N;i++)
		vector1.push_back(v[i]);
}

template<int N>
void vdpo::displayData::setVector2(double v[N])
{
	// Another way to implement the logic of:
	// "take the elements of v and push them in the vector"
	// is to use the insert method :
	this->vector1.insert(std::end(v), std::begin(v), std::end(v));
}

void vdpo::displayData::setVector1(std::vector<double> v)
{
	// Same thing as setVector2 but using vector methods
	// The above uses std methods which are more generic and allow to handle the array
	this->vector1.insert(v.end(), v.begin(), v.end());
}

void vdpo::displayData::setVector2(std::vector<double> v)
{
	this->vector1.insert(v.end(), v.begin(), v.end());
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

void vdpo::displayData::consoleWrite1()
{
	// Using indexing to print the last element seperately
	// The result should be like: 
	// 
	// v = [ 1.1 , 4.5 , 6 , 8.0 , 45 , 7 , 5.33 , 3 , 8 , 4 ]
	// 
	// There might be other more efficient ways to do the same thing
	// Suggested fixes for that are welcomed.

	int i = 0;
	std::cout << "v1 = [ ";
	for (i = 0; i < vector1.size()-1; ++i)
		std::cout << this->vector1[i] << " , ";
	std::cout << this->vector1[i+1] << " ] " << std::endl;
}

void vdpo::displayData::consoleWrite2()
{
	// Using indexing to print the last element seperately
	// The result should be like: 
	// 
	// v = [ 1.1 , 4.5 , 6 , 8.0 , 45 , 7 , 5.33 , 3 , 8 , 4 ]
	// 
	// There might be other more efficient ways to do the same thing
	// Suggested fixes for that are welcomed.

	int i = 0;
	std::cout << "v2 = [ ";
	for (i = 0; i < vector2.size() - 1; ++i)
		std::cout << this->vector2[i] << " , ";
	std::cout << this->vector2[i + 1] << " ] " << std::endl;
}

void vdpo::FDParameters::parameters()
{
	std::cout << "The parameters of the FD algorithm are:" << std::endl;
	std::cout << "h " << std::endl;
	std::cout << "dtheta " << std::endl;
}

void vdpo::displayData::plotData1()
{
	Gnuplot gp("\"C:\\Program Files\\gnuplot\\bin\\gnuplot.exe\""); // DO NOT TOUCH THIS
	
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
	Gnuplot gp("\"C:\\Program Files\\gnuplot\\bin\\gnuplot.exe\""); // DO NOT TOUCH THIS

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
	results << "\n" << this->label1 << " " << this->label2 << std::endl;
	results << "-----------------------------------------" << std::endl;
	if (this->vector1.size() > this->vector2.size())
	{
		for (int i = 0; i < this->vector1.size(); i++)
		{
			if(i >= this->vector2.size())
				results << "\n" << this->vector1[i] << " " << "0.0" << std::endl;
			else
				results << "\n" << this->vector1[i] << " " << this->vector2[i] << std::endl;
		}
	}
	else if (this->vector1.size() < this->vector2.size())
	{
		for (int i = 0; i < this->vector2.size(); i++)
		{
			if (i >= this->vector1.size())
				results << "\n" << "0.0" << " " << this->vector2[i] << std::endl;
			else
				results << "\n" << this->vector1[i] << " " << this->vector2[i] << std::endl;
		}
	}
	else 
	{
		for (int i = 0; i < this->vector2.size(); i++)
		{
			results << "\n" << this->vector1[i] << " " << this->vector2[i] << std::endl;
		}
	}
}

