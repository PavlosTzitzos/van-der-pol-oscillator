#include "bigNum.h"


vdpo::bigNum::bigNum()
{
}
vdpo::bigNum::bigNum(double UnhandledbigNumber)
{
	bigNumber = UnhandledbigNumber;
	length = std::numeric_limits<double>::max();
	this->breakNumber();
}
vdpo::bigNum::bigNum(std::vector<int> UnhandledbigNumber)
{
	this->breakNumber();
}

vdpo::bigNum::~bigNum()
{
	//
}

// Operators

vdpo::bigNum vdpo::operator+ (vdpo::bigNum& a, vdpo::bigNum& b)
{
	// Check if there is enough space in the vector
	if (a.UpperHalf.size() + a.LowHalf.size() > a.length)
		std::cout << "Possible loss of data due to storage limitations" << std::endl;
	if (b.UpperHalf.size() + b.LowHalf.size() > b.length)
		std::cout << "Possible loss of data due to storage limitations" << std::endl;


	// Add Low Half
	int lth = (a.LowHalf.size() > b.LowHalf.size()) ? a.LowHalf.size() : b.LowHalf.size();
	double res = 0;
	int hold = 0;

	vdpo::bigNum ret = vdpo::bigNum::bigNum();
	
	for (int i = lth; i >= 0; i--)
	{
		if(hold==0)
			res = a.LowHalf[i] + b.LowHalf[i];
		else
		{
			res = a.LowHalf[i] + b.LowHalf[i] + 10;
			hold = 0;
		}

		if (res > 9)
		{
			hold = 1;
			res = res - 10;
		}
		ret.LowHalf[i] = res;
	}

	// Add Upper Half

	// the hold variable is not re-initialized because we need the last value
	int lth = (a.UpperHalf.size() > b.UpperHalf.size()) ? a.UpperHalf.size() : b.UpperHalf.size();
	for (int i = 0; i < lth; i++)
	{
		if (hold == 0)
			res = a.UpperHalf[i] + b.UpperHalf[i];
		else
		{
			res = a.UpperHalf[i] + b.UpperHalf[i] + 10;
			hold = 0;
		}

		if (res > 9)
		{
			hold = 1;
			res = res - 10;
		}
		ret.UpperHalf[i] = res;
	}
}
