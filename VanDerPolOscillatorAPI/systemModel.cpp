#include "systemModel.h"

double  vdpo::systemModel::getSysParK()			{ return k; }
double  vdpo::systemModel::getSysParM()			{ return m; }
double  vdpo::systemModel::getSysParC()			{ return c; }

//-----------------------------------------------------------//
// Please fix this section to return either pointers or arrays

int* vdpo::systemModel::getSSV()			{ return x; }
int* vdpo::systemModel::getTheta2()			{ return thetaVar; }
int* vdpo::systemModel::getTheta3()			{ return thetaVar; }
int* vdpo::systemModel::getDx()				{ return dx; }
//-----------------------------------------------------------//

vdpo::theta vdpo::systemModel::getThetaNumber() { return numTheta; }

void	vdpo::systemModel::setSysParK(double setValue) { k = setValue; }
void	vdpo::systemModel::setSysParM(double setValue) { m = setValue; }
void	vdpo::systemModel::setSysParC(double setValue) { c = setValue; }
void	vdpo::systemModel::setSysParKMC(double setValues[3])
{
	k = setValues[0];
	m = setValues[1];
	c = setValues[2];
}
void vdpo::systemModel::setSSV(double setValues[2])
{
	this->x[0] = setValues[0];
	this->x[1] = setValues[1];
}
void vdpo::systemModel::setTheta2(double setValues[2])
{
	this->thetaVar[0] = setValues[0];
	this->thetaVar[1] = setValues[1];
	this->thetaVar[2] = 0;
}
void vdpo::systemModel::setTheta3(double setValues[3])
{
	this->thetaVar[0] = setValues[0];
	this->thetaVar[1] = setValues[1];
	this->thetaVar[2] = setValues[2];
}
void vdpo::systemModel::setThetaNumber1(theta setValue)
{
	numTheta = setValue;
}
void vdpo::systemModel::setThetaNumber2(int setValue = 2)
{
	numTheta = (setValue == 2 ? theta::two : theta::three);
}

vdpo::systemModel::systemModel()
{
	this->k = 1;
	this->m = 1;
	this->c = 1;
	this->numTheta = theta::two;
	this->thetaVar[0] = 0;
	this->thetaVar[1] = 0;
	this->thetaVar[2] = 0;
}

vdpo::systemModel::systemModel(double kk, double mm, double cc, theta numOfTheta)
{
	this->k = kk;
	this->m = mm;
	this->c = cc;
	this->numTheta = numOfTheta;
	this->thetaVar[0] = 0;
	this->thetaVar[1] = 0;
	this->thetaVar[2] = 0;
}
vdpo::systemModel::systemModel(double systemParameters[3], theta numOfTheta)
{
	this->k = systemParameters[0];
	this->m = systemParameters[1];
	this->c = systemParameters[2];
	this->numTheta = numOfTheta;
	this->thetaVar[0] = 0;
	this->thetaVar[1] = 0;
	this->thetaVar[2] = 0;
}

void vdpo::systemModel::dxCalculate()
{
	vdpo::u uSignal;
	double uLocal = (this->numTheta == theta::two ? uSignal.u2() : (this->numTheta == theta::three ? uSignal.u3() : 0.0));
	this->dx[0] = this->x[1];
	this->dx[1] = (-(this->c / this->m) * (std::pow(this->x[0], 2) - 1) * this->x[1] - (this->k / this->m) * this->x[0] + (uLocal / this->m));
}
vdpo::systemModel::~systemModel() { }

vdpo::u::u(theta numTheta)
{
	this->thetaVar[2] = (numTheta == theta::two ? 0 : thetaVar[2]);
}
double vdpo::u::u2()
{
	return this->thetaVar[0] * this->x[0] + this->thetaVar[1] * this->x[1];
}
double vdpo::u::u3()
{
	return this->thetaVar[0] * this->x[0] + this->thetaVar[1] * this->x[1] - thetaVar[2] * this->x[1] * (std::pow(this->x[0], 2) - 1);
}
vdpo::u::~u() { }


