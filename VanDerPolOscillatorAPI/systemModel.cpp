#include "systemModel.h"

double vdpo::systemModel::getSysParK() { return k; }
double vdpo::systemModel::getSysParM() { return m; }
double vdpo::systemModel::getSysParC() { return c; }
double* vdpo::systemModel::getSSV()	{ return x; }
double* vdpo::systemModel::getTheta2() { return thetaVar; }
double* vdpo::systemModel::getTheta3() { return thetaVar; }
vdpo::theta vdpo::systemModel::getThetaNumber() { return numTheta; }
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

vdpo::systemModel::systemModel(double kk, double mm, double cc, theta numOfTheta)
{
	this->k = kk;
	this->m = mm;
	this->c = cc;
	this->numTheta = numOfTheta;
}
vdpo::systemModel::systemModel(double systemParameters[3], theta numOfTheta)
{
	this->k = systemParameters[0];
	this->m = systemParameters[1];
	this->c = systemParameters[2];
	this->numTheta = numOfTheta;
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
void vdpo::systemModel::dxCalculate()
{
	vdpo::u uSignal;
	double uLocal = (this->numTheta == theta::two ? uSignal.u2() : (this->numTheta == theta::three ? uSignal.u3() : 0.0));
	this->dx[0] = this->x[1];
	this->dx[1] = (-(this->c / this->m) * (std::pow(this->x[0], 2) - 1) * this->x[1] - (this->k / this->m) * this->x[0] + (uLocal / this->m));
}

vdpo::u::~u() { }


