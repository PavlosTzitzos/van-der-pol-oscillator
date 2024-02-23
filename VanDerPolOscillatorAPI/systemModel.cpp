#include "systemModel.h"

double  vdpo::systemModel::getSysParK()			{ return this->k; }
double  vdpo::systemModel::getSysParM()			{ return this->m; }
double  vdpo::systemModel::getSysParC()			{ return this->c; }

double* vdpo::systemModel::getSSV()				{ return this->x; }
double* vdpo::systemModel::getTheta2()			{ return this->thetaVar; }
double* vdpo::systemModel::getTheta3()			{ return this->thetaVar; }
double* vdpo::systemModel::getDx()				{ return this->dx; }

vdpo::theta vdpo::systemModel::getThetaNumber() { return this->numTheta; }

void vdpo::systemModel::setSysParK(double setValue) { this->k = setValue; }
void vdpo::systemModel::setSysParM(double setValue) { this->m = setValue; }
void vdpo::systemModel::setSysParC(double setValue) { this->c = setValue; }
void vdpo::systemModel::setSysParKMC(double setValues[3])
{
	this->k = setValues[0];
	this->m = setValues[1];
	this->c = setValues[2];
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
	this->numTheta = setValue;
}
void vdpo::systemModel::setThetaNumber2(int setValue)
{
	this->numTheta = (setValue == 2 ? theta::two : theta::three);
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
	double uLocal = (this->numTheta == theta::two ? this->u2() : (this->numTheta == theta::three ? this->u3() : this->uK()));
	this->dx[0] = this->x[1];
	this->dx[1] = -(this->c / this->m) * (this->x[0] * this->x[0] - 1) * this->x[1] - (this->k / this->m) * this->x[0] + (uLocal / this->m);

	this->uTest = uLocal; // for debugging
}
//vdpo::systemModel::~systemModel() { }

double vdpo::systemModel::u2()
{
	return this->thetaVar[0] * this->x[0] + this->thetaVar[1] * this->x[1];
}
double vdpo::systemModel::u3()
{
	return this->thetaVar[0] * this->x[0] + this->thetaVar[1] * this->x[1] - thetaVar[2] * this->x[1] * (this->x[0] * this->x[0] - 1);
}
double vdpo::systemModel::uK()
{
	return this->K[0] * this->x[0] + this->K[1] * this->x[1];
}