#pragma once
#ifndef ErrorCorrection_H

#define ErrorCorrection_H

#include<stdio.h>
#include<iostream>
#include"Decode.h"
#include"Coor.h"
#include"CONST.h"


void _LinearCombination(OBS& obs);
double _Hopefield(double& H, double& E);
void _RotationCorrection(XYZ& x, double& omega_e, double& dt);

#endif // !ErrorCorrection_H
