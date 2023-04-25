#pragma once
#ifndef SatellitePostion_H

#define SatellitePostion_H

#include<iostream>
#include<stdio.h>
#include"Coor.h"
#include"CONST.h"
#include"Time.h"
#include"Decode.h"
#include"Matrix.h"


void _SatPos(const EPH* eph, OBS& obs);
void _SatPosVel(const EPH& eph, SATPOS& satpos, GPST& gt, double& tk, double& GM, double& omega_e);
void _SatClk(const EPH& eph, SATPOS& satpos, GPST& gt, double& tk, double& GM, double& omega_e);
double _DataValidity(const EPH& eph, const GPST& gt);
void _ParameterSetting(const NAVSYS& system, double& a, double& alpha, double& GM, double& omega_e);

void _RotationCorrection(XYZ& x, double& omega_e, double& dt);

#endif // !SatellitePostion_H
