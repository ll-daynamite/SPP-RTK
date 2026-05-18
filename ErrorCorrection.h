#pragma once
#include"Decode.h"
#include<cmath>
#include"Position.h"
using namespace std;
double hopfield(const double H, const double Elev);//对流层改正函数 
void DetectOutlier(EPOCHOBSDATA* Obs);//粗差探测函数