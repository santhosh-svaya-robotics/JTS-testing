#pragma once
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cstdlib>

#include <Eigen/Dense>
#ifndef FORWARDKINEMATICS_H
#define FORWARDKINEMATICS_H
using transformationMatrix = std::vector<std::vector<double>>;

std::vector<Eigen::Matrix4d> forwardKinematics(const Eigen::VectorXd theta,const double L[5]) ;
#endif // FORWARDKINEMATICS_H
