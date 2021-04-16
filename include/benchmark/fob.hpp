//  F1.hpp
//
//  Author:
//       Ahmad Dajani <eng.adajani@gmail.com>
//
//  Copyright (c) 2020 Ahmad Dajani
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "benchmark.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>
class fob : public Benchmark {
private:
	static std::vector<int> i;
	 int variablesCount_m = 246;//dimension
	Boundaries *boundaries_m;
	


public:
	fob(std::vector<double> lb, std::vector<double> up);
	~fob();
	double fitness(double x[], std::vector<std::vector<std::vector<cv::KeyPoint>>> bestKey, std::vector<std::string> imagens_src, cv::Mat im360, int rows, int cols, std::vector<std::vector<int>> indices);
	static Benchmark *Create( std::vector<double> lb, std::vector<double> up);
};
