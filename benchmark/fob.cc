//  fob.cc
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

#include <cmath> //pow
#include "benchmark/fob.hpp"
#include "utils.hpp"
#include <iostream>
#include <string>
#include <math.h>
#include <sys/stat.h>
#include <ostream>
#include <chrono>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <dirent.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/mat.hpp>
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d.hpp"
#include <opencv2/stitching/detail/blenders.hpp>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

Benchmark* fob::Create(std::vector<double> lb, std::vector<double> up) {

	return new fob( lb, up);
}

fob::fob( std::vector<double> lb, std::vector<double> up) : Benchmark() {
	variablesCount_m = (lb.size() );
	//Limits of variable values
	boundaries_m = new Boundaries[variablesCount_m];
	for (register unsigned int variable = 0; variable < variablesCount_m; variable++)
	{
		boundaries_m[variable].lowerBound = lb[variable], boundaries_m[variable].upperBound = up[variable];

	}

	Benchmark::setName("fob");
	Benchmark::setDimension(variablesCount_m);
	Benchmark::setBoundaries(boundaries_m);
}

fob::~fob() {
	delete boundaries_m;
}



//fob mais nova - Vinicius
double fob::fitness(double x[], std::vector<std::vector<std::vector<cv::KeyPoint>>> bestKey, std::vector<std::string> imagens_src, cv::Mat im360, int rows, int cols, std::vector<std::vector<int>> indices) {

	//auto start_time = std::chrono::high_resolution_clock::now();

	//double erro = 0;
	std::vector<double> erro;
	erro.resize(bestKey.size());
	
	int w = 3599;
	int h = 1799;
	
	double avg = 0;
	std::vector<Eigen::Vector2d> errors;
	
	//#pragma omp parallel for 
	for (int frame0 = 0; frame0 < bestKey.size(); frame0++)
	{

		erro[frame0] = 0;
		int l = 0;

		for (int j = 0; j < bestKey[frame0].size(); j++)
		{
			std::vector<cv::KeyPoint> kpts1 = bestKey[frame0][j];
			std::vector<cv::KeyPoint> kpts2 = bestKey[frame0][j + 1];
			int frame1 = indices[frame0][l];
			if (bestKey[frame0][j].size() > 0)
			{
				//#pragma omp parallel for 
				for (int k = 0; k < kpts1.size(); k++)
				{


					Eigen::Matrix3d  Ki, Kj;
					double fx1 = x[(frame0 * 6) + 2];
					double fy1 = x[(frame0 * 6) + 3];
					double cx1 = x[(frame0 * 6) + 4];
					double cy1 = x[(frame0 * 6) + 5];
					Ki << fx1, 0, cx1,
						0, fy1, cy1,
						0, 0, 1;
					double fx2 = x[(frame1 * 6) + 2];
					double fy2 = x[(frame1 * 6) + 3];
					double cx2 = x[(frame1 * 6) + 4];
					double cy2 = x[(frame1 * 6) + 5];
					Kj << fx2, 0, cx2,
						0, fy2, cy2,
						0, 0, 1;
					cv::KeyPoint kp1 = kpts1[k];
					cv::KeyPoint kp2 = kpts2[k];
			
				
					
					double pan = x[(frame0 * 6) + 1];
					double tilt = x[frame0 * 6];

					Eigen::Matrix3d Ri;
					Ri = Eigen::AngleAxisd(-DEG2RAD(pan), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(-DEG2RAD(tilt), Eigen::Vector3d::UnitX());
					

					Eigen::Matrix3d Rj;

					
					double panj = x[(frame1 * 6) + 1];
					double tiltj = x[frame1 * 6];
					Rj = Eigen::AngleAxisd(-DEG2RAD(panj), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(-DEG2RAD(tiltj), Eigen::Vector3d::UnitX());


					//Matrz de Homografia

					Eigen::Matrix3d H;
					H = Ki * Ri.transpose()*Rj * (Kj).inverse();


					// Ponto no frustrum 3D correspondente a feature na imagem 1 em 2D
					Eigen::Vector3d pij, p, pe1, uki;
					p << kp2.pt.x, kp2.pt.y, 1;
					

					pij = H * p; // Ponto em 3D do plano da imagem
					pij(0) = (pij(0) / pij(2)) + (w / 2);
					pij(1) = (pij(1) / pij(2)) + (h / 2);
					pij(2) = (pij(2) / pij(2));

					for (int i = 0; i < 3; i++)
					{
						if (pij(i) < 0) {
							pij(i) = 0;
						}
						

					}
					if (pij(1) >= 1799)
					{
						pij(1) = 1798;
					}
					if (pij(0) >= 3599)
					{
						pij(0) = 3598;
					}


					//Imagem de referencia
					Eigen::Matrix3d HI;
					HI = Ki * Ri.transpose()*Ri * (Kj).inverse();;


					pe1 << kp1.pt.x, kp1.pt.y, 1;


					//pe1.normalize();

					uki = HI * (pe1);

					uki(0) = (uki(0) / uki(2)) + (w / 2);
					uki(1) = (uki(1) / uki(2)) + (h / 2);
					uki(2) = (uki(2) / uki(2));
					for (int i = 0; i < 3; i++)
					{
						if (uki(i) < 0) {
							uki(i) = 0;
						}
					

					}
					if (uki(1) >= 1799)
					{
						uki(1) = 1798;
					}
					if (uki(0) >= 3599)
					{
						uki(0) = 3598;
					}


					
					///// RESULTADO FINAL, para ir formando a FOB, com o somatorio do erro entre os pontos
					Eigen::Vector3d e = uki - pij;

				
					avg += pow(e.norm(), 2);
				
				}
			}
			l++;
			j++;
		}

	}
	double erroT;
	
	erroT = avg;
	//auto finish_time = std::chrono::high_resolution_clock::now();
	//auto time = std::chrono::duration_cast<std::chrono::nanoseconds>(finish_time - start_time).count() * 1e-9;
	//std::cout <<"fob "<< time<<"\n";
	return erroT;
}