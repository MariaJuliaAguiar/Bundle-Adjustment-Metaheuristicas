//  gwo.hpp
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

#ifndef __PSO_H
#define __PSO_H
#include "utils.hpp"
#include "benchmark.hpp"

class PSO {

private:
	double executionTime_m = 0;
	Benchmark *benchmark_m;
	unsigned int searchAgentsCount_m;
	unsigned int maximumIterations_m;
	Boundaries *boundaries_m;
	unsigned int dimension_m;
	std::vector<int>ind_vazios_m;
	std::vector<int>ind_val_m;
	std::string pasta_m;
	//Parametros do PSO
	double Vmax_m;
	double wMax_m;
	double wMin_m ;
	double c1_m;
	double c2_m;
	//initialize alpha, beta, and delta_pos
	double *Best_pos;
	double Best_score;
	
	
	

	//Initialize the positions of search agents
	double **positions_m;
	double *convergenceCurve_m;
	double **vel_m;
	double *pBest_score;
	double **pBest;

	void calculateFitness(std::vector<std::vector<std::vector<cv::KeyPoint>>> bestKey, std::vector<std::string> imagens_src, cv::Mat im360, int rows, int cols, std::vector<std::vector<int>> indices);
	void update(double w);

public:
	PSO(Benchmark *benchmark, unsigned int searchAgentsCount, unsigned int maximumIterations, std::vector<int>ind_val_m, double **positions_inicial, std::string pasta, double Vmax,double wMax,double wMin,	double c1,	double c2);
	~PSO();
	double *Evaluate(bool debug, std::vector<std::vector<std::vector<cv::KeyPoint>>> bestKey, std::vector<std::string> imagens_src, cv::Mat im360, std::vector<std::vector<int>> indices);
	double *GetBestPositionPSO();
	double GetBestScorePSO();
	
	double GetExecutionTime();
	friend std::ostream& operator << (std::ostream& os, const PSO *pso);
};
#endif
