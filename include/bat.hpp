
#ifndef __BAT_H
#define __BAT_H
#include "utils.hpp"
#include "benchmark.hpp"
#include <cstdlib> //atexit
#include <iostream> //cerr, cout
class BAT {

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
	//Inicialização da ampplitude e taxa de emissao
	std::vector<double> A_m;
	std::vector<double> r_m;
	double ** vel_m;

	double lambda_m;
	double alpha_m;
	double gama_m;
	double fmax_m;// frequencia maxima
	double fmin_m; //frequencia minima
	double A0_m;//amplitude sonora inicial
	double rf_m;//taxa de emissao 

	//Initialize the positions of search agents
	double **positions_m;
	double *best_positions_m;
	double *x_score;
	double *convergenceCurve_m;
	
	double best_score ;


	void calculateFitness(std::vector<std::vector<std::vector<cv::KeyPoint>>> bestKey, std::vector<std::string> imagens_src, cv::Mat im360, int rows, int cols, std::vector<std::vector<int>> indices, double *best_pos, double best_score, int it);
	void fitness_inicial(std::vector<std::vector<std::vector<cv::KeyPoint>>> bestKey, std::vector<std::string> imagens_src, cv::Mat im360, int rows, int cols, std::vector<std::vector<int>> indices);
	/*void updateWolves(double a);*/

public:
	BAT::BAT(Benchmark *benchmark, unsigned int searchAgentsCount, unsigned int maximumIterations, std::vector<int>ind_val_m, std::vector<double> amp_sonora, std::vector<double> taxa, double lambda, double alpha, double gama, double fmax, double fmin, double A0, double rf, double **positions_inicial, std::string pasta);
	~BAT();
	double *Evaluate(bool debug, std::vector<std::vector<std::vector<cv::KeyPoint>>> bestKey, std::vector<std::string> imagens_src, cv::Mat im360, std::vector<std::vector<int>> indices);
	/*double *GetAlphaPosition();
	double GetAlphaScore();
	double *GetBetaPosition();
	double GetBetaScore();
	double *GetDeltaPosition();
	double GetDeltaScore();
	double GetExecutionTime();*/
	friend std::ostream& operator << (std::ostream& os, const BAT *bat);
};
#endif
