#ifndef __SSA_H
#define __SSA_H
#include "utils.hpp"
#include "benchmark.hpp"

class SSA {

private:
	double executionTime_m = 0;
	Benchmark *benchmark_m;
	unsigned int searchAgentsCount_m;
	unsigned int maximumIterations_m;
	Boundaries *boundaries_m;
	unsigned int dimension_m;

	std::vector<int>ind_val_m;

	std::string pasta_m;
	

	double *x_score;
	double *best_pos;
	//Initialize the positions of search agents
	double **positions_m;
	double *convergenceCurve_m;
	double best_score;


	void calculateFitness(std::vector<std::vector<std::vector<cv::KeyPoint>>> bestKey, std::vector<std::string> imagens_src, cv::Mat im360, int rows, int cols, std::vector<std::vector<int>> indices,  int it, double c1);
	void fitness_inicial(std::vector<std::vector<std::vector<cv::KeyPoint>>> bestKey, std::vector<std::string> imagens_src, cv::Mat im360, int rows, int cols, std::vector<std::vector<int>> indices);


public:
	SSA::SSA(Benchmark *benchmark, unsigned int searchAgentsCount, unsigned int maximumIterations, std::vector<int>ind_val,   double **positions_inicial, std::string pasta);
	~SSA();
	double *Evaluate(bool debug, std::vector<std::vector<std::vector<cv::KeyPoint>>> bestKey, std::vector<std::string> imagens_src, cv::Mat im360, std::vector<std::vector<int>> indices);
	double *GetBestPositionSSA();
	double GetBestScore();
	/*double *GetBetaPosition();
	double GetBetaScore();
	double *GetDeltaPosition();
	double GetDeltaScore();
	double GetExecutionTime();*/
	friend std::ostream& operator << (std::ostream& os, const SSA *ssa);
};
#endif