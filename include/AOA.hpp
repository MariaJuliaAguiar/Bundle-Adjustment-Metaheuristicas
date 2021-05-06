#ifndef __AOA_H
#define __AOA_H
#include "utils.hpp"
#include "benchmark.hpp"

class AOA {

private:
	double executionTime_m = 0;
	Benchmark *benchmark_m;
	unsigned int searchAgentsCount_m;
	unsigned int maximumIterations_m;
	Boundaries *boundaries_m;
	unsigned int dimension_m;
	
	std::vector<int>ind_val_m;
	std::string pasta_m;

	std::vector<double> media_inter_m;
	std::vector<double> melhor_inter_m;
	std::vector<double> MOA_m;
	double max_MOA_m;
	double min_MOA_m;
	std::vector<double> MOP_m;
	double alpha_MOP_m;

	double u_m;
	
	double *x_score;
	double epsilon;
	//Initialize the positions of search agents
	double **positions_m;
	double *convergenceCurve_m;
	double best_score;
	double **best_posind;
	double *best_solind;
	double *best_positions_m;
	double *Best_pos;

	void calculateFitness(std::vector<std::vector<std::vector<cv::KeyPoint>>> bestKey, std::vector<std::string> imagens_src, cv::Mat im360, int rows, int cols, std::vector<std::vector<int>> indices, int it,double MOA, double MOP);
	void fitness_inicial(std::vector<std::vector<std::vector<cv::KeyPoint>>> bestKey, std::vector<std::string> imagens_src, cv::Mat im360, int rows, int cols, std::vector<std::vector<int>> indices);


public:
	AOA::AOA(Benchmark *benchmark, unsigned int searchAgentsCount, unsigned int maximumIterations, std::vector<int>ind_val, std::vector<double> media_inter, std::vector<double> melhor_inter, std::vector<double> MOA, double max_MOA, double min_MOA, std::vector<double> MOP, double alpha_MOP, double u, double **positions_inicial, std::string pasta);
	~AOA();
	double *Evaluate(bool debug, std::vector<std::vector<std::vector<cv::KeyPoint>>> bestKey, std::vector<std::string> imagens_src, cv::Mat im360, std::vector<std::vector<int>> indices);
	double mean(double arr[], int n);
	double *GetBestPositionAOA();
	double GetBestScore();
	/*double *GetBetaPosition();
	double GetBetaScore();
	double *GetDeltaPosition();
	double GetDeltaScore();
	double GetExecutionTime();*/
	friend std::ostream& operator << (std::ostream& os, const AOA *aoa);
};
#endif
