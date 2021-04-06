//includes
#include "AOA.hpp"
#include <math.h>
AOA::AOA(Benchmark *benchmark, unsigned int searchAgentsCount, unsigned int maximumIterations, std::vector<int>ind_val, std::vector<double> media_inter, std::vector<double> melhor_inter, std::vector<double> MOA, double max_MOA, double min_MOA, std::vector<double> MOP, double alpha_MOP, double u, double **positions_inicial)
{
	benchmark_m = benchmark;
	searchAgentsCount_m = searchAgentsCount;
	maximumIterations_m = maximumIterations;

	ind_val_m = ind_val;

	boundaries_m = benchmark_m->getBoundaries();
	dimension_m = benchmark_m->GetDimension();
	convergenceCurve_m = Utils::Create1DZeroArray(maximumIterations_m);

	media_inter_m = media_inter;
	melhor_inter_m = melhor_inter;
	MOA_m = MOA;
	max_MOA_m = max_MOA;
	min_MOA_m = min_MOA;
	MOP_m = MOP;
	alpha_MOP_m = alpha_MOP;
	u_m = u;

	//Initialize the positions of search agents
	positions_m = positions_inicial;

	x_score = Utils::Create1DArray(searchAgentsCount);
	best_score = std::numeric_limits<double >::infinity();

}

AOA::~AOA() {
	for (register unsigned int agentId = 0; agentId < searchAgentsCount_m; agentId++) {
		delete positions_m[agentId];
	}

	delete convergenceCurve_m;
}
void AOA::fitness_inicial(std::vector<std::vector<std::vector<cv::KeyPoint>>> bestKey, std::vector<std::string> imagens_src, cv::Mat im360, int rows, int cols, std::vector<std::vector<int>> indices) {
	double fitness;

#pragma omp parallel for
	for (register int agentIndex = 0; agentIndex < searchAgentsCount_m; agentIndex++) {
		Utils::Clip1DArray(positions_m[agentIndex], dimension_m, boundaries_m);

		//Calculate objective function for each search agent

		fitness = benchmark_m->fitness(positions_m[agentIndex], bestKey, imagens_src, im360, rows, cols, indices);

		//Update Alpha, Beta, and Delta
		if (fitness < x_score[agentIndex]) {
			x_score[agentIndex] = fitness; // Update alpha
			//std::copy(&positions_m[agentIndex][0], &positions_m[agentIndex][dimension_m], &alphaPosition_m[0]);
		}


	}

}
void AOA::calculateFitness(std::vector<std::vector<std::vector<cv::KeyPoint>>> bestKey, std::vector<std::string> imagens_src, cv::Mat im360, int rows, int cols, std::vector<std::vector<int>> indices, double *best_pos, double best_score, int it, double MOA, double MOP)
{
	srand(time(NULL));
	double fitness;

#pragma omp parallel for //morceguinhos
	for (register int agentIndex = 0; agentIndex < searchAgentsCount_m; agentIndex++)
	{
		std::vector<double> r1(dimension_m,0), r2(dimension_m, 0), r3(dimension_m, 0);
	
		std::generate(r1.begin(), r1.end(), Utils::GenerateRandomNumber);
		std::generate(r2.begin(), r2.end(), Utils::GenerateRandomNumber);
		std::generate(r3.begin(), r3.end(), Utils::GenerateRandomNumber);
		for (register unsigned int variable = 0; variable < dimension_m; variable++)
		{
			if (r1[variable] > MOA)
			{
				//BUSCA GLOBAL
				if (r2[variable] > 0.5)
				{
					positions_m[agentIndex][variable] = best_pos[variable] / ((MOP + epsilon)*((boundaries_m[variable].upperBound - boundaries_m[variable].lowerBound)*u_m + boundaries_m[variable].lowerBound));

				}
				else
				{
					positions_m[agentIndex][variable] = best_pos[variable] * MOP *((boundaries_m[variable].upperBound - boundaries_m[variable].lowerBound)*u_m + boundaries_m[variable].lowerBound);
				}

			}
			else
			{
				if (r3[variable] > 0.5)
				{
					positions_m[agentIndex][variable] = best_pos[variable] - MOP * ((boundaries_m[variable].upperBound - boundaries_m[variable].lowerBound)*u_m + boundaries_m[variable].lowerBound);
				}
				else
				{
					positions_m[agentIndex][variable] = best_pos[variable] + MOP * ((boundaries_m[variable].upperBound - boundaries_m[variable].lowerBound)*u_m + boundaries_m[variable].lowerBound);
				}

			}



		}
		// ---------------- - Verificação dos Limites do Espaço de Busca / Avaliação------------------

		Utils::Clip1DArray(positions_m[agentIndex], dimension_m, boundaries_m);

		//Calculate objective function for  new bat

		fitness = benchmark_m->fitness(positions_m[agentIndex], bestKey, imagens_src, im360, rows, cols, indices);


		//Etapa de busca global
		/*accept candidate solution as current solution if better or if not better accept solution with some small probability*/

		if (abs(fitness) < abs(best_solind[agentIndex]))
		{
			best_solind[agentIndex] = fitness;
			best_posind[agentIndex] = positions_m[agentIndex];
			if (abs(best_solind[agentIndex]) < abs(best_score))
			{
				best_score = best_solind[agentIndex];
				best_pos = best_posind[agentIndex];
			}

		}

	}

}

double* AOA::Evaluate(bool debug, std::vector<std::vector<std::vector<cv::KeyPoint>>> bestKey, std::vector<std::string> imagens_src, cv::Mat im360, std::vector<std::vector<int>> indices) {
	cv::Mat image1 = cv::imread(imagens_src[0]);
	// A avalia a população inicial
	fitness_inicial(bestKey, imagens_src, im360, image1.rows, image1.cols, indices);
	int n = sizeof(x_score) / sizeof(x_score[0]);

	double *best_pos;

	double mean_x_score = 0;
	//melhor posição e melhor  fitness
	for (int a = 0; a < n; a++)
	{
		if (x_score[a] < best_score)
		{
			best_score = x_score[a];
			best_pos = positions_m[a];
		}
		mean_x_score += x_score[a];
	}
	best_posind = positions_m;
	best_solind = x_score;

	mean_x_score = mean_x_score / n;

	auto start_time = std::chrono::high_resolution_clock::now();
#pragma omp parallel for
	for (register int iteration = 0; iteration < maximumIterations_m; iteration++) {


		media_inter_m[iteration] = mean_x_score;
		melhor_inter_m[iteration] = best_score;
		MOA_m[iteration] = min_MOA_m + iteration * ((max_MOA_m - min_MOA_m) / maximumIterations_m);
		MOP_m[iteration] = 1 - (pow(iteration, (1 / alpha_MOP_m)) / pow(maximumIterations_m, (1 / alpha_MOP_m)));

		calculateFitness(bestKey, imagens_src, im360, image1.rows, image1.cols, indices, best_pos, best_score, iteration, MOA_m[iteration], MOP_m[iteration]);
		
		convergenceCurve_m[iteration] = best_score;
		best_positions_m = best_pos;
		
		if (debug && (iteration % 1 == 0)) {
			std::cout << "At iteration " << iteration << " the best fitness is "
				<< std::setprecision(3)
				<< best_score << std::endl;
		}

	}

	auto finish_time = std::chrono::high_resolution_clock::now();
	executionTime_m = std::chrono::duration_cast<std::chrono::nanoseconds>(finish_time - start_time).count() * 1e-9;

	return convergenceCurve_m;
}
std::ostream& operator << (std::ostream& os, const AOA *aoa) {
	os << std::scientific
		<< std::setprecision(9)
		<< "Benchmark: " << aoa->benchmark_m->GetName() << std::endl
		<< "AOA position = ";

	for (register unsigned int variable = 0; variable < aoa->dimension_m; variable++) {
		os << aoa->best_positions_m[variable] << " ";
	}

	os << std::endl
		<< "Alpha score (Fitness) = " << aoa->best_score << std::endl
		<< "Time = " << aoa->executionTime_m << " seconds";



	return os;
}
