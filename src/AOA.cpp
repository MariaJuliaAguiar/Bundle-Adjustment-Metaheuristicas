//includes
#include "AOA.hpp"

AOA::AOA(Benchmark *benchmark, unsigned int searchAgentsCount, unsigned int maximumIterations, std::vector<int>ind_val, std::vector<double> media_inter, std::vector<double> melhor_inter, std::vector<double> MOA, double max_MOA, double min_MOA, std::vector<double> MOP, double alpha_MOP, double u, double **positions_inicial, std::string pasta)
{
	benchmark_m = benchmark;
	searchAgentsCount_m = searchAgentsCount;
	maximumIterations_m = maximumIterations;

	ind_val_m = ind_val;
	pasta_m = pasta;
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
	epsilon = 0.000001;
	//Initialize the positions of search agents
	positions_m = Utils::inicialization(searchAgentsCount_m, dimension_m, positions_inicial);
	best_solind = Utils::Create1DArray(searchAgentsCount);
	x_score = Utils::Create1DArray(searchAgentsCount);
	best_score = std::numeric_limits<double >::infinity();
	Best_pos = Utils::Create1DZeroArray(dimension_m);// Inicialização da melhor posição encontrada

}

AOA::~AOA() {
	for (register unsigned int agentId = 0; agentId < searchAgentsCount_m; agentId++) {
		delete positions_m[agentId];
	}
	delete positions_m;
	delete Best_pos;
	delete x_score;
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
void AOA::calculateFitness(std::vector<std::vector<std::vector<cv::KeyPoint>>> bestKey, std::vector<std::string> imagens_src, cv::Mat im360, int rows, int cols, std::vector<std::vector<int>> indices,  int it, double MOA, double MOP)
{

	double fitness;

//#pragma omp parallel for 
	for (register int agentIndex = 0; agentIndex < searchAgentsCount_m; agentIndex++)
	{
		std::vector<double> r1(dimension_m, 0), r2(dimension_m, 0), r3(dimension_m, 0);

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
					positions_m[agentIndex][variable] = Best_pos[variable] / ((MOP + epsilon)*((boundaries_m[variable].upperBound - boundaries_m[variable].lowerBound)*u_m + boundaries_m[variable].lowerBound));

				}
				else
				{
					positions_m[agentIndex][variable] = Best_pos[variable] * MOP *((boundaries_m[variable].upperBound - boundaries_m[variable].lowerBound)*u_m + boundaries_m[variable].lowerBound);
				}

			}
			else
			{
				if (r3[variable] > 0.5)
				{
					positions_m[agentIndex][variable] = Best_pos[variable] - MOP * ((boundaries_m[variable].upperBound - boundaries_m[variable].lowerBound)*u_m + boundaries_m[variable].lowerBound);
				}
				else
				{
					positions_m[agentIndex][variable] = Best_pos[variable] + MOP * ((boundaries_m[variable].upperBound - boundaries_m[variable].lowerBound)*u_m + boundaries_m[variable].lowerBound);
				}

			}



		}
		// ---------------- - Verificação dos Limites do Espaço de Busca / Avaliação------------------

		Utils::Clip1DArray(positions_m[agentIndex], dimension_m, boundaries_m);

		//Calculate objective function for  new bat

		fitness = benchmark_m->fitness(positions_m[agentIndex], bestKey, imagens_src, im360, rows, cols, indices);
		x_score[agentIndex] = fitness;

		//Etapa de busca global
		/*accept candidate solution as current solution if better or if not better accept solution with some small probability*/

		if (abs(fitness) < abs(best_solind[agentIndex]))
		{
			best_solind[agentIndex] = fitness;
			//Best_posind[agentIndex] = positions_m[agentIndex];
			std::copy(&positions_m[agentIndex][0], &positions_m[agentIndex][dimension_m], &best_posind[agentIndex][0]);
			if (abs(best_solind[agentIndex]) < abs(best_score))
			{
				best_score = best_solind[agentIndex];
				std::copy(&best_posind[agentIndex][0], &best_posind[agentIndex][dimension_m], &Best_pos[0]);
				

			}

		}

	}

}
double AOA::mean(double arr[], int n)
{
	double mean = 0; // initialize sum

	// Iterate through all elements
	// and add them to sum
	for (int i = 0; i < n; i++) {
		mean += arr[i];
	}
		

	return mean / n;
}
double* AOA::Evaluate(bool debug, std::vector<std::vector<std::vector<cv::KeyPoint>>> bestKey, std::vector<std::string> imagens_src, cv::Mat im360, std::vector<std::vector<int>> indices) {
	cv::Mat image1 = cv::imread(imagens_src[0]);
	// A avalia a população inicial
	fitness_inicial(bestKey, imagens_src, im360, image1.rows, image1.cols, indices);
	
	  
	//melhor posição e melhor  fitness
	for (int a = 0; a < searchAgentsCount_m; a++)
	{
		if (x_score[a] < best_score)
		{
			best_score = x_score[a];
			std::copy(&positions_m[a][0], &positions_m[a][dimension_m], &Best_pos[0]);

		}
		
	}
	best_posind = Utils::inicialization(searchAgentsCount_m, dimension_m, positions_m);
	//best_solind = x_score;
	std::copy(&x_score[0], &x_score[searchAgentsCount_m], &best_solind[0]);

	

	/*auto start_time = std::chrono::high_resolution_clock::now();*/

	for (register int iteration = 0; iteration < maximumIterations_m; iteration++) {
	
		
		media_inter_m[iteration] = mean(x_score, searchAgentsCount_m);
		melhor_inter_m[iteration] = best_score;
		MOA_m[iteration] = min_MOA_m + iteration * ((max_MOA_m - min_MOA_m) / maximumIterations_m);
		MOP_m[iteration] = 1 - (pow(iteration, (1 / alpha_MOP_m)) / pow(maximumIterations_m, (1 / alpha_MOP_m)));
		
		calculateFitness(bestKey, imagens_src, im360, image1.rows, image1.cols, indices,  iteration, MOA_m[iteration], MOP_m[iteration]);

		convergenceCurve_m[iteration] = best_score;
	

		/*if (debug && (iteration % 1 == 0)) {
			std::cout << "At iteration " << iteration << " the best fitness is "
				<< std::setprecision(3)
				<< best_score << std::endl;
		}*/

	}

	/*auto finish_time = std::chrono::high_resolution_clock::now();
	executionTime_m = std::chrono::duration_cast<std::chrono::nanoseconds>(finish_time - start_time).count() * 1e-9;*/

	return convergenceCurve_m;
}
double* AOA::GetBestPositionAOA() {
	return Best_pos;
}
double AOA::GetBestScore() {
	return best_score;
}

std::ostream& operator << (std::ostream& os, const AOA *aoa) {

	//Salvar resultados em arquivos de textos
	std::string path = aoa->pasta_m;

	//Melhores soluções de cada simulação
	std::fstream bests_sol;
	bests_sol.open(path + "bests_sol_AOA.txt", std::fstream::app);

	//os << std::scientific << std::setprecision(9) <<"AOA position = ";

	for (register unsigned int variable = 0; variable < aoa->dimension_m; variable++) {
		//os << aoa->Best_positions_m[variable] << " ";
		bests_sol << aoa->Best_pos[variable] << " ";
	}
	bests_sol << "\n";
	bests_sol.close();

	//melhores fitness em cada simulação
	std::fstream bests_fit;
	bests_fit.open(path + "best_fit_AOA.txt", std::fstream::app);

	bests_fit << aoa->best_score << " ";
	bests_fit << "\n";
	bests_fit.close();


	/*os 	<< "  AOA score (Fitness) = " << aoa->best_score << std::endl
		<< "  Time = " << aoa->executionTime_m << " seconds";*/

	// Salvar valores de convergencia
	std::fstream conv;
	conv.open(path + "convergencia_AOA.txt", std::fstream::app);
	for (int j = 0; j < aoa->maximumIterations_m; j++) {
		conv << aoa->convergenceCurve_m[j] << " ";

	}
	conv << "\n";
	conv.close();

	return os;
}
