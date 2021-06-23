#include<PSO.hpp>

PSO::PSO(Benchmark *benchmark, unsigned int searchAgentsCount,
	unsigned int maximumIterations, std::vector<int>ind_val, double **positions_inicial, std::string pasta, double Vmax, double wMax, double wMin, double c1, double c2)
{
	benchmark_m = benchmark;
	searchAgentsCount_m = searchAgentsCount;
	maximumIterations_m = maximumIterations;

	ind_val_m = ind_val;
	pasta_m = pasta;
	boundaries_m = benchmark_m->getBoundaries();
	dimension_m = benchmark_m->GetDimension();
	convergenceCurve_m = Utils::Create1DZeroArray(maximumIterations_m);
	//parametros do pso
	 Vmax_m = Vmax;
	 wMax_m = wMax;
	 wMin_m = wMin;
	 c1_m = c1;
	 c2_m = c2;

	//Initialize alpha, beta, and delta_pos
	Best_pos = Utils::Create1DZeroArray(dimension_m);// Inicialização da melhor posição encontrada
	Best_score = std::numeric_limits<double>::infinity();//% Inicialização do melhor score encontrado


	//Initialize the positions of search agents
	positions_m = Utils::inicialization(searchAgentsCount_m, dimension_m, positions_inicial);
	vel_m = Utils::Create2DZeroArray(searchAgentsCount, benchmark_m->GetDimension());//%Inicialização do vetor de velocidades das particulas
	pBest_score = Utils::Create1DArray(searchAgentsCount); //Inicialização do histórico de melhor score por particula
	pBest = Utils::Create2DZeroArray(searchAgentsCount, benchmark_m->GetDimension()); // Inicialização do histórico de melhor Posição por particula
}

PSO::~PSO() {
	for (register unsigned int agentId = 0; agentId < searchAgentsCount_m; agentId++) {
		delete positions_m[agentId];
	}
	delete positions_m;
	delete Best_pos;

	delete convergenceCurve_m;
}
void PSO::calculateFitness(std::vector<std::vector<std::vector<cv::KeyPoint>>> bestKey, std::vector<std::string> imagens_src, cv::Mat im360, int rows, int cols, std::vector<std::vector<int>> indices) {
	double fitness;

	//#pragma omp parallel for
	for (register int agentIndex = 0; agentIndex < searchAgentsCount_m; agentIndex++) {
		//Adequa aos limites
		Utils::Clip1DArray(positions_m[agentIndex], dimension_m, boundaries_m);

		//Calculate objective function for each search agent

		fitness = benchmark_m->fitness(positions_m[agentIndex], bestKey, imagens_src, im360, rows, cols, indices);

		//Update Alpha, Beta, and Delta
		if (fitness < pBest_score[agentIndex])
		{
			pBest_score[agentIndex] = fitness;
			std::copy(&positions_m[agentIndex][0], &positions_m[agentIndex][dimension_m], &pBest[agentIndex][0]);
		}


		if (Best_score > fitness)
		{
			Best_score = fitness;
			std::copy(&positions_m[agentIndex][0], &positions_m[agentIndex][dimension_m], &Best_pos[0]);

		}
	}

}
void PSO::update(double w) {

	//#pragma omp parallel for
	for (register int agentIndex = 0; agentIndex < searchAgentsCount_m; agentIndex++)
	{
		for (register unsigned int variable = 0; variable < dimension_m; variable++) {
			double r1 = Utils::GenerateRandomNumber(); //r1 is a random number in [0,1]
			double r2 = Utils::GenerateRandomNumber(); //r2 is a random number in [0,1]

			vel_m[agentIndex][variable] = w * vel_m[agentIndex][variable] + c1_m * r1*(pBest[agentIndex][variable] - positions_m[agentIndex][variable]) + c2_m * r2*(Best_pos[variable] - positions_m[agentIndex][variable]);

			if (vel_m[agentIndex][variable] > Vmax_m)
			{
				vel_m[agentIndex][variable] = Vmax_m;
			}

			if (vel_m[agentIndex][variable] < -Vmax_m)
			{
				vel_m[agentIndex][variable] = -Vmax_m;
			}
			positions_m[agentIndex][variable] = positions_m[agentIndex][variable] + vel_m[agentIndex][variable];



		}

	}

}
double* PSO::Evaluate(bool debug, std::vector<std::vector<std::vector<cv::KeyPoint>>> bestKey, std::vector<std::string> imagens_src, cv::Mat im360, std::vector<std::vector<int>> indices) {
	double w;
	cv::Mat image1 = cv::imread(imagens_src[0]);

	//auto start_time = std::chrono::high_resolution_clock::now();
	//#pragma omp parallel for
	for (register int iteration = 0; iteration < maximumIterations_m; iteration++) {

		calculateFitness(bestKey, imagens_src, im360, image1.rows, image1.cols, indices);

		//a decreases linearly from 2 to 0
		w = wMax_m - iteration * ((wMax_m - wMin_m) / maximumIterations_m);

		update(w);
		convergenceCurve_m[iteration] = Best_score;

		/*	if (debug && (iteration % 1 == 0)) {
				std::cout << "At iteration " << iteration << " the best fitness is "
					<< std::setprecision(3)
					<< alphaScore_m << std::endl;
			}*/

	}

	/*uto finish_time = std::chrono::high_resolution_clock::now();
	executionTime_m = std::chrono::duration_cast<std::chrono::nanoseconds>(finish_time - start_time).count() * 1e-9;*/

	return convergenceCurve_m;
}
double* PSO::GetBestPositionPSO() {
	return Best_pos;
}

double PSO::GetBestScorePSO() {
	return Best_score;
}

double PSO::GetExecutionTime() {
	return executionTime_m;
}
std::ostream& operator << (std::ostream& os, const PSO *pso) {

	//Salvar resultados em arquivos de textos
	std::string path = pso->pasta_m;

	//Melhores soluções de cada simulação
	std::fstream bests_sol;
	bests_sol.open(path + "bests_sol_PSO.txt", std::fstream::app);


	//os << std::scientific << std::setprecision(9)<< "pso position = ";

	for (register unsigned int variable = 0; variable < pso->dimension_m; variable++) {
		/*os << pso->alphaPosition_m[variable] << " ";*/
		bests_sol << pso->Best_pos[variable] << " ";
	}
	bests_sol << "\n";
	bests_sol.close();
	/*os << "  pso score (Fitness) = " << pso->Best_score << std::endl
		<< "  Time = " << pso->executionTime_m << " seconds";*/

	//melhores fitness em cada simulação
	std::fstream bests_fit;
	bests_fit.open(path + "best_fit_PSO.txt", std::fstream::app);

	bests_fit << pso->Best_score << " ";
	bests_fit << "\n";
	bests_fit.close();

	//Salvar valores de convergencia 
	std::fstream conv;
	conv.open(path + "convergencia_PSO.txt", std::fstream::app);
	for (int j = 0; j < pso->maximumIterations_m; j++) {
		conv << pso->convergenceCurve_m[j] << " ";

	}
	conv << "\n";
	conv.close();

	return os;
}