//includes
#include "bat.hpp"


BAT::BAT(Benchmark *benchmark, unsigned int searchAgentsCount, unsigned int maximumIterations, std::vector<int>ind_val, std::vector<double> amp_sonora, std::vector<double> taxa, double lambda,
	double alpha, double gama, double fmax, double fmin, double A0, double rf, double **positions_inicial, std::string pasta)
{
	benchmark_m = benchmark;
	searchAgentsCount_m = searchAgentsCount;
	maximumIterations_m = maximumIterations;

	ind_val_m = ind_val;

	boundaries_m = benchmark_m->getBoundaries();
	dimension_m = benchmark_m->GetDimension();
	convergenceCurve_m = Utils::Create1DZeroArray(maximumIterations_m);
	pasta_m = pasta;
	//Inicialização da ampplitude e taxa de emissao
	A_m = amp_sonora;
	r_m = taxa;

	lambda_m = lambda;
	alpha_m = alpha;
	gama_m = gama;
	fmax_m = fmax;// frequencia maxima
	fmin_m = fmin; //frequencia minima
	A0_m = A0; //amplitude sonora inicial
	rf_m = rf; //taxa de emissa

   //Initialize the positions of search agents
	positions_m = Utils::inicialization(searchAgentsCount_m, dimension_m, positions_inicial);
	vel_m = Utils::Create2DZeroArray(searchAgentsCount, benchmark_m->GetDimension());
	x_score = Utils::Create1DArray(searchAgentsCount);
	best_score = std::numeric_limits<double >::infinity();


}

BAT::~BAT() {
	for (register unsigned int agentId = 0; agentId < searchAgentsCount_m; agentId++) {
		delete positions_m[agentId];
		delete vel_m[agentId];
	}
	delete positions_m;
	delete vel_m;
	delete best_positions_m;
	delete x_score;
	delete convergenceCurve_m;

}
void BAT::calculateFitness(std::vector<std::vector<std::vector<cv::KeyPoint>>> bestKey, std::vector<std::string> imagens_src, cv::Mat im360, int rows, int cols, std::vector<std::vector<int>> indices, double *best_pos,  int it)
{
	double fitness;

//#pragma omp parallel for //morceguinhos
	for (register int agentIndex = 0; agentIndex < searchAgentsCount_m; agentIndex++)
	{
		double *positions_new = new double[dimension_m];
		double f = fmin_m + (fmax_m - fmin_m)*Utils::GenerateRandomNumber();//frequencia 
		for (register unsigned int variable = 0; variable < dimension_m; variable++) {


			vel_m[agentIndex][variable] = vel_m[agentIndex][variable] + (best_pos[variable] - positions_m[agentIndex][variable])*f;//velocidade

			positions_new[variable] = positions_m[agentIndex][variable] + vel_m[agentIndex][variable];

		}

		//etapa de busca local
		if (Utils::GenerateRandomNumber() <r_m[agentIndex])
		{
			for (register unsigned int variable = 0; variable < dimension_m; variable++) {
				double mean_amp = std::accumulate(A_m.begin(), A_m.end(), 0.0) / A_m.size();
				double rand = -1 + 2 * Utils::GenerateRandomNumber();
				positions_new[variable] = positions_new[variable] + (rand * mean_amp * lambda_m);
			}
		}
		// Verificar os limtes de busca 
		Utils::Clip1DArray(positions_new, dimension_m, boundaries_m);



		//Calculate objective function for  new bat

		fitness = benchmark_m->fitness(positions_new, bestKey, imagens_src, im360, rows, cols, indices);


		//Etapa de busca global
		/*accept candidate solution as current solution if better or if not better accept solution with some small probability*/

		if (fitness <= x_score[agentIndex] && Utils::GenerateRandomNumber() < A_m[agentIndex]) {
			positions_m[agentIndex] = positions_new;
			x_score[agentIndex] = fitness;
			r_m[agentIndex] = rf_m * (1 - exp(-gama_m * (it)));//atualiza a taxa de emissao de pulsos
		}

		A_m[agentIndex] = alpha_m * A_m[agentIndex];//atualiza a amplitude sonora

	}
	//Update Alpha, Beta, and Delta

	double best_score_new = std::numeric_limits<double >::infinity();
	double *best_pos_new;
	//melhor posição e melhor  fitness
	for (int a = 0; a < searchAgentsCount_m; a++) {
		if (x_score[a] < best_score_new) {
			best_score_new = x_score[a];
			best_pos_new = positions_m[a];
		}
	}
	best_pos = best_pos_new;
	best_score = best_score_new;
}

void BAT::fitness_inicial(std::vector<std::vector<std::vector<cv::KeyPoint>>> bestKey, std::vector<std::string> imagens_src, cv::Mat im360, int rows, int cols, std::vector<std::vector<int>> indices) {
	double fitness;

//#pragma omp parallel for
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

double* BAT::Evaluate(bool debug, std::vector<std::vector<std::vector<cv::KeyPoint>>> bestKey, std::vector<std::string> imagens_src, cv::Mat im360, std::vector<std::vector<int>> indices) {
	cv::Mat image1 = cv::imread(imagens_src[0]);
	// A avalia a população inicial
	fitness_inicial(bestKey, imagens_src, im360, image1.rows, image1.cols, indices);
	A_m[0] = A0_m;
	std::cout << "fit original" << x_score[0] << std::endl;
	double *best_pos;
	best_pos = new double[dimension_m];
	//melhor posição e melhor  fitness
	for (int a = 0; a < searchAgentsCount_m; a++)
	{
		if (x_score[a] < best_score)
		{
			best_score = x_score[a];
			//best_pos = positions_m[a];
			std::copy(&positions_m[a][0], &positions_m[a][dimension_m], &best_pos[0]);

		}
	}

	auto start_time = std::chrono::high_resolution_clock::now();
//#pragma omp parallel for
	for (register int iteration = 0; iteration < maximumIterations_m; iteration++) {

		calculateFitness(bestKey, imagens_src, im360, image1.rows, image1.cols, indices, best_pos, iteration);
		convergenceCurve_m[iteration] = best_score;
		best_positions_m = best_pos;
		/*if (debug && (iteration % 1 == 0)) {
			std::cout << "At iteration " << iteration << " the best fitness is "
				<< std::setprecision(3)
				<< best_score << std::endl;
		}*/

	}

	auto finish_time = std::chrono::high_resolution_clock::now();
	executionTime_m = std::chrono::duration_cast<std::chrono::nanoseconds>(finish_time - start_time).count() * 1e-9;

	return convergenceCurve_m;
}
double* BAT::GetBestPositionBAT() {
	return best_positions_m;
}
double BAT::GetBestScore() {
	return best_score;
}

std::ostream& operator << (std::ostream& os, const BAT *bat) {
	//Salvar resultados em arquivos de textos
	std::string path = bat->pasta_m;

	//os << std::scientific << std::setprecision(9) << "BAT position = ";

	//Melhores soluções de cada simulação
	std::fstream bests_sol;
	bests_sol.open(path + "bests_sol_BAT.txt", std::fstream::app);
	for (register unsigned int variable = 0; variable < bat->dimension_m; variable++) {
		//os << bat->best_positions_m[variable] << " ";
		bests_sol << bat->best_positions_m[variable] << " ";
	}
	bests_sol << "\n";
	bests_sol.close();
	//MELHOR FITNESS
	os 	<< "  BAT (Fitness) = " << bat->best_score << std::endl
		<< "  Time = " << bat->executionTime_m << " seconds";

	//melhores fitness em cada simulação
	std::fstream bests_fit;
	bests_fit.open(path + "best_fit_BAT.txt", std::fstream::app);

	bests_fit << bat->best_score << " ";
	bests_fit << "\n";
	bests_fit.close();

	//Salvar valores de convergencia 
	std::fstream conv;
	conv.open(path + "convergencia_BAT.txt", std::fstream::app);
	for (int j = 0; j < bat->maximumIterations_m; j++) {
		conv << bat->convergenceCurve_m[j] << " ";

	}
	conv << "\n";
	conv.close();
	return os;
}
