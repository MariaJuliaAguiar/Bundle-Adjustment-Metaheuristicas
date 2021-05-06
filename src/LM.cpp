#include "LM.hpp"
void LM::calculateFitness(std::vector<std::vector<std::vector<cv::KeyPoint>>> bestKey, std::vector<std::string> imagens_src, cv::Mat im360, int rows, int cols, std::vector<std::vector<int>> indices) {
	double fitness;


		

		//Calculate objective function for each search agent

		fitness = benchmark_m->fitness(positions_m[agentIndex], bestKey, imagens_src, im360, rows, cols, indices);


}

double* LM::Evaluate(bool debug, std::vector<std::vector<std::vector<cv::KeyPoint>>> bestKey, std::vector<std::string> imagens_src, cv::Mat im360, std::vector<std::vector<int>> indices) {
	double a;
	cv::Mat image1 = cv::imread(imagens_src[0]);

	auto start_time = std::chrono::high_resolution_clock::now();
	//#pragma omp parallel for
	for (register int iteration = 0; iteration < 1000; iteration++) {

		calculateFitness(bestKey, imagens_src, im360, image1.rows, image1.cols, indices);

		//a decreases linearly from 2 to 0
		/*a = 2.0 - iteration * (2.0 / maximumIterations_m);

		updateWolves(a);
		convergenceCurve_m[iteration] = alphaScore_m;*/

		/*	if (debug && (iteration % 1 == 0)) {
				std::cout << "At iteration " << iteration << " the best fitness is "
					<< std::setprecision(3)
					<< alphaScore_m << std::endl;
			}*/

	}

	/*auto finish_time = std::chrono::high_resolution_clock::now();
	executionTime_m = std::chrono::duration_cast<std::chrono::nanoseconds>(finish_time - start_time).count() * 1e-9;*/

	return convergenceCurve_m;
}
