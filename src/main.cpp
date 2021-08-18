//includes meus 

#include "argument.hpp"
#include "utils.hpp"
#include "gwo.hpp"
#include "bat.hpp"
#include "AOA.hpp"
#include "ssa.hpp"
#include "PSO.hpp"
//Para utilizar as blibliotecs do open cv

#include <windows.h>
#include "opencv2/opencv.hpp"

#include "GWOException.hpp"

/// Definicoes e namespaces

using namespace cv;
using namespace std;
using namespace cv::xfeatures2d;
using namespace pcl;
using namespace pcl::io;

Argument *argument = nullptr;
GWO *gwo = nullptr;
BAT *bat = nullptr;
AOA *aoa = nullptr;
SSA *ssa = nullptr;
PSO *pso = nullptr;


void freeMemory() {
	if (argument) {
		delete argument;
		argument = nullptr;
	}

}
void freeMemoryOti() {

	if (gwo) {
		delete gwo;
		gwo = nullptr;
	}
	if (bat) {
		delete bat;
		bat = nullptr;
	}
	if (aoa) {
		delete aoa;
		aoa = nullptr;
	}
	if (ssa) {
		delete ssa;
		ssa = nullptr;
	}
	if (pso) {
		delete pso;
		pso = nullptr;
	}
	
}
int main() {
	//Localização do arquivo NVM/SFM com posição das imagens de acordo com o PEPO
	std::string pasta = "C:/Users/julia/Desktop/dataset/scan36/";

	//Encontrando posição originais das imagens (Posição do PEPO)
	double fx, fy, cx, cy;
	std::vector<std::string> imagens_src;
	std::vector<std::vector<float> > pose = Utils::lerSFM(pasta, fx, fy, cx, cy, imagens_src);//roll, pitch, yaw, fx, fy, cx e cy

	//Encontrandos os limites maximos e minimos dos parâmetros 
	std::vector<double> lb, up;
	Utils::uplowerBound(pose, fx, fy, cx, cy, lb, up);//definir lower and upper bounds

	vector<int>  ind_val;

	ind_val.resize(imagens_src.size());
	std::iota(std::begin(ind_val), std::end(ind_val), 0);
	//Encontrando imagens vizinhas de acordo com a ordem que foram tiradas 

	std::vector<vector<int>>indices_vizinhos = Utils::FindVizinhos(imagens_src.size());
	
	// Features Matching - Keypoints e descriptors
	vector<vector<cv::KeyPoint>>  kpts_src;
	vector<cv::Mat>  descp_src;
	descp_src.resize(imagens_src.size());
	kpts_src.resize(imagens_src.size());


	std::cout << "Calculando features e matches" << std::endl;
	//Encontrando features
	Utils::calcular_features_sift(descp_src, kpts_src, imagens_src);

	//Encontrando os matches 

	// Ajustar matriz de quantidade de matches
	Eigen::MatrixXi matches_count = Eigen::MatrixXi::Zero(descp_src.size() - 1, descp_src.size() - 1);
	vector<vector<  vector<cv::DMatch> >> matriz_matches(indices_vizinhos.size());
	for (int i = 0; i < matriz_matches.size(); i++)
		matriz_matches.at(i).resize(indices_vizinhos[i].size());

	std::vector<std::vector<std::vector<cv::KeyPoint>>> bestKey = Utils::sift_matches_matrix_encontrar_melhor(matriz_matches, descp_src, kpts_src, imagens_src, indices_vizinhos);
	float step_deg = 0.1; // [DEGREES]//resolução
	int raios_360 = int(360.0 / step_deg), raios_180 = raios_360 / 2.0; // Quantos raios sairao do centro para formar 360 e 180 graus de um circulo 2D

	//Size of final panoramic
	cv::Mat im360 = cv::Mat::zeros(cv::Size(raios_360, raios_180), CV_8UC3); // Imagem 360 ao final de todas as fotos passadas sem blending 

	Utils::clearResultstxt(pasta);//limpar arquivos de simulações anteriores dessa pasta;
	std::cout << "Inicializando simulacoes" << std::endl;

	//************************************************
	//Parâmetros de simulação
	int searchAgentsCount_m = 35;// numero de agentes
	int dimension_m = indices_vizinhos.size() * 6;// dimensão 
	int iterations = 1000; // número de iterações
	int simulations = 1;//quantidade de simulações
	
	//Melhores soluções	
	double **Best_pos_PSO = new double *[simulations];
	double *Best_sol_PSO = new double[simulations];
	double **Best_pos_SSA = new double *[simulations];
	double *Best_sol_SSA = new double[simulations];
	double **Best_pos_AOA = new double *[simulations];
	double *Best_sol_AOA = new double[simulations];
	double **Best_pos_BAT = new double *[simulations];
	double *Best_sol_BAT = new double[simulations];
	double **Best_pos_GWO = new double *[simulations];
	double *Best_sol_GWO = new double[simulations];

	//tempos de cada meaheurística
	std::vector<double> time_gwo, time_bat, time_aoa, time_ssa, time_pso;
	time_gwo.resize(simulations);	time_bat.resize(simulations);	time_aoa.resize(simulations);	time_ssa.resize(simulations);	time_pso.resize(simulations);
	
	
	for (int a = 0; a < simulations; a++)
	{


		////////////// Parametros de simulação ////////////////////////////////
	 // Inicialização das varivaeis para otimzação

		double **positions_inicial = Utils::Create2DRandomArray(searchAgentsCount_m, dimension_m, lb, up);// posição inicial dos agentes 


		//**************************** GWO ****************************
		std::cout << endl << "GWO" << a << endl;


		auto start_time_GWO = std::chrono::high_resolution_clock::now();

		atexit(freeMemory);

		argument = new Argument(searchAgentsCount_m, iterations, lb, up);

		argument->Parse();

		gwo = new GWO(argument->GetBenchmark(), searchAgentsCount_m, iterations, ind_val, positions_inicial, pasta);

		(void)gwo->Evaluate(true, bestKey, imagens_src, im360, indices_vizinhos);
		std::cout /*<< " Result " << a << std::endl*/
			<< gwo << std::endl << std::endl;
		std::cout << "";

		Best_pos_GWO[a] = gwo->GetAlphaPosition();//melhor posição
		Best_sol_GWO[a] = gwo->GetAlphaScore();//melor sitness


		auto finish_time_gwo = std::chrono::high_resolution_clock::now();
		time_gwo[a] = std::chrono::duration_cast<std::chrono::nanoseconds>(finish_time_gwo - start_time_GWO).count() * 1e-9;
		

		freeMemory();


		////**************************** BAT ****************************
		//parâmetros especificos do Bat

		std::vector<double> taxa(searchAgentsCount_m, 0.5);//taxa de emissão dos pulsos
		std::vector<double> amp_sonora(searchAgentsCount_m, Utils::GenerateRandomNumber()); // amplitude sonora
		double lambda = 0.01;
		double alpha = 0.9995;
		double gama = 0.0015;
		double fmax = 100;// frequencia maxima
		double fmin = 0; //frequencia minima
		double A0 = 1;//amplitude sonora inicial
		double rf = 1;//taxa de emissao 


		std::cout << endl << "BAT" << a << endl;
		auto start_time_bat = std::chrono::high_resolution_clock::now();
		atexit(freeMemory);
		argument = new Argument(searchAgentsCount_m, iterations, lb, up);
		argument->Parse();
		bat = new BAT(argument->GetBenchmark(), searchAgentsCount_m, iterations, ind_val, amp_sonora, taxa, lambda, alpha, gama, fmax, fmin, A0, rf, positions_inicial, pasta);
		(void)bat->Evaluate(true, bestKey, imagens_src, im360, indices_vizinhos);
		bat;
		std::cout /*<< " Result " << a << std::endl*/
			<< bat << std::endl << std::endl;
		std::cout << "";

		Best_pos_BAT[a] = bat->GetBestPositionBAT();
		Best_sol_BAT[a] = bat->GetBestScore();
		auto finish_time_Bat = std::chrono::high_resolution_clock::now();
		time_bat[a] = std::chrono::duration_cast<std::chrono::nanoseconds>(finish_time_Bat - start_time_bat).count() * 1e-9;


		freeMemory();


		//////**************************** AOA ****************************

		std::vector<double> media_inter(iterations, 0.0); // Vetor média por interações;
		std::vector<double> melhor_inter(iterations, 0.0); // Vetor melhor solução por interação;
		std::vector<double> MOA(iterations, 0.0); //Math Optimizer Accelerated;
		double max_MOA = 2, min_MOA = 0.1; // Valor máximo e minimo da função MOA;
		std::vector<double> MOP(iterations, 0.0); // Math Optimizer Probability;
		double alpha_MOP = 5; //Parâmetro sensível e define a precisão da exploração nas iterações; (Valor - Artigo original);
		double u = 0.49999; // Parâmetro de controle para ajustar o processo de busca; (Valor 0.5 - Artigo original)


		std::cout << endl << "AOA" << a << endl;
		auto start_time_aoa = std::chrono::high_resolution_clock::now();
		atexit(freeMemory);
		argument = new Argument(searchAgentsCount_m, iterations, lb, up);
		argument->Parse();
		aoa = new AOA(argument->GetBenchmark(), searchAgentsCount_m, iterations, ind_val, media_inter, melhor_inter, MOA, max_MOA, min_MOA, MOP, alpha_MOP, u, positions_inicial, pasta);
		(void)aoa->Evaluate(true, bestKey, imagens_src, im360, indices_vizinhos);
		std::cout /*<< " Result " << a << std::endl*/
			<< aoa << std::endl << std::endl;
		std::cout << "";
		Best_pos_AOA[a] = aoa->GetBestPositionAOA();
		Best_sol_AOA[a] = aoa->GetBestScore();
		auto finish_time_aoa = std::chrono::high_resolution_clock::now();
		time_aoa[a] = std::chrono::duration_cast<std::chrono::nanoseconds>(finish_time_aoa - start_time_aoa).count() * 1e-9;
		freeMemory();

		//////**************************** SSA ****************************

		std::cout << endl << "SSA" << a << endl;


		auto start_time_ssa = std::chrono::high_resolution_clock::now();
		atexit(freeMemory);
		argument = new Argument(searchAgentsCount_m, iterations, lb, up);
		argument->Parse();
		ssa = new SSA(argument->GetBenchmark(), searchAgentsCount_m, iterations, ind_val, positions_inicial, pasta);
		(void)ssa->Evaluate(true, bestKey, imagens_src, im360, indices_vizinhos);
		std::cout /*<< " Result " << a << std::endl*/
			<< ssa << std::endl << std::endl;
		std::cout << "";
		Best_pos_SSA[a] = ssa->GetBestPositionSSA();
		Best_sol_SSA[a] = ssa->GetBestScore();
		auto finish_time_ssa = std::chrono::high_resolution_clock::now();
		time_ssa[a] = std::chrono::duration_cast<std::chrono::nanoseconds>(finish_time_ssa - start_time_ssa).count() * 1e-9;
		freeMemory();
		//////**************************** pso ****************************

		std::cout << endl << "PSO" << a << endl;
		//parâmetros do PSO
		double Vmax = 6;
		double Wmax = 0.9;
		double Wmin = 0.2;
		double c1 = 2;
		double c2 = 2;


		auto start_time_pso = std::chrono::high_resolution_clock::now();
		atexit(freeMemory);
		argument = new Argument(searchAgentsCount_m, iterations, lb, up);
		argument->Parse();
		pso = new PSO(argument->GetBenchmark(), searchAgentsCount_m, iterations, ind_val, positions_inicial, pasta, Vmax, Wmax, Wmin, c1, c2);
		(void)pso->Evaluate(true, bestKey, imagens_src, im360, indices_vizinhos);
		std::cout/* << " Result " << a << std::endl*/
			<< pso << std::endl << std::endl;
		std::cout << "";
		Best_pos_PSO[a] = pso->GetBestPositionPSO();
		Best_sol_PSO[a] = pso->GetBestScorePSO();
		auto finish_time_pso = std::chrono::high_resolution_clock::now();
		time_pso[a] = std::chrono::duration_cast<std::chrono::nanoseconds>(finish_time_pso - start_time_pso).count() * 1e-9;
		
		for (register unsigned int agentId = 0; agentId < searchAgentsCount_m; agentId++) {
			delete positions_inicial[agentId];
		}
		
	}

	freeMemory();
	//salvar os tempos em arquivo txt
	std::ofstream ofs;
	ofs.open(pasta + "Tempo_GWO.txt", std::ofstream::out | std::ofstream::trunc);

	std::ofstream ofs1;
	ofs1.open(pasta + "Tempo_BAT.txt", std::ofstream::out | std::ofstream::trunc);

	std::ofstream ofs2;
	ofs2.open(pasta + "Tempo_AOA.txt", std::ofstream::out | std::ofstream::trunc);

	std::ofstream ofs3;
	ofs3.open(pasta + "Tempo_SSA.txt", std::ofstream::out | std::ofstream::trunc);

	std::ofstream ofs4;
	ofs4.open(pasta + "Tempo_PSO.txt", std::ofstream::out | std::ofstream::trunc);

	for (register unsigned int variable = 0; variable < simulations; variable++)
	{

		ofs  << time_gwo[variable] << "\n";
		ofs1 << time_bat[variable] << "\n";
		ofs2 << time_aoa[variable] << "\n";
		ofs3 << time_ssa[variable] << "\n";
		ofs4 << time_pso[variable] << "\n";

	}

	ofs.close();
	ofs1.close();
	ofs2.close();
	ofs3.close();
	ofs4.close();


	////**************************** Plotar Imagens ****************************



	//std::cout << "Gerando imagens panoramicas" << endl; - Com e Sem Blending
	//////Encontrar melhor solução de todas as simulações 
	std::cout << "GWO - ";
	auto it_gwo = std::min_element(Best_sol_GWO, Best_sol_GWO + simulations);
	int index_GWO = std::distance(Best_sol_GWO, it_gwo);
	std::vector<cv::Mat> im360_GWO = Utils::panoramicas(dimension_m, pasta, Best_pos_GWO[index_GWO]);
	cv::imwrite(pasta + "im360_GWO.png", im360_GWO[0]);
	cv::imwrite(pasta + "im360_GWO_blending.png", im360_GWO[1]);

	std::cout << "BAT - ";
	auto it_BAT = std::min_element(Best_sol_BAT, Best_sol_BAT + simulations);
	int index_BAT = std::distance(Best_sol_BAT, it_BAT);
	std::vector<cv::Mat> im360_BAT = Utils::panoramicas(dimension_m, pasta, Best_pos_BAT[index_BAT]);
	cv::imwrite(pasta + "im360_BAT.png", im360_BAT[0]);
	cv::imwrite(pasta + "im360_BAT_blending.png", im360_BAT[1]);

	std::cout << "AOA - ";
	auto it_AOA = std::min_element(Best_sol_AOA, Best_sol_AOA + simulations);
	int index_AOA = std::distance(Best_sol_AOA, it_AOA);
	std::vector<cv::Mat> im360_AOA = Utils::panoramicas(dimension_m, pasta, Best_pos_AOA[index_AOA]);
	cv::imwrite(pasta + "im360_AOA.png", im360_AOA[0]);
	cv::imwrite(pasta + "im360_AOA_blending.png", im360_AOA[1]);

	std::cout << "SSA - ";
	auto it_SSA = std::min_element(Best_sol_SSA, Best_sol_SSA + simulations);
	int index_SSA = std::distance(Best_sol_SSA, it_SSA);
	std::vector<cv::Mat> im360_SSA = Utils::panoramicas(dimension_m, pasta, Best_pos_SSA[index_SSA]);
	cv::imwrite(pasta + "im360_SSA.png", im360_SSA[0]);
	cv::imwrite(pasta + "im360_SSA_blending.png", im360_SSA[1]);

	freeMemoryOti();

	std::cout << "PSO - ";
	auto it_PSO = std::min_element(Best_sol_PSO, Best_sol_PSO + simulations);
	int index_PSO = std::distance(Best_sol_PSO, it_PSO);
	std::vector<cv::Mat> im360_PSO = Utils::panoramicas(dimension_m, pasta, Best_pos_PSO[index_PSO]);
	cv::imwrite(pasta + "im360_PSO.png", im360_PSO[0]);
	cv::imwrite(pasta + "im360_PSO_blending.png", im360_PSO[1]);

	freeMemoryOti();

	std::cout << " Processo Finalizado";
	return 0;
}