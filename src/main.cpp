//includes meus 
#define NOGDI
#define GLOG_NO_ABBREVIATED_SEVERITIES
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
#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "GWOException.hpp"
#include "bundle_adjutment.hpp"
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
	std::string pasta = "C:/Users/julia/Pictures/geradorartesspace/scan3/";

	//Encontrando posição originais das imagens (Posição do PEPO)
	double fx, fy, cx, cy;
	std::vector<std::string> imagens_src;
	std::vector<std::vector<float> > pose = Utils::lerSFM(pasta, fx, fy, cx, cy, imagens_src);

	//Enocntrandos os limites maximos e minimos dos parâmetros 
	std::vector<double> lb, up;
	Utils::uplowerBound(pose, fx, fy, cx, cy, lb, up);

	vector<int>  ind_val;

	ind_val.resize(imagens_src.size());
	std::iota(std::begin(ind_val), std::end(ind_val), 0);
	//Encontrando imagens vizinhas de acordo com a ordem que foram tiradas 

	std::vector<vector<int>>indices_vizinhos = Utils::FindVizinhos(imagens_src.size());
	/*
	std::vector<std::vector<int>>indices_vizinhos;
	indices_vizinhos.resize(1);
	indices_vizinhos[0].push_back(1);*/
	//indices_vizinhos[0].push_back(2);
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
	//std::vector<std::vector<std::vector<cv::KeyPoint>>> bestKey;

	std::vector<std::vector<std::vector<cv::KeyPoint>>> bestKey = Utils::sift_matches_matrix_encontrar_melhor(matriz_matches, descp_src, kpts_src, imagens_src, indices_vizinhos);
		float step_deg = 0.1; // [DEGREES]
	int raios_360 = int(360.0 / step_deg), raios_180 = raios_360 / 2.0; // Quantos raios sairao do centro para formar 360 e 180 graus de um circulo 2D

	//Size of final panoramic
	cv::Mat im360 = cv::Mat::zeros(cv::Size(raios_360, raios_180), CV_8UC3); // Imagem 360 ao final de todas as fotos passadas sem blending 

	////////////// Parametros de simulação ////////////////////////////////
 // Inicialização das varivaeis para otimzação
	int searchAgentsCount_m = 35;// numero de agentes
	int dimension_m = indices_vizinhos.size() * 6;// imagens_src.size() * 6; //  dimensão 
	int iterations = 1000; // número de iterações
	int simulations = 1;//quantidade de simulações
	double **positions_inicial = Utils::Create2DRandomArray(searchAgentsCount_m, dimension_m, lb, up);// posição inicial dos agentes 
	cout << "aqui"
		;
	std::vector<double> cameras;
	int variable = 0;
	int teste = 0;
	int teste2 = 0; int c = 0;
	/*for (size_t j = 0; j < cameras.size(); j++)
	{
		teste = variable + 2;
		c = 0;
		for (variable; variable < teste; variable++)
		{*/
		//cameras[j][c] = positions_inicial[0][variable];
		/*cameras.push_back(positions_inicial[0][0]);
		cameras.push_back(positions_inicial[0][1]);
		cameras.push_back(positions_inicial[0][2]);
		cameras.push_back(positions_inicial[0][3]);
		cameras.push_back(positions_inicial[0][4]);
		cameras.push_back(positions_inicial[0][5]);
		cameras.push_back(positions_inicial[0][6]);
		cameras.push_back(positions_inicial[0][7]);
		cameras.push_back(positions_inicial[0][8]);
		cameras.push_back(positions_inicial[0][9]);
		cameras.push_back(positions_inicial[0][10]);
		cameras.push_back(positions_inicial[0][11]);*/
		/*		c++;

			}
			variable = variable + 4;
		}*/
		//double* camera = (double*)(&(positions_inicial[0]));//positions_inicial[0];
	Utils::clearResultstxt(pasta);//limpar arquivos de simulações anteriores dessa pasta;

	std::cout << "Iniciando processo de otimizacao ..." << std::endl;
	int w = 3599;
	int h = 1799;
	/*for (int i = 0; i < 336;i++) {
		cout << positions_inicial[0][i] << endl;
	}*/
	//**************************** LEVENBERG ****************************

	//double* camera = new double[12];//(double*)(&(cameras));

	//std::copy(&positions_inicial[0][0], &positions_inicial[0][11], &camera[0]);

	//ceres::Problem ba;
	///*for (int frame0 = 0; frame0 < bestKey.size(); frame0++)
	//{
	//	int l = 0;
	//	for (int j = 0; j < bestKey[frame0].size(); j++)
	//	{*/
	//		std::vector<cv::KeyPoint> kpts1 = bestKey[0][0];
	//		std::vector<cv::KeyPoint> kpts2 = bestKey[0][ 1];
	//		int frame1 = 1;// indices_vizinhos[frame0][l];
	//	/*	if (bestKey[frame0][j].size() > 0)
	//		{
	//			*///#pragma omp parallel for 


	//			for (int k = 0; k < kpts1.size(); k++)
	//			{
	//				// features das duas imagens
	//				cv::KeyPoint kp1 = kpts1[k];
	//				cv::KeyPoint kp2 = kpts2[k];
	//				Eigen::Vector3d  p, pe1;
	//				p << kp2.pt.x, kp2.pt.y, 1;
	//				pe1 << kp1.pt.x, kp1.pt.y, 1;


	//				ceres::CostFunction* cost_func = ReprojectionError::create(pe1, p, 0,frame1,h,w, positions_inicial[0]);
	//				double* camera = new double[12];//(double*)(&(cameras));
	//			
	//				std::copy(&positions_inicial[0][0], &positions_inicial[0][11], &camera[0]);
	//				ba.AddResidualBlock(cost_func, NULL, camera);
	//			}
	///*		}
	//		l++;
	//		j++;
	//	}
	//}*/
	//ceres::Solver::Options options;
	//options.num_threads = 8;//numero de nucleos do computador
	//options.max_num_iterations = 1000;
	//options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
	//
	//options.linear_solver_type = ceres::ITERATIVE_SCHUR;//ceres::SPARSE_SCHUR;//ceres::DENSE_SCHUR;
	//options.minimizer_progress_to_stdout = true;
	//ceres::Solver::Summary summary;
	//ceres::Solve(options, &ba, &summary);
	//;//otimiza os parametros
	////  std::cout<<summary.FullReport()<<std::endl;
	//std::cout << "Erro Inicial: " << summary.initial_cost << std::endl;
	//std::cout << "Erro Final: " << summary.final_cost << std::endl;
	
	//**************************** GWO ****************************
	std::cout << endl << "GWO" << endl;
	double **Best_pos_GWO = new double *[simulations];
	double *Best_sol_GWO = new double[simulations];
	for (int a = 0; a < simulations; a++)
	{

		atexit(freeMemory);
		argument = new Argument(searchAgentsCount_m, iterations, lb, up);
		argument->Parse();
		gwo = new GWO(argument->GetBenchmark(), searchAgentsCount_m, iterations, ind_val, positions_inicial, pasta);
		(void)gwo->Evaluate(true, bestKey, imagens_src, im360, indices_vizinhos);
		std::cout << " Result " << a << std::endl
			<< gwo << std::endl << std::endl;
		cout << "";
		Best_pos_GWO[a] = gwo->GetAlphaPosition();
		Best_sol_GWO[a] = gwo->GetAlphaScore();

	}

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

	double **Best_pos_BAT = new double *[simulations];
	double *Best_sol_BAT = new double[simulations];
	std::cout << endl << "BAT" << endl;
	for (int a = 0; a < simulations; a++)
	{
		atexit(freeMemory);
		argument = new Argument(searchAgentsCount_m, iterations, lb, up);
		argument->Parse();
		bat = new BAT(argument->GetBenchmark(), searchAgentsCount_m, iterations, ind_val, amp_sonora, taxa, lambda, alpha, gama, fmax, fmin, A0, rf, positions_inicial, pasta);
		(void)bat->Evaluate(true, bestKey, imagens_src, im360, indices_vizinhos);
		std::cout << " Result " << a << std::endl
			<< bat << std::endl << std::endl;
		cout << "";

		Best_pos_BAT[a] = bat->GetBestPositionBAT();
		Best_sol_BAT[a] = bat->GetBestScore();
	}



	freeMemory();


	//////**************************** AOA ****************************

	std::vector<double> media_inter(iterations, 0.0); // Vetor média por interações;
	std::vector<double> melhor_inter(iterations, 0.0); // Vetor melhor solução por interação;
	std::vector<double> MOA(iterations, 0.0); //Math Optimizer Accelerated;
	double max_MOA = 2, min_MOA = 0.1; // Valor máximo e minimo da função MOA;
	std::vector<double> MOP(iterations, 0.0); // Math Optimizer Probability;
	double alpha_MOP = 5; //Parâmetro sensível e define a precisão da exploração nas iterações; (Valor - Artigo original);
	double u = 0.49999; // Parâmetro de controle para ajustar o processo de busca; (Valor 0.5 - Artigo original)

	double **Best_pos_AOA = new double *[simulations];
	double *Best_sol_AOA = new double[simulations];
	std::cout << endl << "AOA" << endl;
	for (int a = 0; a < simulations; a++)
	{
		atexit(freeMemory);
		argument = new Argument(searchAgentsCount_m, iterations, lb, up);
		argument->Parse();
		aoa = new AOA(argument->GetBenchmark(), searchAgentsCount_m, iterations, ind_val, media_inter, melhor_inter, MOA, max_MOA, min_MOA, MOP, alpha_MOP, u, positions_inicial, pasta);
		(void)aoa->Evaluate(true, bestKey, imagens_src, im360, indices_vizinhos);
		std::cout << " Result " << a << std::endl
			<< aoa << std::endl << std::endl;
		cout << "";
		Best_pos_AOA[a] = aoa->GetBestPositionAOA();
		Best_sol_AOA[a] = aoa->GetBestScore();
	}
	freeMemory();

	//////**************************** SSA ****************************
	
	std::cout << endl << "SSA" << endl;

	double **Best_pos_SSA = new double *[simulations];
	double *Best_sol_SSA = new double[simulations];
	for (int a = 0; a < simulations; a++)
	{
		atexit(freeMemory);
		argument = new Argument(searchAgentsCount_m, iterations, lb, up);
		argument->Parse();
		ssa = new SSA(argument->GetBenchmark(), searchAgentsCount_m, iterations, ind_val, positions_inicial, pasta);
		(void)ssa->Evaluate(true, bestKey, imagens_src, im360, indices_vizinhos);
		std::cout << " Result " << a << std::endl
			<< ssa << std::endl << std::endl;
		cout << "";
		Best_pos_SSA[a] = ssa->GetBestPositionSSA();
		Best_sol_SSA[a] = ssa->GetBestScore();
	}
	freeMemory();
	//////**************************** pso ****************************

	std::cout << endl << "PSO" << endl;
	//parâmetros do PSO
	double Vmax = 6;
	double Wmax = 0.9;
	double Wmin = 0.2;
	double c1 = 2;
	double c2 = 2;

	double **Best_pos_PSO = new double *[simulations];
	double *Best_sol_PSO = new double[simulations];
	for (int a = 0; a < simulations; a++)
	{
		atexit(freeMemory);
		argument = new Argument(searchAgentsCount_m, iterations, lb, up);
		argument->Parse();
		pso = new PSO(argument->GetBenchmark(), searchAgentsCount_m, iterations, ind_val, positions_inicial, pasta, Vmax, Wmax, Wmin, c1, c2);
		(void)pso->Evaluate(true, bestKey, imagens_src, im360, indices_vizinhos);
		std::cout << " Result " << a << std::endl
			<< pso << std::endl << std::endl;
		cout << "";
		Best_pos_PSO[a] = pso->GetBestPositionPSO();
		Best_sol_PSO[a] = pso->GetBestScorePSO();
	}
	freeMemory();

	////**************************** Plotar Imagens ****************************



	//std::cout << "Gerando imagens panoramicas"<<endl;
	////////Encontrar melhor solução de todas as simulações 
	//////cout << "GWO - ";
	//auto it_gwo = std::min_element(Best_sol_GWO, Best_sol_GWO + simulations);
	//int index_GWO = std::distance(Best_sol_GWO, it_gwo);
	//cv::Mat im360_GWO = Utils::panoramicas(dimension_m, pasta, Best_pos_GWO[index_GWO]);
	//imwrite(pasta + "im360_GWO.png", im360_GWO);

	////cout << "BAT - ";
	////auto it_BAT = std::min_element(Best_sol_BAT, Best_sol_BAT + simulations);
	////int index_BAT = std::distance(Best_sol_BAT, it_BAT);
	////cv::Mat im360_BAT = Utils::panoramicas(dimension_m, pasta, Best_pos_BAT[index_BAT]);
	////imwrite(pasta + "im360_BAT.png", im360_BAT);

	////cout << "AOA - ";
	////auto it_AOA = std::min_element(Best_sol_AOA, Best_sol_AOA + simulations);
	////int index_AOA = std::distance(Best_sol_AOA, it_AOA);
	////cv::Mat im360_AOA = Utils::panoramicas(dimension_m, pasta, Best_pos_AOA[index_AOA]);
	////imwrite(pasta + "im360_AOA.png", im360_AOA);

	////cout << "SSA - ";
	////auto it_SSA = std::min_element(Best_sol_SSA, Best_sol_SSA + simulations);
	////int index_SSA = std::distance(Best_sol_SSA, it_SSA);
	////cv::Mat im360_SSA = Utils::panoramicas(dimension_m, pasta, Best_pos_SSA[index_SSA]);
	////imwrite(pasta + "im360_SSA.png", im360_SSA);
	////freeMemoryOti();
	std::cout << " Processo Finalizado";
	return 0;
}