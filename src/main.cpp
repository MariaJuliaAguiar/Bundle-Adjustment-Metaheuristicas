#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/mat.hpp>
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d.hpp"
#include <opencv2/stitching/detail/blenders.hpp>
#include <cstdlib> //atexit
#include <iostream> //cerr, cout
#include <numeric>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
//includes meus 

#include "argument.hpp"
#include "utils.hpp"
#include "gwo.hpp"
#include "bat.hpp"
#include "AOA.hpp"
#include "ssa.hpp"

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


void freeMemory() {
	if (argument) {
		delete argument;
		argument = nullptr;
	}
	/*if (gwo) {
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
	}*/
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

	// Features Matching - Keypoints e descriptors
	vector<vector<cv::KeyPoint>>  kpts_src;
	vector<cv::Mat>  descp_src;
	descp_src.resize(imagens_src.size());
	kpts_src.resize(imagens_src.size());
	//Encontrando features
	Utils::calcular_features_sift(descp_src, kpts_src, imagens_src);

	//Encontrando os matches 

	// Ajustar matriz de quantidade de matches
	Eigen::MatrixXi matches_count = Eigen::MatrixXi::Zero(descp_src.size() - 1, descp_src.size() - 1);
	vector<vector<  vector<cv::DMatch> >> matriz_matches(indices_vizinhos.size());
	for (int i = 0; i < matriz_matches.size(); i++)
		matriz_matches.at(i).resize(indices_vizinhos[i].size());
	/*std::vector<std::vector<std::vector<cv::KeyPoint>>> bestKey;*/

	std::vector<std::vector<std::vector<cv::KeyPoint>>> bestKey = Utils::sift_matches_matrix_encontrar_melhor(matriz_matches, descp_src, kpts_src, imagens_src, indices_vizinhos);

	float step_deg = 0.1; // [DEGREES]
	int raios_360 = int(360.0 / step_deg), raios_180 = raios_360 / 2.0; // Quantos raios sairao do centro para formar 360 e 180 graus de um circulo 2D

	//Size of final panoramic
	cv::Mat im360 = cv::Mat::zeros(cv::Size(raios_360, raios_180), CV_8UC3); // Imagem 360 ao final de todas as fotos passadas sem blending 

	////////////// Parametros de simulação ////////////////////////////////
 // Inicialização das varivaeis para otimzação
	int searchAgentsCount_m = 35;// numero de agentes
	int dimension_m = imagens_src.size() * 6; //  dimensão 
	int iterations = 1; // número de iterações
	int simulations =1;//quantidade de simulações
	double **positions_inicial = Utils::Create2DRandomArray(searchAgentsCount_m, dimension_m, lb, up);// posição inicial dos agentes 

	Utils::clearResultstxt(pasta);//limpar arquivos de simulações anteriores dessa pasta;


	//**************************** GWO ****************************
	double **Best_pos_GWO = new double *[simulations];
	double *Best_sol_GWO = new double[simulations];
	for (int a = 0; a < simulations; a++)
	{
		atexit(freeMemory);
		argument = new Argument(searchAgentsCount_m, iterations, lb, up);
		argument->Parse();
		gwo = new GWO(argument->GetBenchmark(), searchAgentsCount_m, iterations, ind_val, positions_inicial, pasta);
		(void)gwo->Evaluate(true, bestKey, imagens_src, im360, indices_vizinhos);
		std::cout << " Result GWO:" << std::endl
			<< gwo << std::endl << std::endl;
		cout << "";
		Best_pos_GWO[a] = gwo->GetAlphaPosition();
		Best_sol_GWO[a] = gwo->GetAlphaScore();

	}

	freeMemory();


	////**************************** BAT ****************************
	//parâmetros especificos do Bat

	std::vector<double> taxa(searchAgentsCount_m, 0.0);//taxa de emissão dos pulsos
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

	for (int a = 0; a < simulations; a++)
	{
		atexit(freeMemory);
		argument = new Argument(searchAgentsCount_m, iterations, lb, up);
		argument->Parse();
		bat = new BAT(argument->GetBenchmark(), searchAgentsCount_m, iterations, ind_val, amp_sonora, taxa, lambda, alpha, gama, fmax, fmin, A0, rf, positions_inicial, pasta);
		(void)bat->Evaluate(true, bestKey, imagens_src, im360, indices_vizinhos);
		std::cout << "Result BAT:" << std::endl
			<< bat << std::endl << std::endl;
		cout << "";

		Best_pos_BAT[a] = bat->GetBestPositionBAT();
		Best_sol_BAT[a] = bat->GetBestScore();
	}



	freeMemory();


	//**************************** AOA ****************************

	std::vector<double> media_inter(iterations, 0.0); // Vetor média por interações;
	std::vector<double> melhor_inter(iterations, 0.0); // Vetor melhor solução por interação;
	std::vector<double> MOA(iterations, 0.0); //Math Optimizer Accelerated;
	double max_MOA = 2, min_MOA = 0.1; // Valor máximo e minimo da função MOA;
	std::vector<double> MOP(iterations, 0.0); // Math Optimizer Probability;
	double alpha_MOP = 5; //Parâmetro sensível e define a precisão da exploração nas iterações; (Valor - Artigo original);
	double u = 0.49999; // Parâmetro de controle para ajustar o processo de busca; (Valor 0.5 - Artigo original)

	double **Best_pos_AOA = new double *[simulations];
	double *Best_sol_AOA = new double[simulations];
	for (int a = 0; a < simulations; a++)
	{
		atexit(freeMemory);
		argument = new Argument(searchAgentsCount_m, iterations, lb, up);
		argument->Parse();
		aoa = new AOA(argument->GetBenchmark(), searchAgentsCount_m, iterations, ind_val, media_inter, melhor_inter, MOA, max_MOA, min_MOA, MOP, alpha_MOP, u, positions_inicial, pasta);
		(void)aoa->Evaluate(true, bestKey, imagens_src, im360, indices_vizinhos);
		std::cout << "Result AOA:" << std::endl
			<< aoa << std::endl << std::endl;
		cout << "";
		Best_pos_AOA[a] = aoa->GetBestPositionAOA();
		Best_sol_AOA[a] = aoa->GetBestScore();
	}
	freeMemory();

	//**************************** SSA ****************************
	double **Best_pos_SSA = new double *[simulations];
	double *Best_sol_SSA = new double[simulations];
	for (int a = 0; a < simulations; a++)
	{
		atexit(freeMemory);
		argument = new Argument(searchAgentsCount_m, iterations, lb, up);
		argument->Parse();
		ssa = new SSA(argument->GetBenchmark(), searchAgentsCount_m, iterations, ind_val, positions_inicial, pasta);
		(void)ssa->Evaluate(true, bestKey, imagens_src, im360, indices_vizinhos);
		std::cout << "Result SSA:" << std::endl
			<< ssa << std::endl << std::endl;
		cout << "";
		Best_pos_SSA[a] = ssa->GetBestPositionSSA();
		Best_sol_SSA[a] = ssa->GetBestScore();
	}
	freeMemory();


	//**************************** Plotar Imagens ****************************




	//Encontrar melhor solução de todas as simulações 

	auto it_gwo = std::min_element(Best_sol_GWO, Best_sol_GWO + simulations);
	int index_GWO = std::distance(Best_sol_GWO, it_gwo);
	cv::Mat im360_GWO = Utils::panoramicas(dimension_m, pasta, Best_pos_GWO[index_GWO]);
	imwrite(pasta + "im360_GWO.png", im360_GWO);

	auto it_BAT = std::min_element(Best_sol_BAT, Best_sol_BAT + simulations);
	int index_BAT = std::distance(Best_sol_BAT, it_BAT);
	cv::Mat im360_BAT = Utils::panoramicas(dimension_m, pasta, Best_pos_BAT[index_BAT]);
	imwrite(pasta + "im360_BAT.png", im360_BAT);

	auto it_AOA = std::min_element(Best_sol_AOA, Best_sol_AOA + simulations);
	int index_AOA = std::distance(Best_sol_AOA, it_AOA);
	cv::Mat im360_AOA = Utils::panoramicas(dimension_m, pasta, Best_pos_AOA[index_AOA]);
	imwrite(pasta + "im360_AOA.png", im360_AOA);

	auto it_SSA = std::min_element(Best_sol_SSA, Best_sol_SSA + simulations);
	int index_SSA = std::distance(Best_sol_SSA, it_SSA);
	cv::Mat im360_SSA = Utils::panoramicas(dimension_m, pasta, Best_pos_SSA[index_SSA]);
	imwrite(pasta + "im360_SSA.png", im360_SSA);
	freeMemoryOti();
	std::cout << " Processo Finalizado";
	return 0;
}