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
	if (gwo) {
		delete gwo;
		gwo = nullptr;
	}
	/*	if (bat) {
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
int main() {
	
	//Localiza��o do arquivo NVM/SFM com posi��o das imagens de acordo com o PEPO
	std::string pasta = "C:/Users/julia/Pictures/geradorartesspace/scan3/";

	//Encontrando posi��o originais das imagens (Posi��o do PEPO)
	double fx, fy, cx, cy;
	std::vector<std::string> imagens_src;
	std::vector<std::vector<float> > pose = Utils::lerSFM(pasta, fx, fy, cx, cy, imagens_src);

	//Enocntrandos os limites maximos e minimos dos par�metros 
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


	std::vector<std::vector<std::vector<cv::KeyPoint>>> bestKey = Utils::sift_matches_matrix_encontrar_melhor(matriz_matches, descp_src, kpts_src, imagens_src, indices_vizinhos);

	float step_deg = 0.1; // [DEGREES]
	int raios_360 = int(360.0 / step_deg), raios_180 = raios_360 / 2.0; // Quantos raios sairao do centro para formar 360 e 180 graus de um circulo 2D

	//Size of final panoramic
	cv::Mat im360 = cv::Mat::zeros(cv::Size(raios_360, raios_180), CV_8UC3); // Imagem 360 ao final de todas as fotos passadas sem blending 

	////////////// Parametros de simula��o ////////////////////////////////
 // Inicializa��o das varivaeis para otimza��o
	int searchAgentsCount_m = 35;// numero de agentes
	int dimension_m = imagens_src.size() * 6; //  dimens�o 
	int iterations = 1; // n�mero de itera��es
	int simulations = 1;//quantidade de simula��es
	double **positions_inicial = Utils::Create2DRandomArray(searchAgentsCount_m, dimension_m, lb, up);// posi��o inicial dos agentes 


	

	//**************************** GWO ****************************

	for (int a = 0; a < simulations; a++)
	{
		atexit(freeMemory);
		argument = new Argument(searchAgentsCount_m, iterations, lb, up);
		argument->Parse();
		gwo = new GWO(argument->GetBenchmark(), searchAgentsCount_m, iterations, ind_val, positions_inicial,pasta);
		(void)gwo->Evaluate(true, bestKey, imagens_src, im360, indices_vizinhos);
		std::cout << " Result GWO:" << std::endl
			<< gwo << std::endl << std::endl;
		cout << "";
		
	}
	freeMemory();


	//**************************** BAT ****************************
	//par�metros especificos do Bat

	std::vector<double> taxa(searchAgentsCount_m, 0.0);//taxa de emiss�o dos pulsos
	std::vector<double> amp_sonora(searchAgentsCount_m, Utils::GenerateRandomNumber()); // amplitude sonora
	double lambda = 0.01;
	double alpha = 0.9995;
	double gama = 0.0015;
	double fmax = 100;// frequencia maxima
	double fmin = 0; //frequencia minima
	double A0 = 1;//amplitude sonora inicial
	double rf = 1;//taxa de emissao 


	for (int a = 0; a < simulations; a++)
	{
		atexit(freeMemory);
		argument = new Argument(searchAgentsCount_m, iterations, lb, up);
		argument->Parse();
		bat = new BAT(argument->GetBenchmark(), searchAgentsCount_m, iterations, ind_val, amp_sonora, taxa, lambda, alpha, gama, fmax, fmin, A0, rf, positions_inicial,pasta);
		(void)bat->Evaluate(true, bestKey, imagens_src, im360, indices_vizinhos);
		std::cout << "Result BAT:" << std::endl
			<< bat << std::endl << std::endl;
		cout << "";
	}
	freeMemory();


	//**************************** AOA ****************************

	std::vector<double> media_inter(iterations, 0.0); // Vetor m�dia por intera��es;
	std::vector<double> melhor_inter(iterations, 0.0); // Vetor melhor solu��o por intera��o;
	std::vector<double> MOA(iterations, 0.0); //Math Optimizer Accelerated;
	double max_MOA = 2,  min_MOA = 0.1; // Valor m�ximo e minimo da fun��o MOA;
	std::vector<double> MOP(iterations, 0.0); // Math Optimizer Probability;
	double alpha_MOP = 5; //Par�metro sens�vel e define a precis�o da explora��o nas itera��es; (Valor - Artigo original);
	double u = 0.49999; // Par�metro de controle para ajustar o processo de busca; (Valor 0.5 - Artigo original)

	for (int a = 0; a < simulations; a++)
	{
		atexit(freeMemory);
		argument = new Argument(searchAgentsCount_m, iterations, lb, up);
		argument->Parse();
		aoa = new AOA(argument->GetBenchmark(), searchAgentsCount_m, iterations, ind_val, media_inter, melhor_inter, MOA, max_MOA, min_MOA, MOP, alpha_MOP, u, positions_inicial,pasta);
		(void)aoa->Evaluate(true, bestKey, imagens_src, im360, indices_vizinhos);
		std::cout << "Result AOA:" << std::endl
			<< aoa << std::endl << std::endl;
		cout << "";
		
	}
	freeMemory();

	//**************************** SSA ****************************
	for (int a = 0; a < simulations; a++)
	{
		atexit(freeMemory);
		argument = new Argument(searchAgentsCount_m, iterations, lb, up);
		argument->Parse();
		ssa = new SSA(argument->GetBenchmark(), searchAgentsCount_m, iterations, ind_val, positions_inicial,pasta);
		(void)ssa->Evaluate(true, bestKey, imagens_src, im360, indices_vizinhos);
		std::cout << "Result SSA:" << std::endl
			<< ssa << std::endl << std::endl;
		cout << "";

	}
	freeMemory();

	//**************************** Plotar Imagens ****************************

	return 0; 
}