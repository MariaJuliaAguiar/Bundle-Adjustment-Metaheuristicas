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
#include "argument.hpp"
#include "utils.hpp"
#include "gwo.hpp"
#include "GWOException.hpp"
/// Definicoes e namespaces

using namespace cv;
using namespace std;
using namespace cv::xfeatures2d;
using namespace pcl;
using namespace pcl::io;
Argument *argument = nullptr;
GWO *gwo = nullptr;

void freeMemory() {
	if (argument) {
		delete argument;
		argument = nullptr;
	}
	if (gwo) {
		delete gwo;
		gwo = nullptr;
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

	std::vector<std::vector<std::vector<cv::KeyPoint>>> bestKey = Utils::sift_matches_matrix_encontrar_melhor(matriz_matches, descp_src, kpts_src, imagens_src, indices_vizinhos);

	float step_deg = 0.1; // [DEGREES]
	int raios_360 = int(360.0 / step_deg), raios_180 = raios_360 / 2.0; // Quantos raios sairao do centro para formar 360 e 180 graus de um circulo 2D

	//Size of final panoramic
	cv::Mat im360 = cv::Mat::zeros(cv::Size(raios_360, raios_180), CV_8UC3); // Imagem 360 ao final de todas as fotos passadas sem blending 

 // Inicialização das varivaeis para otimzação
	int searchAgentsCount_m = 35;// numero de agentes
	int dimension_m = imagens_src.size() * 6; //  dimensão 
	int iterations = 1; // número de iterações
	int simulations = 1;//quantidade de simulações
	double **positions_inicial = Utils::Create2DRandomArray(searchAgentsCount_m, dimension_m, lb, up);// posição inicial dos agentes 
	
	
	//**************************** GWO ****************************

	try
	{
		for (int a = 0; a < simulations; a++)
		{
			atexit(freeMemory);
			argument = new Argument(searchAgentsCount_m, iterations, lb, up);
			argument->Parse();
			gwo = new GWO(argument->GetBenchmark(), searchAgentsCount_m, iterations, ind_val, positions_inicial);
			(void)gwo->Evaluate(true, bestKey, imagens_src, im360, indices_vizinhos);
			std::cout << "Result:" << std::endl
				<< gwo << std::endl;
			cout << "";
		}
		freeMemory();
	}
	catch (GWOException &e) {
		std::cerr << "Grey wolf optimizer exception : " << e.what() << std::endl;
		return EXIT_FAILURE;
	}

	//**************************** BAT ****************************


	return 0;
}