
#include <iostream>
#include <string>
#include <math.h>
#include <sys/stat.h>
#include <ostream>
#include <chrono>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <dirent.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/mat.hpp>
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d.hpp"
#include <opencv2/stitching/detail/blenders.hpp>

#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

//Definicoes e namespaces

using namespace pcl;
using namespace pcl::io;
using namespace cv;
using namespace std;
using namespace Eigen;
using namespace cv::xfeatures2d;
typedef PointXYZRGBNormal PointT;
typedef PointXYZRGB       PointC;
void doTheThing(float sd, Vector3d p2, Vector3d p4, Vector3d p5, Vector3d pc, Mat im, Mat &img, Mat im360) {
	// A partir de frustrum, calcular a posicao de cada pixel da imagem fonte em XYZ, assim como quando criavamos o plano do frustrum
	Vector3d hor_step, ver_step; // Steps pra se andar de acordo com a resolucao da imagem
	hor_step = (p4 - p5) / float(im.cols);
	ver_step = (p2 - p5) / float(im.rows);
#pragma omp parallel for
	for (int i = 0; i < im.rows; i++) { // Vai criando o frustrum a partir dos cantos da imagem
		for (int j = 0; j < im.cols; j++) {
			Vector3d ponto;
			ponto = p5 + hor_step * j + ver_step * i;

			if ((pc - ponto).norm() < (p4 - p5).norm() / 2)
			{

				// Calcular latitude e longitude da esfera de volta a partir de XYZ
				float lat = RAD2DEG(acos(ponto[1] / ponto.norm()));
				float lon = -RAD2DEG(atan2(ponto[2], ponto[0]));
				lon = (lon < 0) ? lon += 360.0 : lon; // Ajustar regiao do angulo negativo, mantendo o 0 no centro da imagem

				// Pelas coordenadas, estimar posicao do pixel que deve sair na 360 final e pintar - da forma como criamos a esfera
				int u = int(lon / sd);
				u = (u >= im360.cols) ? im360.cols - 1 : u; // Nao deixar passar do limite de colunas por seguranca
				u = (u < 0) ? 0 : u;
				int v = im360.rows - 1 - int(lat / sd);
				v = (v >= im360.rows) ? im360.rows - 1 : v; // Nao deixar passar do limite de linhas por seguranca
				v = (v < 0) ? 0 : v;
				// Pintar a imagem final com as cores encontradas
				im360.at<Vec3b>(Point(u, v)) = im.at<Vec3b>(Point(j, im.rows - 1 - i));
				img.at<Vec3b>(Point(u, v)) = im.at<Vec3b>(Point(j, im.rows - 1 - i));

			}

		}
	}
}

void panoramicas(int dimension, std::string pasta, double* best_Gwo) {

	std::vector<Eigen::Vector2d> Cs;
	std::vector<Eigen::Matrix3d> rot;
	std::vector<double> par;
	std::vector<std::string> nomes_imagens;
	std::vector<Eigen::Vector2d> foco;
	foco.resize(dimension / 6);
	rot.resize(dimension / 6);
	Cs.resize(dimension / 6);
	nomes_imagens.resize(dimension / 6);
	int teste = 0;
	int variable = 0;
	for (int indice_posicao = 0; indice_posicao < dimension / 6; indice_posicao++)
	{
		std::string nome_imagem_atual;
		if (indice_posicao + 1 < 10)
			nome_imagem_atual = "imagem_00" + std::to_string(indice_posicao + 1);
		else if (indice_posicao + 1 < 100)
			nome_imagem_atual = "imagem_0" + std::to_string(indice_posicao + 1);
		else
			nome_imagem_atual = "imagem_" + std::to_string(indice_posicao + 1);
		teste = variable + 6;

		nomes_imagens[indice_posicao] = pasta + nome_imagem_atual + ".png";
		for (variable; variable < teste; variable++)
		{

			par.push_back(best_Gwo[variable]);
		}



		Eigen::Matrix3d r1;

		r1 = Eigen::AngleAxisd(-DEG2RAD(par[1]), Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(-DEG2RAD(par[0]), Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(-DEG2RAD(0), Eigen::Vector3d::UnitZ());
		rot[indice_posicao] = r1;
		foco[indice_posicao] << par[2], par[3];


		// Centro
		Eigen::Vector2d C((par[4]), (par[5]));

		Cs[indice_posicao] = C;
		par.clear();
	}

	int qnt_images_linha = (dimension / 6) / 8; //  Quantidade de Imagens por linha 
	int i = 0;
	std::vector<int> ind;
	// Reorganizando nvm/sfm para facilitar o blending -  Colocando em linhas
	while (i < dimension / 6)
	{
		ind.push_back(i);
		i = i + 15;
		if (i >= dimension / 6) break;
		ind.push_back(i);
		i = i + 1;
	}
	i = 1;
	while (i < dimension / 6)
	{
		ind.push_back(i);
		i = i + 13;
		if (i >= dimension / 6) break;
		ind.push_back(i);
		i = i + 3;
	}
	i = 2;
	while (i < dimension / 6)
	{
		ind.push_back(i);
		i = i + 11;
		if (i >= dimension / 6) break;
		ind.push_back(i);
		i = i + 5;
	}
	i = 3;
	while (i < dimension / 6)
	{
		ind.push_back(i);
		i = i + 9;
		if (i >= dimension / 6) break;
		ind.push_back(i);
		i = i + 7;
	}

	i = 4;
	while (i < dimension / 6)
	{
		ind.push_back(i);
		i = i + 7;
		if (i >= dimension / 6) break;
		ind.push_back(i);
		i = i + 9;
	}
	i = 5;
	while (i < dimension / 6)
	{
		ind.push_back(i);
		i = i + 5;
		if (i >= dimension / 6) break;
		ind.push_back(i);
		i = i + 11;
	}
	i = 6;
	while (i < dimension / 6)
	{
		ind.push_back(i);
		i = i + 3;
		if (i >= dimension / 6) break;
		ind.push_back(i);
		i = i + 13;
	}
	i = 7;
	while (i < dimension / 6)
	{
		ind.push_back(i);
		i = i + 1;
		if (i >= dimension / 6) break;
		ind.push_back(i);
		i = i + 15;
	}

	int index = 0;

	// Supoe a esfera com resolucao em graus de tal forma - resolucao da imagem final
	float R = 1; // Raio da esfera [m]
	// Angulos para lat e lon, 360 de range para cada, resolucao a definir no step_deg
	float step_deg = 0.1; // [DEGREES]
	int raios_360 = int(360.0 / step_deg), raios_180 = raios_360 / 2.0; // Quantos raios sairao do centro para formar 360 e 180 graus de um circulo 2D

	//Panoramica para cada Linha
	std::vector <cv::Mat>  im360_parcial; im360_parcial.resize(8);
	cv::Mat anterior = cv::Mat::zeros(cv::Size(raios_360, raios_180), CV_8UC3);
	cv::Mat im360 = cv::Mat::zeros(cv::Size(raios_360, raios_180), CV_8UC3); // Imagem 360 ao final de todas as fotos passadas sem blending 


	int contador = 0;

	//ros::Time tempo = ros::Time::now();
	/// Para cada imagem

	std::vector<Eigen::Vector3d> pontos;
	printf("Processando cada foto, sem print nenhum pra ir mais rapido ...\n");
	for (int i = 0; i < nomes_imagens.size(); i++)
	{
		printf("Processando foto %d...\n", i + 1);
		// Ler a imagem a ser usada
		cv::Mat image = cv::imread(nomes_imagens[ind[i]]);
		if (image.cols < 3)
			std::cout << ("Imagem nao foi encontrada, checar NVM ...");

		// Definir o foco em dimensoes fisicas do frustrum
		double F = R;
		double minX, minY, maxX, maxY;
		double dx = Cs[ind[i]][0] - double(image.cols) / 2, dy = Cs[ind[i]][1] - double(image.rows) / 2;
		//    double dx = 0, dy = 0;
		maxX = F * (float(image.cols) - 2 * dx) / (2.0*foco[ind[i]][0]);
		minX = -F * (float(image.cols) + 2 * dx) / (2.0*foco[ind[i]][0]);
		maxY = F * (float(image.rows) - 2 * dy) / (2.0*foco[ind[i]][1]);
		minY = -F * (float(image.rows) + 2 * dy) / (2.0*foco[ind[i]][1]);
		//		// Calcular os 4 pontos do frustrum
		//		/*
		//								origin of the camera = p1
		//								p2--------p3
		//								|          |
		//								|  pCenter |<--- Looking from p1 to pCenter
		//								|          |
		//								p5--------p4
		//		*/
		Eigen::Vector3d p, p1, p2, p3, p4, p5, pCenter;
		p << 0, 0, 0;
		p1 = rot[ind[i]] * p;
		p << minX, minY, F;
		p2 = rot[ind[i]] * p;
		p << maxX, minY, F;
		p3 = rot[ind[i]] * p;
		p << maxX, maxY, F;
		p4 = rot[ind[i]] * p;
		p << minX, maxY, F;
		p5 = rot[ind[i]] * p;
		p << 0, 0, F;
		pCenter = rot[ind[i]] * p;



		// Fazer tudo aqui nessa nova funcao, ja devolver a imagem esferica inclusive nesse ponto
		cv::Mat imagem_esferica = cv::Mat::zeros(cv::Size(raios_360, raios_180), CV_8UC3);
		doTheThing(step_deg, p2.block<3, 1>(0, 0), p4.block<3, 1>(0, 0), p5.block<3, 1>(0, 0), pCenter.block<3, 1>(0, 0), image, im360, imagem_esferica);
		//	doTheThing2(step_deg, H, image, im360);

		//if (i == 0) {

		//	anterior.release();
		//	index = 0;
		//	anterior = imagem_esferica;
		//	anterior.convertTo(anterior, CV_32F, 1.0 / 255.0);

		//}
		//if (i > 0 && i < qnt_images_linha)
		//{

		//	imagem_esferica.convertTo(imagem_esferica, CV_32F, 1.0 / 255.0);
		//	im360_parcial[0] = Utils::multiband_blending(anterior, imagem_esferica, index, qnt_images_linha);
		//	anterior = im360_parcial[0];
		//	imagem_esferica.release();

		//}
		//if (i == qnt_images_linha) {


		//	anterior.release();
		//	index = 0;
		//	anterior = imagem_esferica;
		//	anterior.convertTo(anterior, CV_32F, 1.0 / 255.0);
		//}

		//if (i > qnt_images_linha && i < 2 * qnt_images_linha)
		//{


		//	imagem_esferica.convertTo(imagem_esferica, CV_32F, 1.0 / 255.0);
		//	im360_parcial[1] = Utils::multiband_blending(anterior, imagem_esferica, index, qnt_images_linha);
		//	anterior = im360_parcial[1];
		//	imagem_esferica.release();
		//	//imwrite(pasta + "imagem_esferica_Blending.png", im360_parcial[1] * 255);
		//}
		//if (i == 2 * qnt_images_linha)
		//{


		//	anterior.release();
		//	index = 0;
		//	anterior = imagem_esferica;
		//	anterior.convertTo(anterior, CV_32F, 1.0 / 255.0);
		//}

		//if (i > 2 * qnt_images_linha && i < 3 * qnt_images_linha)
		//{


		//	imagem_esferica.convertTo(imagem_esferica, CV_32F, 1.0 / 255.0);
		//	im360_parcial[2] = Utils::multiband_blending(anterior, imagem_esferica, index, qnt_images_linha);
		//	anterior = im360_parcial[2];
		//	//	imwrite(pasta + "imagem_esferica_Blending.png", im360_parcial[2] * 255);
		//	imagem_esferica.release();

		//}
		//if (i == 3 * qnt_images_linha) {


		//	anterior.release();
		//	index = 0;
		//	anterior = imagem_esferica;
		//	anterior.convertTo(anterior, CV_32F, 1.0 / 255.0);
		//}

		//if (i > 3 * qnt_images_linha && i < 4 * qnt_images_linha)
		//{

		//	imagem_esferica.convertTo(imagem_esferica, CV_32F, 1.0 / 255.0);
		//	im360_parcial[3] = Utils::multiband_blending(anterior, imagem_esferica, index, qnt_images_linha);
		//	anterior = im360_parcial[3];
		//	imagem_esferica.release();
		//	//imwrite(pasta + "imagem_esferica_Blending.png", im360_parcial[3] * 255);

		//}
		//if (i == 4 * qnt_images_linha) {


		//	anterior.release();
		//	index = 0;
		//	anterior = imagem_esferica;
		//	anterior.convertTo(anterior, CV_32F, 1.0 / 255.0);
		//}
		//if (i > 4 * qnt_images_linha && i < 5 * qnt_images_linha)
		//{

		//	imagem_esferica.convertTo(imagem_esferica, CV_32F, 1.0 / 255.0);
		//	im360_parcial[4] = Utils::multiband_blending(anterior, imagem_esferica, index, qnt_images_linha);
		//	anterior = im360_parcial[4];
		//	//	imwrite(pasta + "imagem_esferica_Blending.png", im360_parcial[4] * 255);
		//	imagem_esferica.release();

		//}
		//if (i == 5 * qnt_images_linha) {


		//	anterior.release();
		//	index = 0;
		//	anterior = imagem_esferica;
		//	anterior.convertTo(anterior, CV_32F, 1.0 / 255.0);
		//}
		//if (i > 5 * qnt_images_linha && i < 6 * qnt_images_linha)
		//{

		//	imagem_esferica.convertTo(imagem_esferica, CV_32F, 1.0 / 255.0);
		//	im360_parcial[5] = Utils::multiband_blending(anterior, imagem_esferica, index, qnt_images_linha);
		//	anterior = im360_parcial[5];
		//	//	imwrite(pasta + "imagem_esferica_Blending.png", im360_parcial[5] * 255);
		//	imagem_esferica.release();

		//}
		//if (i == 6 * qnt_images_linha) {


		//	anterior.release();
		//	index = 0;
		//	anterior = imagem_esferica;
		//	anterior.convertTo(anterior, CV_32F, 1.0 / 255.0);
		//}
		//if (i > 6 * qnt_images_linha && i < 7 * qnt_images_linha)
		//{

		//	imagem_esferica.convertTo(imagem_esferica, CV_32F, 1.0 / 255.0);
		//	im360_parcial[6] = Utils::multiband_blending(anterior, imagem_esferica, index, qnt_images_linha);
		//	anterior = im360_parcial[6];
		//	//	imwrite(pasta + "imagem_esferica_Blending.png", im360_parcial[6] * 255);
		//	imagem_esferica.release();

		//}
		//if (i == 7 * qnt_images_linha) {


		//	anterior.release();
		//	index = 0;
		//	anterior = imagem_esferica;
		//	anterior.convertTo(anterior, CV_32F, 1.0 / 255.0);
		//}
		//if (i > 7 * qnt_images_linha && i < 8 * qnt_images_linha)
		//{

		//	imagem_esferica.convertTo(imagem_esferica, CV_32F, 1.0 / 255.0);
		//	im360_parcial[7] = Utils::multiband_blending(anterior, imagem_esferica, index, qnt_images_linha);
		//	anterior = im360_parcial[7];
		//	//	imwrite(pasta + "imagem_esferica_Blending.png", im360_parcial[6] * 255);
		//	imagem_esferica.release();

		//}

		index++;
	} // Fim do for imagens;

	//////Resultado Final - Juntando os blendings horizontais
	/*cv::Mat result;
	index = 73;
	result = im360_parcial[7];
	for (int i = 7; i > 0; i--) {

		result = Utils::multiband_blending(result, im360_parcial[i - 1], index, qnt_images_linha);
	}

	result.convertTo(result, CV_8UC3, 255);
	imwrite(pasta + "GWO_BLENDING.png", result);*/
	// Salvando imagem esferica final
	imwrite(pasta + "GWO_normal.png", im360);
	printf("Processo finalizado.");


	
}