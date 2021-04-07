
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