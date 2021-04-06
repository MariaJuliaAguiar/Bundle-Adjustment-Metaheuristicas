#ifndef __UTILS_H
#define __UTILS_H
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>
#include "benchmark.hpp"
class Utils {

public:
	static std::vector<std::vector<float>> lerSFM(std::string pasta, double &fx, double &fy, double &cx, double &cy, std::vector<std::string> &imagens_src);
	static void uplowerBound( std::vector<std::vector<float> > pose, double fx, double fy, double cx, double cy, std::vector<double> &lb, std::vector<double> &up);
	static std::vector<std::vector<int>> FindVizinhos(int images_size );
	static void calcular_features_sift(std::vector<cv::Mat>  &descp_src, std::vector<std::vector<cv::KeyPoint> >  &kpts_src, std::vector<std::string> imagens_src);
	static std::vector<std::vector<std::vector<cv::KeyPoint>>> sift_matches_matrix_encontrar_melhor(std::vector<std::vector<  std::vector<cv::DMatch> >> matriz_matches, std::vector<cv::Mat>  descp_src, std::vector< std::vector<cv::KeyPoint> >  kpts_src, std::vector<std::string> imagens_src, std::vector<std::vector<int>> &indices);
	static void filtrar_matches_keypoints_repetidos(std::vector<cv::KeyPoint> &kt, std::vector<cv::KeyPoint> &ks, std::vector<cv::DMatch> &m);
	static double **Create2DRandomArray(unsigned int rowCount, unsigned int columnCount, std::vector<double> lb, std::vector<double> up);
	static double GenerateRandomNumber();
	static double *Create1DZeroArray(unsigned int columnCount);
	static double** Create2DZeroArray(int search_agents, int dimension);
	static double* Create1DArray(unsigned int columnCount);
	static void Clip1DArray(double array[], unsigned int columnCount, Boundaries boundaries[]);
};

#endif