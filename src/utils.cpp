#include "utils.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/core/eigen.hpp>
#include <numeric>

Utils *utils = nullptr;


// Função para encontrar a posição vinda do robo que sera necessaria para encontrar o chute inicial e os limites dos parametros
std::vector<std::vector<float>>  Utils::lerSFM(std::string pasta, double &fx, double &fy, double &cx, double &cy, std::vector<std::string> &imagens_src) {

	std::string arquivo_sfm = pasta + "cameras.sfm";

	std::ifstream sfm(arquivo_sfm);
	int contador_linhas = 1;

	std::vector<std::string>  linhas;
	std::string linha;

	if (sfm.is_open()) {
		while (getline(sfm, linha)) {
			if (contador_linhas > 2 && linha.size() > 4)
				linhas.push_back(linha);

			contador_linhas++;
		}
	}
	else {
		printf("Arquivo de cameras nao encontrado. Desligando ...\n");

	}

	std::vector<std::vector<float>> pose;
	// Para cada imagem, obter valores

	for (int i = 0; i < linhas.size(); i++) {
		std::istringstream iss(linhas[i]);
		std::vector<std::string> splits(std::istream_iterator<std::string>{iss}, std::istream_iterator<std::string>());
		// Nome
		std::string nome_fim = splits[0].substr(splits[0].find_last_of('/') + 1, splits[0].size() - 1);
		imagens_src.push_back(pasta + nome_fim);
		//rotation 3x3
		Eigen::Matrix3f r;
		r << stof(splits[1]), stof(splits[2]), stof(splits[3]),
			stof(splits[4]), stof(splits[5]), stof(splits[6]),
			stof(splits[7]), stof(splits[8]), stof(splits[9]);

		Eigen::Vector3f ea = r.eulerAngles(2, 0, 1);// transforma matriz de rotação nos angulos de euler
		ea = RAD2DEG(ea);//graus

		std::vector<float> pos{ ea[0],ea[1],ea[2] };
		pose.push_back(pos);
		fx = stod(splits[13]);
		fy = stod(splits[14]);
		cx = stod(splits[15]);
		cy = stod(splits[16]);

	}

	return pose;//roll,tilt,pan

}

// Função para encontrar os limites maximos e minimos dos parâmetros
void Utils::uplowerBound(std::vector<std::vector<float> > pose, double fx, double fy, double cx, double cy, std::vector<double> &lb, std::vector<double> &up)
{

	for (int j = 0; j < pose.size(); j++)
	{
		//Lower bounds
		lb.push_back(pose[j][1] - 3);//tilt
		lb.push_back(pose[j][2] - 3);//pan
		lb.push_back(fx - 3);//foco x
		lb.push_back(fy - 3);////foco y
		lb.push_back(cx - 3);//centro x
		lb.push_back(cy - 3);//centro y

		//Upper Bounds
		up.push_back(pose[j][1] + 3);//tilt
		up.push_back(pose[j][2] + 3);//pan
		up.push_back(fx + 3);//foco x
		up.push_back(fy + 3);//foco y
		up.push_back(cx + 3);//centro x
		up.push_back(cy + 3);//centro y
	}

}
// Encontrando os indices das imagens vizinhas - 8 imagens na vertical
std::vector<std::vector<int>> Utils::FindVizinhos(int images_size) {
	int initial = 0;
	std::vector<std::vector<int>>indices_vizinhos;
	indices_vizinhos.resize(images_size);
	for (int a = initial; a < images_size; a++)
	{

		indices_vizinhos[a].push_back(a + 15 >= images_size ? initial : a + 15);
		indices_vizinhos[a].push_back(a + 1);

		a = a + 15;
		if (a >= images_size) break;
		indices_vizinhos[a].push_back(a + 1 >= images_size ? initial : a + 1);
		indices_vizinhos[a].push_back(a - 1);

	}
	initial = 1;

	for (int a = initial; a < images_size; a++)
	{

		indices_vizinhos[a].push_back(a + 13 >= images_size ? initial : a + 13);
		indices_vizinhos[a].push_back(a + 1);

		a = a + 13;
		if (a >= images_size) break;
		indices_vizinhos[a].push_back(a + 3 >= images_size ? initial : a + 3);
		indices_vizinhos[a].push_back(a - 1);
		a = a + 2;

	}

	initial = 2;

	for (int a = initial; a < images_size; a++)
	{

		indices_vizinhos[a].push_back(a + 11 >= images_size ? initial : a + 11);
		indices_vizinhos[a].push_back(a + 1);
		a = a + 11;
		if (a >= images_size) break;
		indices_vizinhos[a].push_back(a + 5 >= images_size ? initial : a + 5);
		indices_vizinhos[a].push_back(a - 1);
		a = a + 4;

	}
	initial = 3;

	for (int a = initial; a < images_size; a++)
	{

		indices_vizinhos[a].push_back(a + 9 >= images_size ? initial : a + 9);
		indices_vizinhos[a].push_back(a + 1);
		a = a + 9;
		if (a >= images_size) break;
		indices_vizinhos[a].push_back(a + 7 >= images_size ? initial : a + 7);
		indices_vizinhos[a].push_back(a - 1);
		a = a + 6;

	}
	initial = 4;
	for (int a = initial; a < images_size; a++)
	{

		indices_vizinhos[a].push_back(a + 7 >= images_size ? initial : a + 7);
		indices_vizinhos[a].push_back(a + 1);
		a = a + 7;
		if (a >= images_size) break;
		indices_vizinhos[a].push_back(a + 9 >= images_size ? initial : a + 9);
		indices_vizinhos[a].push_back(a - 1);
		a = a + 8;

	}
	initial = 5;
	for (int a = initial; a < images_size; a++)
	{

		indices_vizinhos[a].push_back(a + 5 >= images_size ? initial : a + 5);
		indices_vizinhos[a].push_back(a + 1);
		a = a + 5;
		if (a >= images_size) break;
		indices_vizinhos[a].push_back(a + 11 >= images_size ? initial : a + 11);
		indices_vizinhos[a].push_back(a - 1);
		a = a + 10;

	}

	initial = 6;
	for (int a = initial; a < images_size; a++)
	{

		indices_vizinhos[a].push_back(a + 3 >= images_size ? initial : a + 3);
		indices_vizinhos[a].push_back(a + 1);
		a = a + 3;
		if (a >= images_size) break;
		indices_vizinhos[a].push_back(a + 13 >= images_size ? initial : a + 13);
		indices_vizinhos[a].push_back(a - 1);
		a = a + 12;

	}
	initial = 7;
	for (int a = initial; a < images_size; a++)
	{

		indices_vizinhos[a].push_back(a + 1 >= images_size ? initial : a + 1);
		a = a + 1;
		if (a >= images_size) break;
		indices_vizinhos[a].push_back(a + 15 >= images_size ? initial : a + 15);
		a = a + 14;

	}

	return indices_vizinhos;
}
// Encontrar as features de todas as imagens - ROOTSFIT para melhor desempenho
void Utils::calcular_features_sift(std::vector<cv::Mat>  &descp_src, std::vector<std::vector<cv::KeyPoint> >  &kpts_src, std::vector<std::string> imagens_src)
{

#pragma omp parallel for
	for (int i = 0; i < descp_src.size(); i++) {
		// Iniciando Keypoints e Descritores atuais
		std::vector<cv::KeyPoint> kpsrc;
		cv::Mat  dsrc;

		// Ler a imagem inicial
		cv::Mat imsrc = cv::imread(imagens_src[i], cv::IMREAD_COLOR);

		// Descritores SIFT calculados
		cv::Ptr<cv::xfeatures2d::SIFT> sift = cv::xfeatures2d::SIFT::create();

		sift->detectAndCompute(imsrc, cv::Mat(), kpsrc, dsrc);
		// Calculando somatorio para cada linha de descritores
		cv::Mat dsrcsum;

		reduce(dsrc, dsrcsum, 1, CV_16UC1);

		////RootSIFT
		// Normalizando e passando raiz em cada elementos de linha nos descritores da src
#pragma omp parallel for
		for (int i = 0; i < dsrc.rows; i++) {
			for (int j = 0; j < dsrc.cols; j++) {
				dsrc.at<float>(i, j) = sqrt(dsrc.at<float>(i, j) / (dsrcsum.at<float>(i, 0) + std::numeric_limits<float>::epsilon()));
			}
		}

		kpts_src[i] = kpsrc;
		// Salvando no vetor de cada um os descritores
		descp_src[i] = dsrc;


	}
}
void Utils::filtrar_matches_keypoints_repetidos(std::vector<cv::KeyPoint> &kt, std::vector<cv::KeyPoint> &ks, std::vector<cv::DMatch> &m) {
	// Matriz de bins para keypoints de target e source

	const int w = 1280 / 5, h = 720 / 5;

	std::vector<std::vector<cv::DMatch>>matriz_matches[w];

#pragma omp parallel for
	for (int i = 0; i < w; i++) {
		matriz_matches[i].resize(h);
	}

	// Itera sobre os matches pra colocar eles nos bins certos

	for (int i = 0; i < m.size(); i++) {
		cv::KeyPoint ktt = kt[m[i].trainIdx];
		int u = ktt.pt.x / 5, v = ktt.pt.y / 5;
		matriz_matches[u][v].push_back(m[i]);
	}
	// Vetor auxiliar de matches que vao passar no teste de melhor distancia
	std::vector<cv::DMatch> boas_matches;
	// Procurando na matriz de matches
	for (int i = 0; i < w; i++) {
		for (int j = 0; j < h; j++) {
			if (matriz_matches[i][j].size() > 0) {
				// Se ha matches e for so uma, adicionar ela mesmo
				if (matriz_matches[i][j].size() == 1) {
					boas_matches.push_back(matriz_matches[i][j][0]);
				}
				else { // Se for mais de uma comparar a distancia com as outras
					cv::DMatch mbest = matriz_matches[i][j][0];
					for (int k = 1; k < matriz_matches[i][j].size(); k++) {
						if (matriz_matches[i][j][k].distance < mbest.distance)
							mbest = matriz_matches[i][j][k];
					}
					// Adicionar ao vetor a melhor opcao para aquele bin
					boas_matches.push_back(mbest);
				}
			}
			matriz_matches[i][j].clear(); // Ja podemos limpar aquele vetor, ja trabalhamos
		}
	}
	m = boas_matches;
	// Fazer o mesmo agora para as matches que sobraram e kpts da src
	// Itera sobre os matches pra colocar eles nos bins certos
	for (int i = 0; i < boas_matches.size(); i++) {
		cv::KeyPoint kst = ks[m[i].queryIdx];
		int u = kst.pt.x / 5, v = kst.pt.y / 5;
		matriz_matches[u][v].push_back(m[i]);
	}
	// Vetor auxiliar de matches que vao passar no teste de melhor distancia
	std::vector<cv::DMatch> otimas_matches;
	// Procurando na matriz de matches
	for (int i = 0; i < w; i++) {
		for (int j = 0; j < h; j++) {
			if (matriz_matches[i][j].size() > 0) {
				// Se ha matches e for so uma, adicionar ela mesmo
				if (matriz_matches[i][j].size() == 1) {
					otimas_matches.push_back(matriz_matches[i][j][0]);
				}
				else { // Se for mais de uma comparar a distancia com as outras
					cv::DMatch mbest = matriz_matches[i][j][0];
					for (int k = 1; k < matriz_matches[i][j].size(); k++) {
						if (matriz_matches[i][j][k].distance < mbest.distance)
							mbest = matriz_matches[i][j][k];
					}
					// Adicionar ao vetor a melhor opcao para aquele bin
					otimas_matches.push_back(mbest);
				}
			}
			matriz_matches[i][j].clear(); // Ja podemos limpar aquele vetor, ja trabalhamos
		}
	}

	// Retornando as matches que restaram
	m = otimas_matches;
}

// Encontrar as correspondencias entre as imagens e suas vizinhas 
std::vector<std::vector<std::vector<cv::KeyPoint>>> Utils::sift_matches_matrix_encontrar_melhor(std::vector<std::vector<  std::vector<cv::DMatch> >> matriz_matches, std::vector<cv::Mat>  descp_src, std::vector< std::vector<cv::KeyPoint> >  kpts_src, std::vector<std::string> imagens_src, std::vector<std::vector<int>> &indices) {
	// Matcher de FLANN
	srand(time(NULL));
	cv::Ptr<cv::DescriptorMatcher> matcher;
	matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);

	std::vector<std::vector<int>> filter_zero;
	filter_zero.resize(imagens_src.size());
	for (int frame0 = 0; frame0 < imagens_src.size(); frame0++) {

		for (int frame1 = 0; frame1 < indices[frame0].size(); frame1++)
		{
			std::vector<std::vector<cv::DMatch>> matches;
			std::vector<cv::DMatch> good_matches;
			if (!descp_src[frame0].empty() && !descp_src[indices[frame0][frame1]].empty()) {
				matcher->knnMatch(descp_src[frame0], descp_src[indices[frame0][frame1]], matches, 2);

				for (size_t k = 0; k < matches.size(); k++)
				{
					if (matches.at(k).size() >= 2)
					{
						if (matches.at(k).at(0).distance < 0.7*matches.at(k).at(1).distance) // Se e bastante unica frente a segunda colocada
							good_matches.push_back(matches.at(k).at(0));
					}
				}
				if (good_matches.size() > 0)
				{
					// Filtrar keypoints repetidos
					Utils::filtrar_matches_keypoints_repetidos(kpts_src[indices[frame0][frame1]], kpts_src[frame0], good_matches);
					// Anota quantas venceram nessa combinacao
					matriz_matches.at(frame0).at(frame1) = good_matches;

				}
			}
		}
	}
	bool debug = false;
	std::vector<std::vector<std::vector<cv::KeyPoint>>> bestKey;
	std::vector<cv::DMatch> best_matches;
	std::vector<cv::KeyPoint> best_kptgt, best_kpsrc;
	bestKey.resize(imagens_src.size());

	for (int frame0 = 0; frame0 < imagens_src.size(); frame0++)
	{

		for (int frame1 = 0; frame1 < indices[frame0].size(); frame1++)
		{

			std::vector<cv::KeyPoint> curr_kpts_tgt = kpts_src[indices[frame0][frame1]], curr_kpts_src = kpts_src[frame0];
			best_matches = matriz_matches.at(frame0).at(frame1);

			for (auto m : best_matches) {
				best_kptgt.emplace_back(curr_kpts_tgt[m.trainIdx]);
				best_kpsrc.emplace_back(curr_kpts_src[m.queryIdx]);
			}

			// Converter os pontos para o formato certo
			std::vector<cv::Point2d> kptgt(best_kptgt.size()), kpsrc(best_kpsrc.size());
#pragma omp parallel for
			for (int i = 0; i < best_kptgt.size(); i++) {
				kptgt[i] = best_kptgt[i].pt;
				kpsrc[i] = best_kpsrc[i].pt;
			}
			if (best_matches.size() > 20)
			{
				// Calcular matriz fundamental
				cv::Mat F = findFundamentalMat(kpsrc, kptgt); // Transformacao da src para a tgt
				// Calcular pontos que ficam por conferencia da matriz F
				Eigen::Matrix3d F_;
				cv::cv2eigen(F, F_);
				std::vector<cv::Point2d> tempt, temps;
				std::vector<int> indices_inliers;
				for (int i = 0; i < kpsrc.size(); i++) {
					Eigen::Vector3d pt{ kptgt[i].x, kptgt[i].y, 1 }, ps = { kpsrc[i].x, kpsrc[i].y, 1 };
					Eigen::MatrixXd erro = pt.transpose()*F_*ps;
					if (abs(erro(0, 0)) < 0.2) {
						tempt.push_back(kptgt[i]); temps.push_back(kpsrc[i]);
						indices_inliers.push_back(i);
					}
				}
				kpsrc = temps; kptgt = tempt;

				// Segue so com os inliers dentre os best_kpts
				std::vector<cv::KeyPoint> temp_kptgt, temp_kpsrc;
				std::vector<cv::DMatch> temp_matches;
				for (auto i : indices_inliers) {
					temp_kptgt.push_back(best_kptgt[i]); temp_kpsrc.push_back(best_kpsrc[i]);
					temp_matches.push_back(best_matches[i]);
				}
				best_kptgt = temp_kptgt; best_kpsrc = temp_kpsrc;
				best_matches = temp_matches;
			}
			if (debug) {
				cv::Mat im1 = cv::imread(imagens_src[indices[frame0][frame1]], cv::IMREAD_COLOR);
				cv::Mat im2 = cv::imread(imagens_src[frame0], cv::IMREAD_COLOR);
				for (int i = 0; i < best_kpsrc.size(); i++) {
					int r = rand() % 255, b = rand() % 255, g = rand() % 255;
					circle(im1, cv::Point(best_kptgt[i].pt.x, best_kptgt[i].pt.y), 8, cv::Scalar(r, g, b), cv::FILLED, cv::LINE_8);
					circle(im2, cv::Point(best_kpsrc[i].pt.x, best_kpsrc[i].pt.y), 8, cv::Scalar(r, g, b), cv::FILLED, cv::LINE_8);
				}
				imwrite("C:/dataset3/im_tgt1.png", im1);
				imwrite("C:/dataset3/im_src1.png", im2);

			}

			int rand0, rand1, rand2;
			std::vector<cv::KeyPoint> best_kptgt_org, best_kpsrc_org;
			float scale = 0.95;
			int ncorr = best_kpsrc.size();
			int number_of_trial = ncorr * 100;
			std::vector<std::pair<int, int> > corres_tuple;
			int cnt = 0;
			int i;
			int tuple_max_cnt_ = 300;
			for (i = 0; i < number_of_trial; i++)
			{
				rand0 = rand() % ncorr;
				rand1 = rand() % ncorr;
				rand2 = rand() % ncorr;


				Eigen::Vector3f pti0, pti1, pti2;
				pti0 << best_kpsrc[rand0].pt.x, best_kpsrc[rand0].pt.y, 1;
				pti1 << best_kpsrc[rand1].pt.x, best_kpsrc[rand1].pt.y, 1;
				pti2 << best_kpsrc[rand2].pt.x, best_kpsrc[rand2].pt.y, 1;

				float li0 = (pti0 - pti1).norm();
				float li1 = (pti1 - pti2).norm();
				float li2 = (pti2 - pti0).norm();

				Eigen::Vector3f ptj0, ptj1, ptj2;

				ptj0 << best_kptgt[rand0].pt.x, best_kptgt[rand0].pt.y, 1;
				ptj1 << best_kptgt[rand1].pt.x, best_kptgt[rand1].pt.y, 1;
				ptj2 << best_kptgt[rand2].pt.x, best_kptgt[rand2].pt.y, 1;

				float lj0 = (ptj0 - ptj1).norm();
				float lj1 = (ptj1 - ptj2).norm();
				float lj2 = (ptj2 - ptj0).norm();

				if ((li0 * scale < lj0) && (lj0 < li0 / scale) &&
					(li1 * scale < lj1) && (lj1 < li1 / scale) &&
					(li2 * scale < lj2) && (lj2 < li2 / scale))
				{
					best_kpsrc_org.push_back(best_kpsrc[rand0]);
					best_kpsrc_org.push_back(best_kpsrc[rand1]);
					best_kpsrc_org.push_back(best_kpsrc[rand2]);
					best_kptgt_org.push_back(best_kptgt[rand0]);
					best_kptgt_org.push_back(best_kptgt[rand1]);
					best_kptgt_org.push_back(best_kptgt[rand2]);
					cnt++;
				}
				if (cnt >= tuple_max_cnt_)
					break;

			}

			if (debug) {
				cv::Mat im1 = cv::imread(imagens_src[indices[frame0][frame1]], cv::IMREAD_COLOR);
				cv::Mat im2 = cv::imread(imagens_src[frame0], cv::IMREAD_COLOR);
				for (int i = 0; i < best_kptgt_org.size(); i++) {
					int r = rand() % 255, b = rand() % 255, g = rand() % 255;
					circle(im1, cv::Point(best_kptgt_org[i].pt.x, best_kptgt_org[i].pt.y), 8, cv::Scalar(r, g, b), cv::FILLED, cv::LINE_8);
					circle(im2, cv::Point(best_kpsrc_org[i].pt.x, best_kpsrc_org[i].pt.y), 8, cv::Scalar(r, g, b), cv::FILLED, cv::LINE_8);
				}
				imwrite("C:/dataset3/im_tgt2.png", im1);
				imwrite("C:/dataset3/im_src2.png", im2);

			}

			printf("%d tuples (%d trial, %d actual).\n", cnt, number_of_trial, i);
			best_kptgt.clear();
			best_kpsrc.clear();

			std::vector<int> v(best_kptgt_org.size()); // vector with 100 ints.
			std::iota(std::begin(v), std::end(v), 0); // Fill with 0, 1, ..., 99.
			//std::random_shuffle(v.begin(), v.end());
			int t = best_kpsrc_org.size();

			t = (t < 50) ? t : 50;

			for (int i = 0; i < t; i++) {
				best_kpsrc.push_back(best_kpsrc_org[v[i]]);
				best_kptgt.push_back(best_kptgt_org[v[i]]);
			}
			if (t > 0) {
				filter_zero[frame0].push_back(indices[frame0][frame1]);
				bestKey[frame0].push_back(best_kpsrc);
				bestKey[frame0].push_back(best_kptgt);
			}

			best_kptgt.clear();
			best_kpsrc.clear();
			best_kpsrc_org.clear();
			best_kptgt_org.clear();
		}


	}
	indices.clear();
	indices = filter_zero;

	return bestKey;
}
// random number between 0 and 1
double Utils::GenerateRandomNumber() {
	static bool init = false;
	if (!init) {
		init = true;
		srand(time(NULL));
	}
	return (double)rand() / RAND_MAX;
}
// create 2d long double array, its value is between (0,1) * (ub-lb)+lb

double** Utils::Create2DRandomArray(unsigned int rowCount, unsigned int columnCount, std::vector<double> lb, std::vector<double> up)
{

	double **array = new double *[rowCount];

#pragma omp parallel for
	for (int y = 0; y < rowCount; y++)
	{
		array[y] = new double[columnCount];
		if (y == 0) {
			for (int x = 0; x < columnCount; x++) {

				array[y][x] = lb[x] + 3;
				array[y][x + 1] = lb[x + 1] + 3;
				array[y][x + 2] = lb[x + 2] + 3;
				array[y][x + 3] = lb[x + 3] + 3;
				array[y][x + 4] = lb[x + 4] + 3;
				array[y][x + 5] = lb[x + 5] + 3;
				x = x + 5;

			}
		}
		else {
			// randomize data and apply between (lower,upper) bound
			for (int x = 0; x < columnCount; x++) {
				array[y][x] = lb[x] + (up[x] - lb[x]) * GenerateRandomNumber();

			}
		}
	}


	return array;

}
// create 1D long double array with value zero
double* Utils::Create1DZeroArray(unsigned int columnCount) {
	double *array = new double[columnCount];
	std::fill_n(array, columnCount, 0.0);
	return array;
}
void Utils::Clip1DArray(double array[], unsigned int columnCount, Boundaries boundaries[]) {

	for (int column = 0; column < columnCount; column++) {

		double value = array[column];
		if (value < boundaries[column].lowerBound) {

			array[column] = boundaries[column].lowerBound;

		}
		if (value > boundaries[column].upperBound) {

			array[column] = boundaries[column].upperBound;

		}
	}
}
