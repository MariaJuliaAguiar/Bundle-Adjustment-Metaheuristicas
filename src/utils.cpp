#include "utils.hpp"


Utils* utils = nullptr;


// Função para encontrar a posição vinda do robo que sera necessaria para encontrar o chute inicial e os limites dos parametros
std::vector<std::vector<float>>  Utils::lerSFM(std::string pasta, double& fx, double& fy, double& cx, double& cy, std::vector<std::string>& imagens_src) {

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
	//std::fstream conv;
	std::ofstream conv;
	conv.open(pasta + "original.sfm", std::ofstream::out | std::ofstream::trunc);
	conv << linhas.size() << "\n"<<"\n";
	//conv.open(pasta+"original.sfm");
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

		conv << pasta + nome_fim << " " << ea[1] << " " << ea[2] << " " << fx << " " << fy << " " << cx << " " << cy << "\n";
	}
	conv.close();
	return pose;//roll,tilt,pan

}

// Função para encontrar os limites maximos e minimos dos parâmetros
void Utils::uplowerBound(std::vector<std::vector<float> > pose, double fx, double fy, double cx, double cy, std::vector<double>& lb, std::vector<double>& up)
{


	for (int j = 0; j < pose.size(); j++)
	{

		//	if (pose[j][1]>0) {
		//		lb.push_back(pose[j][1]*0.98);//tilt
		//		up.push_back(pose[j][1]*1.02);//tilt
		//	}
		//	else {
		//		lb.push_back(pose[j][1] * 1.02);//tilt
		//		up.push_back( pose[j][1] * 0.98);//tilt
		//	}


		//	if (pose[j][2] > 0) {
		//		lb.push_back(pose[j][2] * 0.98);//tilt
		//		up.push_back(pose[j][2] * 1.02);//tilt
		//	}
		//	else {
		//		lb.push_back(pose[j][2] * 1.02);//tilt
		//		up.push_back(pose[j][2] * 0.98);//tilt
		//	}




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

	//for (int a = initial; a < images_size; a++)
	//{

	//	indices_vizinhos[a].push_back(a + 9 >= images_size ? initial : a + 9);
	//	indices_vizinhos[a].push_back(a + 1);
	//	a = a + 9;
	//	if (a >= images_size) break;
	//	indices_vizinhos[a].push_back(a + 1 >= images_size ? initial : a + 1);
	//	indices_vizinhos[a].push_back(a - 1);
	//	/*a = a + 1;*/

	//}
	//initial = 1;
	//for (int a = initial; a < images_size; a++)
	//{

	//	indices_vizinhos[a].push_back(a + 7 >= images_size ? initial : a + 7);
	//	indices_vizinhos[a].push_back(a + 1);
	//	a = a + 7;
	//	if (a >= images_size) break;
	//	indices_vizinhos[a].push_back(a + 3 >= images_size ? initial : a + 3);
	//	indices_vizinhos[a].push_back(a - 1);
	//	a = a + 2;

	//}
	//initial = 2;
	//for (int a = initial; a < images_size; a++)
	//{

	//	indices_vizinhos[a].push_back(a + 5 >= images_size ? initial : a + 5);
	//	indices_vizinhos[a].push_back(a + 1);
	//	a = a + 5;
	//	if (a >= images_size) break;
	//	indices_vizinhos[a].push_back(a + 5 >= images_size ? initial : a + 5);
	//	indices_vizinhos[a].push_back(a - 1);
	//	a = a + 4;

	//}

	//initial = 3;
	//for (int a = initial; a < images_size; a++)
	//{

	//	indices_vizinhos[a].push_back(a + 3 >= images_size ? initial : a + 3);
	//	indices_vizinhos[a].push_back(a + 1);
	//	a = a + 3;
	//	if (a >= images_size) break;
	//	indices_vizinhos[a].push_back(a + 7 >= images_size ? initial : a + 7);
	//	indices_vizinhos[a].push_back(a - 1);
	//	a = a + 6;

	//}
	//initial = 4;
	//for (int a = initial; a < images_size; a++)
	//{

	//	indices_vizinhos[a].push_back(a + 1 >= images_size ? initial : a + 1);
	//	a = a + 1;
	//	if (a >= images_size) break;
	//	indices_vizinhos[a].push_back(a + 9 >= images_size ? initial : a + 9);
	//	a = a + 8;

	//}






	return indices_vizinhos;
}
// Encontrar as features de todas as imagens - ROOTSFIT para melhor desempenho
void Utils::calcular_features_sift(std::vector<cv::Mat>& descp_src, std::vector<std::vector<cv::KeyPoint> >& kpts_src, std::vector<std::string> imagens_src)
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
void Utils::filtrar_matches_keypoints_repetidos(std::vector<cv::KeyPoint>& kt, std::vector<cv::KeyPoint>& ks, std::vector<cv::DMatch>& m) {
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


bool lexico_compare(const cv::Point2f& p1, const cv::Point2f& p2) {
	if (p1.x < p2.x) { return true; }
	if (p1.x > p2.x) { return false; }
	return (p1.y < p2.y);
}
// Encontrar as correspondencias entre as imagens e suas vizinhas 
std::vector<std::vector<std::vector<cv::KeyPoint>>> Utils::sift_matches_matrix_encontrar_melhor(std::vector<std::vector<  std::vector<cv::DMatch> >> matriz_matches, std::vector<cv::Mat>  descp_src, std::vector< std::vector<cv::KeyPoint> >  kpts_src, std::vector<std::string> imagens_src, std::vector<std::vector<int>>& indices) {
	//Matcher de FLANN
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
						if (matches.at(k).at(0).distance < 0.7 * matches.at(k).at(1).distance) // Se e bastante unica frente a segunda colocada
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
	/*std::fstream outkeypt1; std::fstream  outkeypt2;
	outkeypt1.open("C:/dataset3/teste/key1.txt");
	outkeypt2.open("C:/dataset3/teste/key2.txt");*/
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
					Eigen::MatrixXd erro = pt.transpose() * F_ * ps;
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
			if (debug) {//plota as features nas imagens
				cv::Mat im1 = cv::imread(imagens_src[indices[frame0][frame1]], cv::IMREAD_COLOR);
				cv::Mat im2 = cv::imread(imagens_src[frame0], cv::IMREAD_COLOR);
				for (int i = 0; i < best_kpsrc.size(); i++) {
					int r = rand() % 255, b = rand() % 255, g = rand() % 255;
					circle(im1, cv::Point(best_kptgt[i].pt.x, best_kptgt[i].pt.y), 8, cv::Scalar(r, g, b), cv::FILLED, cv::LINE_8);
					circle(im2, cv::Point(best_kpsrc[i].pt.x, best_kpsrc[i].pt.y), 8, cv::Scalar(r, g, b), cv::FILLED, cv::LINE_8);
				}
				imwrite("C:/dataset3/im_tgt1.png", im1);//trocar a pasta
				imwrite("C:/dataset3/im_src1.png", im2);

			}
			//Filtragem 
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

			if (debug) {//plota as features depois da filtragem dos outliers
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

			//printf("%d tuples (%d trial, %d actual).\n", cnt, number_of_trial, i);
			best_kptgt.clear();
			best_kpsrc.clear();

			std::vector<int> v(best_kptgt_org.size());
			std::iota(std::begin(v), std::end(v), 0);


			//std::sort(XY.begin(), XY.end(), lexico_compare);
			if (frame1 == 0)
			{
				std::sort(std::begin(v), std::end(v),
					[&](int i1, int i2) {
						if (best_kpsrc_org[i1].pt.x < best_kpsrc_org[i2].pt.x) { return true; }
						if (best_kpsrc_org[i1].pt.x > best_kpsrc_org[i2].pt.x) { return false; }
						return (best_kpsrc_org[i1].pt.y < best_kpsrc_org[i2].pt.y);
					});

				v.erase(std::unique(std::begin(v), std::end(v),
					[&](int i1, int i2) {
						return ((best_kpsrc_org[i1].pt.x == best_kpsrc_org[i2].pt.x) && (best_kpsrc_org[i1].pt.y == best_kpsrc_org[i2].pt.y));
					}), std::end(v));
			}
			else {
				std::sort(std::begin(v), std::end(v),
					[&](int i1, int i2) {
						/*if (best_kpsrc_org[i1].pt.x < best_kpsrc_org[i2].pt.x) { return true; }
						if (best_kpsrc_org[i1].pt.x > best_kpsrc_org[i2].pt.x) { return false; }*/
						return (best_kpsrc_org[i1].pt.y < best_kpsrc_org[i2].pt.y);
					});

				v.erase(std::unique(std::begin(v), std::end(v),
					[&](int i1, int i2) {
						return ((best_kpsrc_org[i1].pt.x == best_kpsrc_org[i2].pt.x) && (best_kpsrc_org[i1].pt.y == best_kpsrc_org[i2].pt.y));
					}), std::end(v));



			}

			//std::random_shuffle(v.begin(), v.end());
			int t = v.size();

			t = (t < 50) ? t : 50;

			//selecionando as features das bordas - 50 features
			for (int i = v.size() - 50; i < v.size(); i++) {
				//for (int i = 0; i < t; i++) {
				best_kpsrc.push_back(best_kpsrc_org[v[i]]);
				best_kptgt.push_back(best_kptgt_org[v[i]]);

			}
			if (t > 0) {
				filter_zero[frame0].push_back(indices[frame0][frame1]);
				bestKey[frame0].push_back(best_kpsrc);
				bestKey[frame0].push_back(best_kptgt);

			}
			if (debug) {
				cv::Mat im1 = cv::imread(imagens_src[indices[frame0][frame1]], cv::IMREAD_COLOR);
				cv::Mat im2 = cv::imread(imagens_src[frame0], cv::IMREAD_COLOR);
				for (int i = 0; i < best_kptgt.size(); i++)
				{
					int r = rand() % 255, b = rand() % 255, g = rand() % 255;
					circle(im1, cv::Point(best_kptgt[i].pt.x, best_kptgt[i].pt.y), 8, cv::Scalar(r, g, b), cv::FILLED, cv::LINE_8);
					circle(im2, cv::Point(best_kpsrc[i].pt.x, best_kpsrc[i].pt.y), 8, cv::Scalar(r, g, b), cv::FILLED, cv::LINE_8);
				}
				imwrite("C:/dataset3/im_tgt3.png", im1);
				imwrite("C:/dataset3/im_src3.png", im2);

			}
			//
			//for (int i = 0; i < best_kpsrc.size(); i++) 
			//{
			//					outkeypt1 << best_kpsrc[i].pt.x << " " << best_kpsrc[i].pt.y << '\n'; // << Chega aqui ele não faz nada
			//					//cout << "? ";
			//				}
			//for (int i = 0; i < best_kptgt.size(); i++)
			//{
			//	outkeypt2 << best_kptgt[i].pt.x << " " << best_kptgt[i].pt.y << '\n'; // << Chega aqui ele não faz nada
			//	//cout << "? ";
			//}
			//outkeypt1.close();
			//outkeypt2.close();
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
	/*std::string pasta = "C:/Users/julia/Pictures/geradorartesspace/scan3/";
	double fx, fy, cx, cy;
	std::vector<std::string> imagens_src;
	std::vector<std::vector<float> > pose = Utils::lerSFM(pasta, fx, fy, cx, cy, imagens_src);*/

	double** array = new double* [rowCount]; //Primeiro individuo é o valor do robô -  

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

double** Utils::inicialization(unsigned int rowCount, unsigned int columnCount, double** inicial)
{

	double** array = new double* [rowCount];

#pragma omp parallel for
	for (int y = 0; y < rowCount; y++)
	{
		array[y] = new double[columnCount];

		for (int x = 0; x < columnCount; x++) {

			array[y][x] = inicial[y][x];


		}

	}


	return array;

}
// create 1D long double array with value zero
double* Utils::Create1DZeroArray(unsigned int columnCount) {
	double* array = new double[columnCount];
	std::fill_n(array, columnCount, 0.0);
	return array;
}
double* Utils::Create1DArray(unsigned int columnCount) {
	double* array = new double[columnCount];
	std::fill_n(array, columnCount, std::numeric_limits<double>::infinity());
	return array;
}
double** Utils::Create2DZeroArray(int search_agents, int dimension) {
	double** zeros_vector = new double* [search_agents];
#pragma omp parallel for
	for (int y = 0; y < search_agents; y++)
	{
		zeros_vector[y] = new double[dimension];

		for (int x = 0; x < dimension; x++)
		{

			zeros_vector[y][x] = 0.0;

		}
	}
	return zeros_vector;
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

void Utils::sortArr(double array[], int n, std::vector<int>& Ind_Salp_Ordenado)
{
	std::vector<std::pair<double, int> > vp;
	// Inserting element in pair vector
	// to keep track of previous indexes
	for (int i = 0; i < n; ++i)
	{
		vp.push_back(std::make_pair(array[i], i));
	}

	// Sorting pair vector
	sort(vp.begin(), vp.end());

	// Displaying sorted element
	// with previous indexes
	// corresponding to each element

	for (int i = 0; i < vp.size(); i++)
	{

		array[i] = vp[i].first;
		Ind_Salp_Ordenado[i] = vp[i].second;
	}
}

void Utils::clearResultstxt(std::string pasta)
{
	std::ofstream ofs;
	ofs.open(pasta + "convergencia_GWO.txt", std::ofstream::out | std::ofstream::trunc);
	ofs.close();
	std::ofstream ofs1;
	ofs1.open(pasta + "convergencia_BAT.txt", std::ofstream::out | std::ofstream::trunc);
	ofs1.close();
	std::ofstream ofs2;
	ofs2.open(pasta + "convergencia_AOA.txt", std::ofstream::out | std::ofstream::trunc);
	ofs2.close();
	std::ofstream ofs3;
	ofs3.open(pasta + "convergencia_SSA.txt", std::ofstream::out | std::ofstream::trunc);
	ofs3.close();
	std::ofstream ofs4;
	ofs4.open(pasta + "convergencia_PSO.txt", std::ofstream::out | std::ofstream::trunc);
	ofs4.close();



	std::ofstream ofs5;
	ofs5.open(pasta + "best_fit_GWO.txt", std::ofstream::out | std::ofstream::trunc);
	ofs5.close();
	std::ofstream ofs6;
	ofs6.open(pasta + "best_fit_BAT.txt", std::ofstream::out | std::ofstream::trunc);
	ofs6.close();
	std::ofstream ofs7;
	ofs7.open(pasta + "best_fit_AOA.txt", std::ofstream::out | std::ofstream::trunc);
	ofs7.close();
	std::ofstream ofs8;
	ofs8.open(pasta + "best_fit_SSA.txt", std::ofstream::out | std::ofstream::trunc);
	ofs8.close();
	std::ofstream ofs9;
	ofs9.open(pasta + "best_fit_PSO.txt", std::ofstream::out | std::ofstream::trunc);
	ofs9.close();

	std::ofstream ofs10;
	ofs10.open(pasta + "bests_sol_GWO.txt", std::ofstream::out | std::ofstream::trunc);
	ofs10.close();
	std::ofstream ofs11;
	ofs11.open(pasta + "bests_sol_BAT.txt", std::ofstream::out | std::ofstream::trunc);
	ofs11.close();
	std::ofstream ofs12;
	ofs12.open(pasta + "bests_sol_AOA.txt", std::ofstream::out | std::ofstream::trunc);
	ofs12.close();
	std::ofstream ofs13;
	ofs13.open(pasta + "bests_sol_SSA.txt", std::ofstream::out | std::ofstream::trunc);
	ofs13.close();
	std::ofstream ofs14;
	ofs14.open(pasta + "bests_sol_PSO.txt", std::ofstream::out | std::ofstream::trunc);
	ofs14.close();
}

//Image plots

void Utils::doTheThing(float sd, Eigen::Vector3d p2, Eigen::Vector3d p4, Eigen::Vector3d p5, Eigen::Vector3d pc, cv::Mat im, cv::Mat& img, cv::Mat im360) {
	// A partir de frustrum, calcular a posicao de cada pixel da imagem fonte em XYZ, assim como quando criavamos o plano do frustrum
	Eigen::Vector3d hor_step, ver_step; // Steps pra se andar de acordo com a resolucao da imagem
	hor_step = (p4 - p5) / float(im.cols);
	ver_step = (p2 - p5) / float(im.rows);
#pragma omp parallel for
	for (int i = 0; i < im.rows; i++) { // Vai criando o frustrum a partir dos cantos da imagem
		for (int j = 0; j < im.cols; j++) {
			Eigen::Vector3d ponto;
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
				im360.at<cv::Vec3b>(cv::Point(u, v)) = im.at<cv::Vec3b>(cv::Point(j, im.rows - 1 - i));
				img.at<cv::Vec3b>(cv::Point(u, v)) = im.at<cv::Vec3b>(cv::Point(j, im.rows - 1 - i));

			}

		}
	}
}
bool findMinMaxRows(cv::Point const& a, cv::Point const& b)
{
	return a.y < b.y;
}
bool findMinMaxcols(cv::Point const& a, cv::Point const& b)
{
	return a.x < b.x;
}
cv::Mat Utils::createMask(cv::Mat img, std::vector<std::vector<cv::Point>> contours, int k, int qnt_images_linha)
{
	//verificando os pontos pertencentes ao contorno
	std::vector<cv::Point> pts;

	for (int cC = 0; cC < contours.size(); cC++)
	{
		for (int cP = 0; cP < contours[cC].size(); cP++)
		{
			cv::Point currentContourPixel = contours[cC][cP];
			pts.push_back(currentContourPixel);

		}
	}

	//Blending Vertical

	if (k == 1000)
	{
		auto valV = std::minmax_element(pts.begin(), pts.end(), findMinMaxRows);
		int sizeV = abs(valV.first->y - valV.second->y);
#pragma omp parallel for
		for (int i = 0; i < img.cols; i++)
		{
			for (int j = valV.first->y; j < valV.second->y - sizeV / 2 - 10; j++)
			{
				cv::Vec3b color1(0, 0, 0);
				img.at< cv::Vec3b>(cv::Point(i, j)) = color1;
			}
		}
	}

	//Encontrando Pontos maximos e mininmos - linha
	auto val = std::minmax_element(pts.begin(), pts.end(), findMinMaxcols);
	int size = abs(val.first->x - val.second->x); //  tamanho

	//Blending horizontal - Possibilidades :
	// última Imagem Tem comportamento diferente
	if (k == qnt_images_linha - 1 || k == qnt_images_linha - 2)
	{
		if (k == qnt_images_linha - 2) {
			//Se tiver pedaços de imagem nos 2 extremos da imagem
			if (val.first->x == 0 && val.second->x == img.cols - 1)
			{
#pragma omp parallel for
				for (int i = val.first->x; i < img.cols / 2; i++)
				{
					for (int j = 0; j < img.rows; j++)
					{

						cv::Vec3b color1(0, 0, 0);
						img.at< cv::Vec3b>(cv::Point(i, j)) = color1;


					}
				}

				/*vector<Point>pts_cols, points;
				for (int i = val.second->x / 2; i < val.second->x; i++)
				{
					for (int j = 0; j < img.rows; j++)
					{
						if (img.at< Vec3b>(Point(i, j))[0] != 0 && img.at< Vec3b>(Point(i, j))[1] != 0 && img.at< Vec3b>(Point(i, j))[2] != 0) {

							Point p;
							p.x = i; p.y = j;
							pts_cols.push_back(p);
						}
					}
				}*/

			}
			else
			{
#pragma omp parallel for
				for (int i = val.first->x + 200; i < val.second->x + 1; i++)
				{
					for (int j = 0; j < img.rows; j++)
					{

						cv::Vec3b color1(0, 0, 0);
						img.at< cv::Vec3b>(cv::Point(i, j)) = color1;

					}
				}
			}


		}
		if (k == qnt_images_linha - 1)
		{
			//Se tiver pedaços de imagem nos 2 extremos da imagem
			if (val.first->x == 0 && val.second->x == img.cols - 1)
			{
#pragma omp parallel for
				for (int i = val.first->x; i < img.cols / 2; i++)
				{
					for (int j = 0; j < img.rows; j++)
					{

						cv::Vec3b color1(0, 0, 0);
						img.at< cv::Vec3b>(cv::Point(i, j)) = color1;


					}
				}

				std::vector<cv::Point>pts_cols, points;
				for (int i = val.second->x / 2; i < val.second->x; i++)
				{
					for (int j = 0; j < img.rows; j++)
					{
						if (img.at<cv::Vec3b>(cv::Point(i, j))[0] != 0 && img.at< cv::Vec3b>(cv::Point(i, j))[1] != 0 && img.at<cv::Vec3b>(cv::Point(i, j))[2] != 0) {

							cv::Point p;
							p.x = i; p.y = j;
							pts_cols.push_back(p);
						}
					}
				}
				//Encontrando min e max em x e y
				auto valH = minmax_element(pts_cols.begin(), pts_cols.end(), findMinMaxcols);
				auto valVert = minmax_element(pts_cols.begin(), pts_cols.end(), findMinMaxRows);

				int size1 = abs(valH.first->x - valH.second->x);
#pragma omp parallel for
				for (int i = valH.first->x; i < valH.first->x + 300; i++)
				{
					for (int j = 0; j < img.rows; j++)
					{
						cv::Vec3b color1(0, 0, 0);
						img.at< cv::Vec3b>(cv::Point(i, j)) = color1;

					}
				}
#pragma omp parallel for
				for (int i = valH.second->x - 300; i < valH.second->x; i++)
				{
					for (int j = 0; j < img.rows; j++)
					{

						cv::Vec3b color1(0, 0, 0);
						img.at<cv::Vec3b>(cv::Point(i, j)) = color1;
					}
				}
			}
			else
			{

#pragma omp parallel for
				for (int i = val.first->x; i < val.first->x + 200; i++)
				{
					for (int j = 0; j < img.rows; j++)
					{
						cv::Vec3b color1(0, 0, 0);
						img.at< cv::Vec3b>(cv::Point(i, j)) = color1;

					}
				}
#pragma omp parallel for
				for (int i = val.second->x - 200; i < val.second->x; i++)
				{
					for (int j = 0; j < img.rows; j++)
					{

						cv::Vec3b color1(0, 0, 0);
						img.at<cv::Vec3b>(cv::Point(i, j)) = color1;
					}
				}
			}


		}

		//Caso contrario tira pedaço dos dois extremos para não aparecer as bordas no blending

	}
	// Outras imagens
	if (k != qnt_images_linha - 1 && k != 1000 && k != qnt_images_linha - 2)
	{

		if (k != qnt_images_linha - 3 && val.first->x == 0 && val.second->x == img.cols - 1) {


#pragma omp parallel for
			for (int i = val.first->x + 500; i < img.cols / 2; i++)
			{
				for (int j = 0; j < img.rows; j++)
				{
					if (img.at<cv::Vec3b>(cv::Point(i, j))[0] != 0 && img.at< cv::Vec3b>(cv::Point(i, j))[1] != 0 && img.at< cv::Vec3b>(cv::Point(i, j))[2] != 0) {

						cv::Vec3b color1(0, 0, 0);
						img.at< cv::Vec3b>(cv::Point(i, j)) = color1;
					}
				}
			}


		}
		else {
			//Extremos de imagens
			std::vector<cv::Point>pts_Teste, points;
			if (val.first->x == 0 && val.second->x == img.cols - 1)
			{
#pragma omp parallel for
				for (int i = val.first->x; i < img.cols / 2; i++)
				{
					for (int j = 0; j < img.rows; j++)
					{
						if (img.at< cv::Vec3b>(cv::Point(i, j))[0] != 0 && img.at< cv::Vec3b>(cv::Point(i, j))[1] != 0 && img.at<cv::Vec3b>(cv::Point(i, j))[2] != 0) {

							cv::Vec3b color1(0, 0, 0);
							img.at< cv::Vec3b>(cv::Point(i, j)) = color1;
						}
					}
				}


				/*	vector<Point>pts_cols, points;
					for (int i = val.second->x / 2; i < val.second->x; i++)
					{
						for (int j = 0; j < img.rows; j++)
						{
							if (img.at< Vec3b>(Point(i, j))[0] != 0 && img.at< Vec3b>(Point(i, j))[1] != 0 && img.at< Vec3b>(Point(i, j))[2] != 0) {
								Point p;
								p.x = i; p.y = j;
								pts_cols.push_back(p);
							}
						}
					}*/



			}

			else
			{
				std::vector<cv::Point> pt;
				//A imagem não se encontra exatamente nos extremos mas  estão distantes
				if (size > img.cols / 2)
				{
					for (int i = val.first->x; i < img.cols / 2; i++)
					{
						for (int j = 0; j < img.rows; j++)
						{
							if (img.at< cv::Vec3b>(cv::Point(i, j))[0] != 0 && img.at< cv::Vec3b>(cv::Point(i, j))[1] != 0 && img.at< cv::Vec3b>(cv::Point(i, j))[2] != 0) {
								cv::Point p;
								p.x = i;	p.y = j;
								pt.push_back(p);

							}
						}
					}
					auto val4 = std::minmax_element(pt.begin(), pt.end(), findMinMaxRows);
					int size2 = abs(val4.first->x - val4.second->x);
#pragma omp parallel for
					for (int i = val4.first->x + size2 / 2; i < val.second->x + 1; i++)
					{
						for (int j = 0; j < img.rows; j++)
						{
							cv::Vec3b color1(0, 0, 0);
							img.at< cv::Vec3b>(cv::Point(i, j)) = color1;

						}
					}
				}
				// Imagem normal sem ser cortada - pega um pouco mais da metade e tira;
				else
				{
#pragma omp parallel for
					for (int i = val.first->x + size / 2 + 5; i < val.second->x + 1; i++)
					{
						for (int j = 0; j < img.rows; j++)
						{

							cv::Vec3b color1(0, 0, 0);
							img.at< cv::Vec3b>(cv::Point(i, j)) = color1;

						}
					}
				}
			}
		}

	}

	return img;

}
cv::Mat Utils::multiband_blending(cv::Mat a, const cv::Mat b, int k, int qnt_images_linha, int ind) {

	int level_num = 4;//numero de niveis

	std::vector <cv::Mat> a_pyramid;
	std::vector <cv::Mat> b_pyramid;
	std::vector <cv::Mat> mask;
	a_pyramid.resize(level_num);
	b_pyramid.resize(level_num);
	mask.resize(level_num);

	a_pyramid[0] = a;
	b_pyramid[0] = b;

	//Contorno imagem 1
	cv::Mat src_gray;
	cvtColor(a, src_gray, CV_BGR2GRAY);
	src_gray.convertTo(src_gray, CV_8UC3, 255);
	cv::Mat dst(src_gray.rows, src_gray.cols, CV_8UC3, cv::Scalar::all(0));
	std::vector<std::vector<cv::Point> > contours; // Vector for storing contour
	std::vector<cv::Vec4i> hierarchy;

	findContours(src_gray, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); // Find the contours in the image
#pragma omp parallel for
	for (int i = 0; i < contours.size(); i++) // iterate through each contour.
	{
		cv::Scalar color(255, 255, 255);
		drawContours(dst, contours, i, color, CV_FILLED);
	}


	cv::Mat src_gray1;
	cvtColor(b, src_gray1, CV_BGR2GRAY);
	src_gray1.convertTo(src_gray1, CV_8UC3, 255);
	cv::Mat dst1(src_gray1.rows, src_gray1.cols, CV_8UC3, cv::Scalar::all(0));
	std::vector<std::vector<cv::Point> > contours1; //
	std::vector<cv::Vec4i> hierarchy1;

	findContours(src_gray1, contours1, hierarchy1, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); // Encontrando contorno
#pragma omp parallel for
	for (int i = 0; i < contours1.size(); i++)
	{
		cv::Scalar color(255, 255, 255);
		drawContours(dst1, contours1, i, color, CV_FILLED);
	}

	//Parte comum entre as imagens
	cv::Mat out(src_gray1.rows, src_gray1.cols, CV_8UC3, cv::Scalar::all(0));
	bitwise_and(dst1, dst, out);
	//imwrite("C:/Users/julia/Desktop/dissertação/Resultados/Res/1/out.png", out);
	/////////////Contorno Parte comum
	cv::Mat src_gray3;
	//src_gray3 = out;
	cvtColor(out, src_gray3, CV_BGR2GRAY);
	src_gray3.convertTo(src_gray3, CV_8UC3, 255);
	cv::Mat dst3(src_gray3.rows, src_gray3.cols, CV_8UC3, cv::Scalar::all(0));
	std::vector<std::vector<cv::Point> > contours3; // Vector for storing contour
	std::vector<cv::Vec4i> hierarchy3;

	findContours(src_gray3, contours3, hierarchy3, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); // Find the contours in the image
#pragma omp parallel for
	for (int i = 0; i < contours3.size(); i++) // iterate through each contour.
	{
		cv::Scalar color(255, 255, 255);
		drawContours(dst3, contours3, i, color, -1, 8, hierarchy3, 0, cv::Point());
	}

	//Encontrando a máscara
	//imwrite("C:/Users/julia/Pictures/geradorartesspace/scan3/dst3.png", dst3);
	cv::Mat mask_out = Utils::createMask(dst3, contours3, k, qnt_images_linha);
	//imwrite("C:/Users/julia/Pictures/geradorartesspace/scan3/mask_out.png", mask_out);
	cv::subtract(dst, mask_out, dst);
	//imwrite("C:/Users/julia/Pictures/geradorartesspace/scan3/mask.png", dst);
	dst.convertTo(dst, CV_32FC3, 1.0 / 255.0);

	mask[0] = dst;
	contours.clear();
	contours3.clear();
	dst.release();
	dst3.release();
	
	//Filtro Gaussiano e o resultado é uma imagem reduzida com a metade do tamanho de cada dimensão

	for (int i = 1; i < level_num; ++i)
	{

		cv::Mat new_a, new_b, new_mask;

		// a imagem é inicialmente desfocada e depois reduzida
		pyrDown(a_pyramid[i - 1], new_a, cv::Size(a_pyramid[i - 1].cols / 2, a_pyramid[i - 1].rows / 2));
		pyrDown(b_pyramid[i - 1], new_b, cv::Size(a_pyramid[i - 1].cols / 2, a_pyramid[i - 1].rows / 2));
		pyrDown(mask[i - 1], new_mask, cv::Size(a_pyramid[i - 1].cols / 2, a_pyramid[i - 1].rows / 2));

		a_pyramid[i] = new_a;
		b_pyramid[i] = new_b;
		mask[i] = new_mask;
	}

	//Computando a piramide Laplaciana das imagens e da máscara
	//Expande as imagens, fazendo elas maiores de forma que seja possivel subtrai-las
	//Subtrair cada nivel da pirâmide


	for (int i = 0; i < level_num - 1; ++i) {

		cv::Mat dst_a, dst_b, new_a, new_b;

		cv::resize(a_pyramid[i + 1], dst_a, cv::Size(a_pyramid[i].cols, a_pyramid[i].rows));
		cv::resize(b_pyramid[i + 1], dst_b, cv::Size(a_pyramid[i].cols, a_pyramid[i].rows));

		cv::subtract(a_pyramid[i], dst_a, a_pyramid[i]);
		cv::subtract(b_pyramid[i], dst_b, b_pyramid[i]);
	}


	// Criação da imagem "misturada" em cada nível da piramide

	std::vector <cv::Mat> blend_pyramid;
	blend_pyramid.resize(level_num);

	for (int i = 0; i < level_num; ++i)
	{

		blend_pyramid[i] = cv::Mat::zeros(cv::Size(a_pyramid[i].cols, a_pyramid[i].rows), CV_32FC3);

		blend_pyramid[i] = a_pyramid[i].mul(mask[i]) + b_pyramid[i].mul(cv::Scalar(1.0, 1.0, 1.0) - mask[i]);


	}

	//Reconstruir a imagem completa
	//O nível mais baixo da nova pirâmide gaussiana dá o resultado final

	cv::Mat expand = blend_pyramid[level_num - 1];
	for (int i = level_num - 2; i >= 0; --i)
	{
		cv::resize(expand, expand, cv::Size(blend_pyramid[i].cols, blend_pyramid[i].rows));

		add(blend_pyramid[i], expand, expand);

	}
	a_pyramid.clear();
	b_pyramid.clear();
	mask.clear();
	blend_pyramid.clear();
	return expand;
}
void Utils::dotsFilter(cv::Mat& in) {
	// Imagem temporaria para servir de fonte, enquanto altera a imagem passada por ponteiro
	cv::Mat temp;
	in.copyTo(temp);
	// Varrer imagem de forma paralela, se achar ponto preto, tirar a media da vizinhanca nxn predefinida e trocar a cor do pixel

	const int n = 3;
	int cont = 0;
#pragma omp parallel for
	for (int u = n + 1; u < temp.cols - n - 1; u++) {
		for (int v = n + 1; v < temp.rows - n - 1; v++) {
			cv::Vec3b cor_atual = temp.at<cv::Vec3b>(cv::Point(u, v));
			// Se preto, alterar com vizinhos
			if (cor_atual[0] == 0 && cor_atual[1] == 0 && cor_atual[2] == 0)
			{
				int r = 0, g = 0, b = 0;
#pragma omp parallel for
				for (int i = u - n; i < u + n; i++) {
					for (int j = v - n; j < v + n; j++) {

						cv::Vec3b c = temp.at<cv::Vec3b>(cv::Point(i, j));
						if (c[0] != 0 && c[1] != 0 && c[2] != 0) {
							r = c[0]; g = c[1]; b = c[2];
							cont = 1;
							break;
						}

					}
				}
				cor_atual[0] = r; cor_atual[1] = g; cor_atual[2] = b;

				//Altera somente na imagem de saida
				in.at<cv::Vec3b>(cv::Point(u, v)) = cor_atual;

			}
		}
	}

	// Limpa imagem temp

	temp.release();
}
std::vector<cv::Mat> Utils::panoramicas(int dimension, std::string pasta, double* best_Gwo) {

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

		r1 = Eigen::AngleAxisd(-DEG2RAD(par[1]), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(-DEG2RAD(par[0]), Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(-DEG2RAD(0), Eigen::Vector3d::UnitZ());
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
	printf("Processando Imagem 360 final\n");
	for (int i = 0; i < nomes_imagens.size(); i++)
	{
		//printf("Processando foto %d...\n", i + 1);
		// Ler a imagem a ser usada
		cv::Mat image = cv::imread(nomes_imagens[ind[i]]);
		if (image.cols < 3)
			std::cout << ("Imagem nao foi encontrada, checar NVM ...");

		// Definir o foco em dimensoes fisicas do frustrum
		double F = R;
		double minX, minY, maxX, maxY;
		double dx = Cs[ind[i]][0] - double(image.cols) / 2, dy = Cs[ind[i]][1] - double(image.rows) / 2;
		//    double dx = 0, dy = 0;
		maxX = F * (float(image.cols) - 2 * dx) / (2.0 * foco[ind[i]][0]);
		minX = -F * (float(image.cols) + 2 * dx) / (2.0 * foco[ind[i]][0]);
		maxY = F * (float(image.rows) - 2 * dy) / (2.0 * foco[ind[i]][1]);
		minY = -F * (float(image.rows) + 2 * dy) / (2.0 * foco[ind[i]][1]);
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
		Utils::doTheThing(step_deg, p2.block<3, 1>(0, 0), p4.block<3, 1>(0, 0), p5.block<3, 1>(0, 0), pCenter.block<3, 1>(0, 0), image, im360, imagem_esferica);
		//	doTheThing2(step_deg, H, image, im360);
		//Tirar pontos pretos quando aumenta resolução
		dotsFilter(imagem_esferica);

		if (i == 0) {

			anterior.release();
			index = 0;
			anterior = imagem_esferica;
			anterior.convertTo(anterior, CV_32F, 1.0 / 255.0);

		}
		if (i > 0 && i < qnt_images_linha)
		{

			imagem_esferica.convertTo(imagem_esferica, CV_32F, 1.0 / 255.0);
			im360_parcial[0] = Utils::multiband_blending(anterior, imagem_esferica, index, qnt_images_linha, i);
			anterior = im360_parcial[0];
			imagem_esferica.release();

		}
		if (i == qnt_images_linha) {


			anterior.release();
			index = 0;
			anterior = imagem_esferica;
			anterior.convertTo(anterior, CV_32F, 1.0 / 255.0);
		}

		if (i > qnt_images_linha && i < 2 * qnt_images_linha)
		{


			imagem_esferica.convertTo(imagem_esferica, CV_32F, 1.0 / 255.0);
			im360_parcial[1] = Utils::multiband_blending(anterior, imagem_esferica, index, qnt_images_linha, i);
			anterior = im360_parcial[1];
			imagem_esferica.release();
			//imwrite(pasta + "imagem_esferica_Blending.png", im360_parcial[1] * 255);
		}
		if (i == 2 * qnt_images_linha)
		{


			anterior.release();
			index = 0;
			anterior = imagem_esferica;
			anterior.convertTo(anterior, CV_32F, 1.0 / 255.0);
		}

		if (i > 2 * qnt_images_linha && i < 3 * qnt_images_linha)
		{


			imagem_esferica.convertTo(imagem_esferica, CV_32F, 1.0 / 255.0);
			im360_parcial[2] = Utils::multiband_blending(anterior, imagem_esferica, index, qnt_images_linha, i);
			anterior = im360_parcial[2];
			//	imwrite(pasta + "imagem_esferica_Blending.png", im360_parcial[2] * 255);
			imagem_esferica.release();

		}
		if (i == 3 * qnt_images_linha) {


			anterior.release();
			index = 0;
			anterior = imagem_esferica;
			anterior.convertTo(anterior, CV_32F, 1.0 / 255.0);
		}

		if (i > 3 * qnt_images_linha && i < 4 * qnt_images_linha)
		{

			imagem_esferica.convertTo(imagem_esferica, CV_32F, 1.0 / 255.0);
			im360_parcial[3] = Utils::multiband_blending(anterior, imagem_esferica, index, qnt_images_linha, i);
			anterior = im360_parcial[3];
			imagem_esferica.release();
			//imwrite(pasta + "imagem_esferica_Blending.png", im360_parcial[3] * 255);

		}
		if (i == 4 * qnt_images_linha) {


			anterior.release();
			index = 0;
			anterior = imagem_esferica;
			anterior.convertTo(anterior, CV_32F, 1.0 / 255.0);
		}
		if (i > 4 * qnt_images_linha && i < 5 * qnt_images_linha)
		{

			imagem_esferica.convertTo(imagem_esferica, CV_32F, 1.0 / 255.0);
			im360_parcial[4] = Utils::multiband_blending(anterior, imagem_esferica, index, qnt_images_linha, i);
			anterior = im360_parcial[4];
			//	imwrite(pasta + "imagem_esferica_Blending.png", im360_parcial[4] * 255);
			imagem_esferica.release();

		}
		if (i == 5 * qnt_images_linha) {


			anterior.release();
			index = 0;
			anterior = imagem_esferica;
			anterior.convertTo(anterior, CV_32F, 1.0 / 255.0);
		}
		if (i > 5 * qnt_images_linha && i < 6 * qnt_images_linha)
		{

			imagem_esferica.convertTo(imagem_esferica, CV_32F, 1.0 / 255.0);
			im360_parcial[5] = Utils::multiband_blending(anterior, imagem_esferica, index, qnt_images_linha, i);
			anterior = im360_parcial[5];
			//	imwrite(pasta + "imagem_esferica_Blending.png", im360_parcial[5] * 255);
			imagem_esferica.release();

		}
		if (i == 6 * qnt_images_linha) {


			anterior.release();
			index = 0;
			anterior = imagem_esferica;
			anterior.convertTo(anterior, CV_32F, 1.0 / 255.0);
		}
		if (i > 6 * qnt_images_linha && i < 7 * qnt_images_linha)
		{

			imagem_esferica.convertTo(imagem_esferica, CV_32F, 1.0 / 255.0);
			im360_parcial[6] = Utils::multiband_blending(anterior, imagem_esferica, index, qnt_images_linha, i);
			anterior = im360_parcial[6];
			//	imwrite(pasta + "imagem_esferica_Blending.png", im360_parcial[6] * 255);
			imagem_esferica.release();

		}
		if (i == 7 * qnt_images_linha) {


			anterior.release();
			index = 0;
			anterior = imagem_esferica;
			anterior.convertTo(anterior, CV_32F, 1.0 / 255.0);
		}
		if (i > 7 * qnt_images_linha && i < 8 * qnt_images_linha)
		{

			imagem_esferica.convertTo(imagem_esferica, CV_32F, 1.0 / 255.0);
			im360_parcial[7] = Utils::multiband_blending(anterior, imagem_esferica, index, qnt_images_linha, i);
			anterior = im360_parcial[7];
			//	imwrite(pasta + "imagem_esferica_Blending.png", im360_parcial[6] * 255);
			imagem_esferica.release();

		}

		index++;
	} // Fim do for imagens;

	//////Resultado Final - Juntando os blendings horizontais
	cv::Mat result;
	index = 1000;
	result = im360_parcial[7];
	for (int i = 7; i > 0; i--) {

		result = Utils::multiband_blending(result, im360_parcial[i - 1], index, qnt_images_linha, i);
	}
	result.convertTo(result, CV_8UC3, 255);
#pragma omp parallel for
	for (int u = 0; u < 20; u++) {
		for (int v = 0; v < result.rows; v++) {
			result.at<cv::Vec3b>(cv::Point(u, v)) = result.at<cv::Vec3b>(cv::Point(30, v));
			result.at<cv::Vec3b>(cv::Point(result.cols - u, v)) = result.at<cv::Vec3b>(cv::Point(result.cols - 30, v));
		}
	}
	// Salvando imagem esferica final
	std::vector<cv::Mat> images_res;
	images_res.push_back(im360);
	images_res.push_back(result);
	im360.release();
	result.release();
	im360_parcial.clear();
	return images_res;


}
