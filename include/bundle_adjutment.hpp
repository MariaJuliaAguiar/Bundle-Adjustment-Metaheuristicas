#pragma once
#ifndef __BUNDLE_ADJUSTMENT__
#define __BUNDLE_ADJUSTMENT__

#include "opencv2/opencv.hpp"
#include "ceres/ceres.h"
#include "ceres/rotation.h"

// Reprojection error for bundle adjustment
struct ReprojectionError
{

	ReprojectionError(const Eigen::Vector3d& _kpts1, Eigen::Vector3d& _kpts2, int _frame0,int _frame1,int _h, int _w, double* _pose) : kpts1(_kpts1), kpts2(_kpts2), frame0(_frame0), frame1(_frame1),h(_h),w(_w) ,pose(_pose){ }

	template <typename T>
	bool operator()(const T* const camera,T* residuals) const
	{
		//// X' = R*X + t
		
		//ceres::AngleAxisRotatePoint(camera, point, X);
		Eigen::Matrix<T, 3, 3> Ri; //rotation matrix
		Eigen::Matrix<T, 3, 3> Rj;
		T camera_euler_angles[3];
		T camera_euler_angles1[3];
		camera_euler_angles[0]= -camera[0];
		camera_euler_angles[1]= -camera[ 1];
		camera_euler_angles[2] = T{0};
		camera_euler_angles1[0] = -camera[6];//-T{ pose[(frame1 * 6)] };
		camera_euler_angles1[1] = -camera[7];//- T{ pose[(frame1 * 6) + 1] };
		camera_euler_angles1[2] = T{ 0 };
		T R[9];
		T R1[9];
		/*Ri = Eigen::AngleAxisd(-DEG2RAD(camera[(frame0 * 6) + 1]), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(-DEG2RAD(camera[(frame0 * 6) ]), Eigen::Vector3d::UnitX());
		Rj = Eigen::AngleAxisd(-DEG2RAD(camera[(frame1 * 6) + 1]), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(-DEG2RAD(camera[(frame1 * 6) ]), Eigen::Vector3d::UnitX());*/
		ceres::EulerAnglesToRotationMatrix(camera_euler_angles, 3, R);
		ceres::EulerAnglesToRotationMatrix(camera_euler_angles1, 3, R1);
		Ri << R[0], R[1], R[2],
			R[3], R[4], R[5], 
			R[6], R[7], R[8];
		Rj << R1[0], R1[1], R1[2],
			R1[3], R1[4], R1[5],
			R1[6], R1[7], R1[8];
		/*cout << Rj;*/
		const T& cx1 = camera[4];//T{ pose[(frame1 * 6) + 4] }; //mera[4];
		const T& cy1 = camera[5];// T{ pose[(frame1 * 6) + 5] };//mera[5];
		const T& fx1 = camera[2];//T{ pose[(frame1 * 6) + 2] };//mera[2];
		const T& fy1 = camera[3];//T{ pose[(frame1 * 6) + 3] };//camera[3];

		const T& cx2 = camera[10];//T{ pose[(frame1 * 6) + 4] };
		const T& cy2 = camera[11];//T{ pose[(frame1 * 6) + 5] };
		const T& fx2 = camera[8]; //T{ pose[(frame1 * 6) + 2] };
		const T& fy2 = camera[9]; //T{ pose[(frame1 * 6) + 3] };
		Eigen::Matrix<T, 3, 3>  Ki, Kj;
		
		Ki(0) = fx1;
		Ki(1) = T{ 0 };
		Ki(2) = T{ 0 };
		Ki(3) = T{ 0 };
		Ki(4) = fy1;
		Ki(5) = T{ 0 };
		Ki(6) = cx1;
		Ki(7) = cy1;
		Ki(8) = T{ 1 };
		
		Kj(0) = fx2;
		Kj(1) = T{ 0 };
		Kj(2) = T{ 0 };
		Kj(3) = T{ 0 };
		Kj(4) = fy2;
		Kj(5) = T{ 0 };
		Kj(6) = cx2;
		Kj(7) = cy2;
		Kj(8) = T{ 1 };
		/*Kj << fx2, 0, cx2,
			0, fy2, cy2,
			0, 0, 1;*/
		Eigen::Matrix<T, 3, 3> H;
		Eigen::Matrix<T, 3, 3> HI;
		H = Ki * Ri.transpose()*Rj * (Kj).inverse();
		HI = Ki * Ri.transpose()*Ri * (Kj).inverse();

		T P1[3];
		P1[0] = HI(0)*kpts1(0)+ HI(3)*kpts1(1)+ HI(6)*kpts1(2);
		P1[1] = HI(1)*kpts1(0) + HI(4)*kpts1(1) + HI(7)*kpts1(2);
		P1[2] = HI(2)*kpts1(0) + HI(5)*kpts1(1) + HI(8)*kpts1(2);
		
		T P2[3];
		P2[0] = H(0)*kpts2(0) + H(3)*kpts2(1) + H(6)*kpts2(2);
		P2[1] = H(1)*kpts2(0) + H(4)*kpts2(1) + H(7)*kpts2(2);
		P2[2] = H(2)*kpts2(0) + H(5)*kpts2(1) + H(8)*kpts2(2);

		
		
		P2[0] = (P2[0] / P2[2]) + T(w / 2);
		P2[1] = (P2[1] / P2[2]) + T(h / 2);
		P2[2] = (P2[2] / P2[2]);

		P1[0] = (P1[0] / P1[2]) + T(w / 2);
		P1[1] = (P1[1] / P1[2]) + T(h / 2);
		P1[2] = (P1[2] / P1[2]);
		for (int i = 0; i < 3; i++)
		{
			if (P2[i] < T{ 0 }) {
				P2[i] = T{ 0 };
			}
		}
		if (P2[1] >= T{ 1799 })
		{
			P2[1] = T{ 1798 };
		}
		if (P2[0] >= T{ 3599 })
		{
			P2[0] = T{ 3598 };
		}
		for (int i = 0; i < 3; i++)
		{
			if (P1[i] < T{ 0 }) {
				P1[i] = T{ 0 };
			}

		}
		if (P1[1] >= T{ 1799 })
		{
			P1[1] = T{ 1798 };
		}
		if (P1[0] >= T{ 3599 })
		{
			P1[0] = T{ 3598 };
		}
		
		//// residual = x - x'
		residuals[0] = P1[0] - P2[0] ;// kpts1[0] - kpts2[0];//; T(x.x) - x_p;
		residuals[1] = P1[1] - P2[1];// kpts1[1] - kpts2[1]; //T(x.y) - y_p;

		return true;
	}

	static ceres::CostFunction* create(const Eigen::Vector3d& _kpts1, Eigen::Vector3d& _kpts2, int _frame0,int _frame1, int _h, int _w, double* _pose)
	{
		return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 12>(new ReprojectionError(_kpts1, _kpts2, _frame0, _frame1,_h,_w,_pose)));
	}

private:
	const Eigen::Vector3d kpts1;
	const Eigen::Vector3d kpts2;
	const int  frame1;
	const int  frame0;
	const int  h;
	const int  w;
	double* pose;
};

#endif // End of '__BUNDLE_ADJUSTMENT__'
