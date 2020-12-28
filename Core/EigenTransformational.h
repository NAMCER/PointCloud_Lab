#pragma once
#include "global.h"

namespace Transformational
{
	/*\
	//旋转矩阵（3X3）:Eigen::Matrix3d
	//旋转向量（3X1）:Eigen::AngleAxisd
	//四元数（4X1）:Eigen::Quaterniond
	//平移向量（3X1）:Eigen::Vector3d
	//变换矩阵（4X4）:Eigen::Isometry3d
	*/

	void E_Transformational()
	{
		Eigen::AngleAxisd t_V(M_PI / 4, Eigen::Vector3d(0, 0, 1));
		Eigen::Matrix3d t_R = t_V.matrix();
		Eigen::Quaterniond t_Q(t_V);
		std::cout.precision(3);
		std::cout << "Translate vector:\n" << t_V.matrix() << std::endl;
		std::cout << "Translate VECTOR convert MATRIX:\n " << t_R << std::endl;
		std::cout << "Translate vector convert Quaternion:\n" << t_Q.matrix() << std::endl;
	
		//1.使用旋转的角度和旋转轴向量（此向量为单位向量）来初始化角轴
		Eigen::AngleAxisd V1(M_PI / 4, Eigen::Vector3d(0, 0, 1));
		std::cout << "Rotation_vector1:\n" << V1.matrix() << std::endl;
		
		//2.使用旋转矩阵转旋转向量的方式
		//2.1 使用旋转向量的fromRotationMatrix()函数来对旋转向量赋值（注意此方法为旋转向量独有,四元数没有）
		Eigen::AngleAxisd V2;
		V2.fromRotationMatrix(t_R);
		std::cout << "Rotation_vector2:\n" << V2.matrix() << std::endl;
		//2.2 直接使用旋转矩阵来对旋转向量赋值
		Eigen::AngleAxisd V3;
		V3 = t_R;
		std::cout << "Rotation_vector3:\n" << V3.matrix() << std::endl;
		//2.3 使用旋转矩阵来对旋转向量进行初始化
		Eigen::AngleAxisd V4(t_R);
		std::cout << "Rotation_vector4\n" << V4.matrix() << std::endl;
		
		//3. 使用四元数来对旋转向量进行赋值
		//3.1 直接使用四元数来对旋转向量赋值
		Eigen::AngleAxisd V5;
		V5 = t_Q;
		std::cout << "Rotation_vector5:\n" << V5.matrix() << std::endl;
		//3.2 使用四元数来对旋转向量进行初始化
		Eigen::AngleAxisd V6(t_Q);
		std::cout << "Rotation_vector6:\n" << V6.matrix() << std::endl;
		std::cout<<"/**************************************************/"<<std::endl;
		
		//对四元数赋值的三大种方法（注意Eigen库中的四元数前三维是虚部,最后一维是实部）
		 //1.使用旋转的角度和旋转轴向量（此向量为单位向量）来初始化四元数,
		//即使用q=[cos(A/2),n_x*sin(A/2),n_y*sin(A/2),n_z*sin(A/2)]
		Eigen::Quaterniond Q1(cos((M_PI / 4) / 2), 0 * sin((M_PI / 4) / 2), 0 * sin((M_PI / 4) / 2), 1 * sin((M_PI / 4) / 2));//以（0,0,1）为旋转轴，旋转45度
		//out show method: 1
		std::cout << "Quaternion1:\n" << Q1.coeffs() << std::endl;
		std::cout << "//////////////////////" << std::endl;
		//out show method: 2
		std::cout << Q1.x() << std::endl;
		std::cout << Q1.y() << std::endl;
		std::cout << Q1.z() << std::endl;
		std::cout << Q1.w() << std::endl;

		//2. 使用旋转矩阵转四元數的方式
		//2.1 直接使用旋转矩阵来对旋转向量赋值
		Eigen::Quaterniond Q2;
		Q2 = t_R;
		std::cout << "Quaternion2" << std::endl << Q2.coeffs() << std::endl;
		//2.2 使用旋转矩阵来对四元數进行初始化
		Eigen::Quaterniond Q3(t_R);
		std::cout << "Quaternion3" << std::endl << Q3.coeffs() <<std::endl;

		//3. 使用旋转向量对四元数来进行赋值
		//3.1 直接使用旋转向量对四元数来赋值
		Eigen::Quaterniond Q4;
		Q4 = t_V;
		std::cout << "Quaternion4" << std::endl << Q4.coeffs() << std::endl;

		//3.2 使用旋转向量来对四元数进行初始化
		Eigen::Quaterniond Q5(t_V);
		std::cout << "Quaternion5" << std::endl << Q5.coeffs() << std::endl;


		std::cout << "/**************************************************/" << std::endl;
		//对旋转矩阵赋值的三大种方法
		//1.使用旋转矩阵的函数来初始化旋转矩阵
		Eigen::Matrix3d R1 = Eigen::Matrix3d::Identity();
		std::cout << "Rotation_matrix1\n" << R1 << std::endl;

		//2. 使用旋转向量转旋转矩阵来对旋转矩阵赋值
		Eigen::Matrix3d R2;
		R2 = t_V.matrix();
		std::cout << "Rotation_matrix2" << std::endl << R2 << std::endl;

		//2.2 使用旋转向量的成员函数toRotationMatrix()来对旋转矩阵赋值
		Eigen::Matrix3d R3;
		R3 = t_V.toRotationMatrix(); 
		std::cout << "Rotation_matrix3" << std::endl << R3 << std::endl;

		//3. 使用四元数转旋转矩阵来对旋转矩阵赋值
		Eigen::Matrix3d R4;
		R4 = t_Q.matrix();
		std::cout << "Rotation_matrix4" << std::endl << R4 << std::endl;

		//3.2 使用四元数的成员函数toRotationMatrix()来对旋转矩阵赋值
		Eigen::Matrix3d R5;
		R5 = t_Q.toRotationMatrix();
		std::cout << "Rotation_matrix5" << std::endl << R5 << std::endl;
	}

}