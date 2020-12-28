#pragma once
#include "global.h"

namespace Transformational
{
	/*\
	//��ת����3X3��:Eigen::Matrix3d
	//��ת������3X1��:Eigen::AngleAxisd
	//��Ԫ����4X1��:Eigen::Quaterniond
	//ƽ��������3X1��:Eigen::Vector3d
	//�任����4X4��:Eigen::Isometry3d
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
	
		//1.ʹ����ת�ĽǶȺ���ת��������������Ϊ��λ����������ʼ������
		Eigen::AngleAxisd V1(M_PI / 4, Eigen::Vector3d(0, 0, 1));
		std::cout << "Rotation_vector1:\n" << V1.matrix() << std::endl;
		
		//2.ʹ����ת����ת��ת�����ķ�ʽ
		//2.1 ʹ����ת������fromRotationMatrix()����������ת������ֵ��ע��˷���Ϊ��ת��������,��Ԫ��û�У�
		Eigen::AngleAxisd V2;
		V2.fromRotationMatrix(t_R);
		std::cout << "Rotation_vector2:\n" << V2.matrix() << std::endl;
		//2.2 ֱ��ʹ����ת����������ת������ֵ
		Eigen::AngleAxisd V3;
		V3 = t_R;
		std::cout << "Rotation_vector3:\n" << V3.matrix() << std::endl;
		//2.3 ʹ����ת����������ת�������г�ʼ��
		Eigen::AngleAxisd V4(t_R);
		std::cout << "Rotation_vector4\n" << V4.matrix() << std::endl;
		
		//3. ʹ����Ԫ��������ת�������и�ֵ
		//3.1 ֱ��ʹ����Ԫ��������ת������ֵ
		Eigen::AngleAxisd V5;
		V5 = t_Q;
		std::cout << "Rotation_vector5:\n" << V5.matrix() << std::endl;
		//3.2 ʹ����Ԫ��������ת�������г�ʼ��
		Eigen::AngleAxisd V6(t_Q);
		std::cout << "Rotation_vector6:\n" << V6.matrix() << std::endl;
		std::cout<<"/**************************************************/"<<std::endl;
		
		//����Ԫ����ֵ�������ַ�����ע��Eigen���е���Ԫ��ǰ��ά���鲿,���һά��ʵ����
		 //1.ʹ����ת�ĽǶȺ���ת��������������Ϊ��λ����������ʼ����Ԫ��,
		//��ʹ��q=[cos(A/2),n_x*sin(A/2),n_y*sin(A/2),n_z*sin(A/2)]
		Eigen::Quaterniond Q1(cos((M_PI / 4) / 2), 0 * sin((M_PI / 4) / 2), 0 * sin((M_PI / 4) / 2), 1 * sin((M_PI / 4) / 2));//�ԣ�0,0,1��Ϊ��ת�ᣬ��ת45��
		//out show method: 1
		std::cout << "Quaternion1:\n" << Q1.coeffs() << std::endl;
		std::cout << "//////////////////////" << std::endl;
		//out show method: 2
		std::cout << Q1.x() << std::endl;
		std::cout << Q1.y() << std::endl;
		std::cout << Q1.z() << std::endl;
		std::cout << Q1.w() << std::endl;

		//2. ʹ����ת����ת��Ԫ���ķ�ʽ
		//2.1 ֱ��ʹ����ת����������ת������ֵ
		Eigen::Quaterniond Q2;
		Q2 = t_R;
		std::cout << "Quaternion2" << std::endl << Q2.coeffs() << std::endl;
		//2.2 ʹ����ת����������Ԫ�����г�ʼ��
		Eigen::Quaterniond Q3(t_R);
		std::cout << "Quaternion3" << std::endl << Q3.coeffs() <<std::endl;

		//3. ʹ����ת��������Ԫ�������и�ֵ
		//3.1 ֱ��ʹ����ת��������Ԫ������ֵ
		Eigen::Quaterniond Q4;
		Q4 = t_V;
		std::cout << "Quaternion4" << std::endl << Q4.coeffs() << std::endl;

		//3.2 ʹ����ת����������Ԫ�����г�ʼ��
		Eigen::Quaterniond Q5(t_V);
		std::cout << "Quaternion5" << std::endl << Q5.coeffs() << std::endl;


		std::cout << "/**************************************************/" << std::endl;
		//����ת����ֵ�������ַ���
		//1.ʹ����ת����ĺ�������ʼ����ת����
		Eigen::Matrix3d R1 = Eigen::Matrix3d::Identity();
		std::cout << "Rotation_matrix1\n" << R1 << std::endl;

		//2. ʹ����ת����ת��ת����������ת����ֵ
		Eigen::Matrix3d R2;
		R2 = t_V.matrix();
		std::cout << "Rotation_matrix2" << std::endl << R2 << std::endl;

		//2.2 ʹ����ת�����ĳ�Ա����toRotationMatrix()������ת����ֵ
		Eigen::Matrix3d R3;
		R3 = t_V.toRotationMatrix(); 
		std::cout << "Rotation_matrix3" << std::endl << R3 << std::endl;

		//3. ʹ����Ԫ��ת��ת����������ת����ֵ
		Eigen::Matrix3d R4;
		R4 = t_Q.matrix();
		std::cout << "Rotation_matrix4" << std::endl << R4 << std::endl;

		//3.2 ʹ����Ԫ���ĳ�Ա����toRotationMatrix()������ת����ֵ
		Eigen::Matrix3d R5;
		R5 = t_Q.toRotationMatrix();
		std::cout << "Rotation_matrix5" << std::endl << R5 << std::endl;
	}

}