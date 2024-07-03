#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <unsupported/Eigen/NonLinearOptimization>

using namespace Eigen;

// 代价函数
struct TransformFunctor {
    const MatrixXf srcPoints;
    const MatrixXf dstPoints;

    TransformFunctor(const MatrixXf& src, const MatrixXf& dst) : srcPoints(src), dstPoints(dst) {}

    // 计算代价
    int operator()(const VectorXf& x, VectorXf& fvec) const {
        // 解析x中的参数（旋转和平移）
        Quaternionf q(x[0], x[1], x[2], x[3]);
        q.normalize(); // 保证是单位四元数
        Vector3f t(x[4], x[5], x[6]);

        // 计算代价
        for (int i = 0; i < srcPoints.cols(); ++i) {
            Vector3f srcPoint = srcPoints.col(i).head<3>();
            Vector3f dstPoint = dstPoints.col(i).head<3>();

            Vector3f transformedPoint = q * srcPoint + t;
            fvec[i] = (transformedPoint - dstPoint).squaredNorm();
        }
        return 0;
    }

    // 返回源点的数量
    int inputs() const { return 7; } // 4个四元数参数和3个平移参数
    int values() const { return srcPoints.cols(); } // 每个源点一个代价
};

// 使用LM算法估计变换矩阵
Matrix4f estimateTransformLM(const MatrixXf& srcPoints, const MatrixXf& dstPoints) {
    // 初始化参数（四元数和平移）
    VectorXf x(7);
    x << 0, 0, 0, 1, 0, 0, 0; // 初始旋转为单位四元数，平移为0

    TransformFunctor functor(srcPoints, dstPoints);
    NumericalDiff<TransformFunctor> numDiff(functor);
    LevenbergMarquardt<NumericalDiff<TransformFunctor>, float> lm(numDiff);
    lm.parameters.maxfev = 1000; // 设置最大迭代次数
    lm.minimize(x);

    // 从x中提取变换矩阵
    Quaternionf q(x[0], x[1], x[2], x[3]);
    q.normalize();
    Vector3f t(x[4], x[5], x[6]);

    Matrix4f transform = Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = q.toRotationMatrix();
    transform.block<3, 1>(0, 3) = t;

    return transform;
}
int main() {
    // 创建源点集和目标点集
    // 假设每个点集都有3个点，每个点都是一个三维坐标（x, y, z）
    MatrixXf srcPoints(3, 3);
    MatrixXf dstPoints(3, 3);

    // 初始化源点集
    srcPoints << 1.0, 2.0, 3.0,
                 4.0, 5.0, 6.0,
                 7.0, 8.0, 9.0;

    // 初始化目标点集
    dstPoints << 1.1, 2.1, 3.1,
                 4.1, 5.1, 6.1,
                 7.1, 8.1, 9.1;

    // 调用函数计算变换矩阵
    Matrix4f transform = estimateTransform(srcPoints, dstPoints);

    // 打印变换矩阵
    std::cout << "变换矩阵:\n" << transform << std::endl;

    return 0;
}