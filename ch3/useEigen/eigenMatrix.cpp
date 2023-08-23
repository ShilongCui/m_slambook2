#include <iostream>
using namespace std;

#include <ctime>
//Eigen 核心部分
#include <Eigen/Core>
// 稠密矩阵的代数运算（逆，特征值等）
#include <Eigen/Dense>
using namespace Eigen;

#define MATRIX_SIZE 50

/******************************
 * 本程序演示了Eigen基本类型的使用
*******************************/

int main(int argc, char **argv){
    // Eigen 中所有的向量和矩阵都是 Eigen：：Matrix, 它是一个模版类。它的前三个参数为数据类型、行、列
    // 声明一个2*3 的float 矩阵
    Matrix<float, 2, 3> matrix_23;

    // 同时，Eigen 通过typedef提供了许多内置类型，不过底层仍是Eigen：：Matrix
    // 例如，Vector3d 实质上是 Eigen：：Matrix<double, 3, 1>，即三维向量
    Vector3d v_3d;
    // 这是一样的
    Matrix<float, 3, 1> vd_3d;

    // Matrix3d 实质上是 Eigen::Matrix<double, 3, 3>
    Matrix3d matrix_33 = Matrix3d::Zero(); //初始化为零
    // 如果不确定矩阵大小，可以使用动态大小的矩阵
    Matrix<double, Dynamic, Dynamic> matrix_dynamic;
    // 更简单的
    MatrixXd matrix_x;
    // 这种类型还有很多，我们不一一列举

    // 下面是对Eigen阵的操作
    // 输入数据（初始化）
    matrix_23 << 1, 2, 3, 4, 5, 6;
    // 输出
    cout<< "matrix 2x3 from 1 to 6: \n" << matrix_23 << endl;

    // 用（）访问矩阵中的元素
    cout<<"print matrix 2x3:" << endl;
    for (int i=0; i < 2; i++){
        for (int j = 0; j < 3; j++) cout << matrix_23(i, j) << "\t";
        cout << endl;
    }

    //矩阵和向量相乘（实际上仍是矩阵和矩阵）
    v_3d << 3, 2, 1;
    vd_3d << 4, 5, 6;

    //但是在Eigen里你不能混合两种不同类型的矩阵，像这样是错的,matrix_23 是float， v_3d是double
    // Matrix<doubel, 2, 1> result_wrong_type = matrix_23 * v_3d;
    // 应该显式转换
    Matrix<double, 2, 1> result = matrix_23.cast<double>() * v_3d;
    cout << "[1,2,3,4,5,6]*[3,2,1]:" << result.transpose() << endl;

    Matrix<float, 2, 1> result2 = matrix_23 * vd_3d;
    cout << "[1,2,3,4,5,6] * [4,5,6]:" << result2.transpose() << endl;

    // 同样，你不能搞错矩阵的维度，如下面注释的语句
    // Eigen::Matrix<double,2,3> result_wrong_dimension = matrix_23.cast<double>() * v_3d;

    /*  一些矩阵运算  */
    matrix_33 = Matrix3d::Random(); // 随机矩阵
    cout << "random matrix: \n" << matrix_33 << endl;
    cout << "transpose: \n" << matrix_33.transpose() << endl; // 转置
    cout << "sum: " << matrix_33.sum() << endl;               // 各元素求和
    cout << "trace: " << matrix_33.trace() << endl;           // 求迹
    cout << "times 10: \n" << 10 * matrix_33 << endl;         // 数乘
    cout << "inverse: \n" << matrix_33.inverse() << endl;     // 求逆
    cout << "det: " << matrix_33.determinant() << endl;       // 行列式

    // 特征值
    // 实对称矩阵可以保证对角化成功
    SelfAdjointEigenSolver<Matrix3d> eigen_solver(matrix_33.transpose() * matrix_33); // 计算
    cout << "Eigen values = \n" << eigen_solver.eigenvalues() << endl;     // 取出特征值
    cout << "Eigen vectors = \n" << eigen_solver.eigenvectors() << endl;   // 取出特征向量

    // 因为特征值一般按从小到大排列,所以col(0)就是最小特征值对应的特征向量
    Eigen::VectorXd v0 = eigen_solver.eigenvectors().col(0);
    cout << "v0: " << v0 << endl;

    // 解方程
    // 我们求解 matrix_NN * x = v_Nd 方程
    // N的 大 小 在 前 边 的 宏 里 定 义 ，它 由 随 机 数 生 成
    // 直 接 求 逆 自 然 是 最 直 接 的 ，但 是 运 算 量 大

    Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN
        = MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    matrix_NN = matrix_NN * matrix_NN.transpose();            // 保证半正定
    Matrix<double, MATRIX_SIZE, 1> v_Nd = MatrixXd::Random(MATRIX_SIZE, 1);

    clock_t time_stt = clock();   // 计时
    // 直接求逆
    Matrix<double, MATRIX_SIZE, 1> x = matrix_NN.inverse() * v_Nd;
    cout << "time of normal inverse is "
         << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
    cout << "x= " << x.transpose() << endl;

    // 通常用矩阵分解来求解，例如QR分解，速度会快很多
    time_stt = clock();   // 计时
    x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    cout << "time of QR decomposition is "
         << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
    cout << "x= " << x.transpose() << endl;

    // 对于正定矩阵，还可以用 cholesky 分解来解方程
    time_stt = clock();
    x = matrix_NN.ldlt().solve(v_Nd);
    cout << "time of ldlt decomposition is "
         << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
    cout << "x= " << x.transpose() << endl;

    return 0;
}