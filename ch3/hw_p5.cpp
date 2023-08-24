#include <iostream>
#include <Eigen/Core>

using namespace std;
using namespace Eigen;

#define MATRIX_SIZE 4

int main(int argc, char **argv) {
    //生成一个 MATRIX_SIZE × MATRIX_SIZE的随机矩阵
    Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN= MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    cout<<"原始矩阵： \n"<<matrix_NN<<endl;
    cout<<endl;

    //提取左上角3×3块, 用 Identity 赋值
    matrix_NN.block(0,0,3,3) = Matrix3d::Identity();
    cout<<"左上3x3 Identity: \n"<<matrix_NN<<endl;
    
    return 0;
}