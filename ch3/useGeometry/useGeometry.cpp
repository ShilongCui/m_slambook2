#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace Eigen;
// 本程序演示了 Eigen 几何模块的使用方法

int main(int argc, char **argv){
    // Eigen/Geomatry 模块提供了各种旋转和平移的表示
    // 3D 旋转矩阵直接使用 Matrix3d 或者 Matrix3f， （这里的 d 表示double，f: float）

    /*  定义旋转向量，然后转换为旋转矩阵  */
    Matrix3d rotation_matrix = Matrix3d::Identity();
    // 旋转向量使用AngleAxis，它底层不直接是Matrix，但是运算可以当作矩阵（因为重载了运算符）
    AngleAxisd rotation_vector(M_PI / 4, Vector3d(0, 0, 1));  // 沿Z轴旋转45度
    cout.precision(3);
    // cout <<" rotation vector = \n" = rotation_vector << endl;
    cout << "rotation matrix =\n" << rotation_vector.matrix() <<endl;  // 用 Matrix()转换成矩阵
    // 也可以直接赋值
    rotation_matrix = rotation_vector.toRotationMatrix();

    /*  用AngleAxis可以进行坐标变换  */ 
    Vector3d v(1,0,0);
    Vector3d v_rotated = rotation_vector * v;
    cout <<"(1, 0, 0) after rotation(by angle axis) = " << v_rotated.transpose() << endl;
    // 或者用旋转矩阵
    v_rotated = rotation_matrix * v;
    cout <<"(1,0,0) after rotation (by matrix) = " << v_rotated.transpose() <<endl;

    /*  欧拉角：可以将旋转矩阵直接转换成欧拉角  */ 
    Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0); //ZYX 顺序，即roll pitch yaw 顺序
    cout <<"yaw picth roll = " << euler_angles.transpose() <<endl;

    /*  欧式变换矩阵使用Eigen::Isometry  */
    Isometry3d T = Isometry3d::Identity();    // 初始化为单位矩阵， 虽然称为3D，实质上是4*4 的矩阵
    T.rotate(rotation_vector);                // 按照rotation_vector 进行旋转。
    T.pretranslate(Vector3d(1, 3, 4));        // 把平移量设置成（1，3，4）
    cout << "Transform matrix = \n" << T.matrix() << endl;

    // 用变换矩阵进行坐标变换
    Vector3d v_transformed = T * v;      // 相当于 R*v + t
    cout << "v transformed = " << v_transformed.transpose() << endl;

    /*  四元数  */
    // 可以直接把 AngleAxis 赋值给四元数，反之亦然。
    Quaterniond q = Quaterniond(rotation_vector);
    cout << "quanterniond from rotation vector = " << q.coeffs().transpose() 
        << endl; // coeffs的顺序是（x, y ,z, w）,w为实部，前三者为虚部。
    
    // 也可以把旋转矩阵赋给它
    q = Quaterniond(rotation_matrix);
    cout << "quaternion from rotation matrix = " << q.coeffs().transpose() << endl;

    // 使用四元数旋转一个向量，使用重载的乘法即可
    v_rotated = q * v;   // 数学上是 qvq^{-1}
    cout << "(1,0,0) after rotation = " << v_rotated.transpose() << endl;

    // 用常规向量乘法表示， 则计算如下
    cout << "should be equal to " << (q * Quaterniond(0,1,0,0) * q.inverse()).coeffs().transpose() << endl;

    return 0;

}