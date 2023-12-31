#include<iostream>
#include<chrono>

using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv){
    // 读取 argv[1] 指定的图像
    cv::Mat image;
    image  = cv::imread(argv[1]); 

    // 判断图像文本是否正确读取
    if(image.data == nullptr){// 数据不存在可能是文件不存在
        cerr<<"文件 "<<argv[1]<<"不存在。"<<endl;
        return 0;
    }

    // 文件顺利读取，首先输出一些基本信息
    cout<<"图像寛为："<<image.cols<<", 高为："<<image.rows
        <<", 通道数为："<<image.channels()<<endl;
    cv::imshow("image", image);
    cv::waitKey(0); //暂停程序，等待一个按键输入

    // 判断 image 的类型
    if(image.type() != CV_8UC1 && image.type() != CV_8UC3){
        // 图像类型不符合要求
        cout<<"请输入一张彩色图或灰度图。"<<endl;
        return 0;
    }

    // 遍历图像，请注意一下遍历方式也可以用于随机像素访问
    // 用std::chrono 给算法计时
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    for(size_t y = 0; y<image.rows; y++){
        // 用 cv::Mat::ptr 获得图像的行指针, row_ptr是第y行的头指针
        unsigned char *row_ptr = image.ptr<unsigned char>(y);
        for(size_t x = 0; x < image.cols; x++){
            // 访问位于x,y处的像素
            // data_ptr 指向待访问的像素数据
            unsigned char *data_ptr = &row_ptr[x * image.channels()];
            // 输出该像素的每个通道
            for(int c = 0; c != image.channels(); c++){
                
                unsigned char data = data_ptr[c]; //data 为I(x,y)第c个通道的值
                 
            }
        }
    }
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
    cout<<"遍历图像用时： "<<time_used.count()<<"秒。"<<endl;

    // 关于cv::Mat 的拷贝
    // 直接赋值并不会拷贝数据
    cv::Mat image_another = image;
    // 修改 iamge_another 会导致image发生变化
    image_another(cv::Rect(0, 0, 100, 100)).setTo(0); // 将左上角100 * 100 的块置零
    cv::imshow("image", image);
    cv::waitKey(0);

    // 使用clone函数拷贝数据
    cv::Mat image_clone  = image.clone();
    image_clone(cv::Rect(0,0,100,100)).setTo(255);
    cv::imshow("image", image);
    cv::imshow("image_clone", image_clone);
    cv::waitKey(0);

    cv::destroyAllWindows();
    return 0;
}
