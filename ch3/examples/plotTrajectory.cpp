#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <unistd.h>

using namespace std;
using namespace Eigen;

// path to trajectory file
string trajectory_file = "../trajectory.txt";

//aligned_allocator用来分配内存空间
void DrawTrajectory(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>>);

int main(int argc, char **argv){
    vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses;
    ifstream fin(trajectory_file);
    if (!fin){
        cout << "cannot find trajectory file at "<< trajectory_file << endl;
        return 1;
    }
    // eof() 判断文件是否为空，或者判断其是否读到文件结尾。
    while(!fin.eof()){
        double time, tx, ty, tz, qx, qy, qz, qw; // qw 为四元数的实部
        fin>>time>>tx>>ty>>tz>>qx>>qy>>qz>>qw;

        // 欧式变换也称为等距变换(Isometry Transform); 
        // 所以欧式变换矩阵使用 Isometry
        Isometry3d Twr(Quaterniond(qx, qy, qz, qw));
        cout<<Twr.matrix()<<endl;
        Twr.pretranslate(Vector3d(tx, ty, tz));
        cout<<Twr.matrix()<<endl;
        poses.push_back(Twr);
    }
    cout <<"real total "<<poses.size()<<" poses entries "<<endl;
    
    // draw trajectory in pangolin
    DrawTrajectory(poses);
    return 0;
}

// create pangolin window and plot the trajectory
void DrawTrajectory(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses)
{
    // 创建一个名为 "trajectory viewer" GUI图形界面窗口，大小为1024 * 768
    pangolin::CreateWindowAndBind( "trajectory viewer", 1024, 768);
    //glEnable()用于启用各种功能。功能由参数决定。GL_DEPTH_TEST启用深度测试。
    glEnable(GL_DEPTH_TEST);
    // GL_BLEND启用颜色混合,后面跟着使用glBlendFunc()。
    glEnable(GL_BLEND);
    // 启动混合
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // 参数依次为相机所在的位置，以及相机所看的视点位置（一般会设置在原点）
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );
    // 创建视角窗口
    pangolin::View &d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0,1.0,0.0,1.0,-1024.0f/768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));
    
    // ShouldQuit() 检测是否关闭OpenGL 窗口
    while(pangolin::ShouldQuit()==false){
        // 清空颜色和深度缓存
        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
        // 启动相机
        d_cam.Activate(s_cam);
        // 背景设置为白色，glClearColor(red, green, blue, alpha），数值范围(0, 1)
        glClearColor(1.0f,1.0f,1.0f,1.0f); 
        // 设置线宽
        glLineWidth(2);
        for(size_t i = 0; i < poses.size(); i++)
        {
            //画每个位姿的三个坐标轴
            //取出平移向量
            Vector3d Ow=poses[i].translation();
            // 画出X轴，0.1是缩小0.1倍。这个地方有点小难理解，最好写程序测试一下，此处是旋转矩阵T*(1,0,0),
            // 结果是旋转后的X轴位置.
            Vector3d Xw=poses[i]*(0.1*Vector3d(1,0,0));
            //画出Y轴
            Vector3d Yw=poses[i]*(0.1*Vector3d(0,1,0));
            //画出Z轴
            Vector3d Zw=poses[i]*(0.1*Vector3d(0,0,1));
            //开始画线,GL_LINES：线,GL_POINTS：点
            glBegin(GL_LINES);
            //绘制线的颜色
            glColor3f(1.0,0.0,0.0);
            //设置线起始点
            glVertex3d(Ow[0],Ow[1],Ow[2]);
            //设置线结束点
            glVertex3d(Xw[0],Xw[1],Xw[2]);
            glColor3f(0.0,1.0,0.0);
            glVertex3d(Ow[0],Ow[1],Ow[2]);
            glVertex3d(Yw[0],Yw[1],Yw[2]);
            glColor3f(0.0,0.0,1.0);
            glVertex3d(Ow[0],Ow[1],Ow[2]);
            glVertex3d(Zw[0],Zw[1],Zw[2]);
            glEnd();
            //结束画线
        }
        //画出连线（白线：运动轨迹）
        for(size_t i=0;i<poses.size();i++)
        {
            glColor3f(1.0,1.0,1.0);
            glBegin(GL_LINES);
            auto p1=poses[i],p2=poses[i+1];
            glVertex3d(p1.translation()[0],p1.translation()[1],p1.translation()[2]);
            glVertex3d(p2.translation()[0],p2.translation()[1],p2.translation()[2]);
            glEnd();
        }
        //运行帧循环以推进窗口事件,在绘制完成后，需要使用FinishFrame命令刷新视窗。
        pangolin::FinishFrame();
        usleep(5000);//sleep 5 ms

    }
}