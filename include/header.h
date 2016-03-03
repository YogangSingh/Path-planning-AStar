#include "cv.h"
#include "highgui.h"
#include <math.h>
#include <string>

const std::string window_name="map";
class map_vis
{
public:
	int y_height;//图像高度
	int x_width;//图像宽度

	//小车初始坐标
	int center_x;
	int center_y;

	//显示窗的属性
	int win_w;
	int win_h;
    std::string win_name;

	IplImage* image;  // 图像帧
	
	void update_image(int x_map,int y_map,int type);//对图像帧进行更新

	void point_to_px(int h,int w,int &y,int &x);//从坐标位置转成像素位置
	map_vis();
	map_vis(int h,int w,int init_carx,int init_cary,int window_h,int window_w);
	void init(int h,int w,int init_carx,int init_cary,int window_h,int window_w);
	~map_vis();
};

//输入小车坐标系下的扫到的点的坐标
//更新image，并显示更新后的image
void map_vis::update_image(int x_map_carcord=-100000,int y_map_carcord=-100000,int type=0)//对图像帧进行更新
{
    int radius ;
    CvScalar rgb ;
    switch(type)
    {
    case 0:
        radius = 1;
        rgb = CV_RGB(0,255,0);
        break;
    case 1:
        radius = 3;
        rgb = CV_RGB(0,0,255);
        break;
    case -1:
        radius = 3;
        rgb = CV_RGB(255,0,0);
        break;
    case 2:
        radius = 2;
        rgb=CV_RGB(255,255,255);
        break;
    default:
        radius = 3;
        rgb = CV_RGB(0,0,0);
        break;
    }
	if (x_map_carcord==-100000 && y_map_carcord==-100000)//只有在构造函数中才会这样调用
	{
        cvShowImage( win_name.c_str(), image );  // 将图像显示到目标窗口中
		return;
	}
    

	int px_x=0;
	int px_y=0;
	point_to_px(y_map_carcord,x_map_carcord,px_y,px_x);//从坐标位置转成像素位置
    cvCircle( image, cvPoint(px_x,px_y) ,radius ,rgb ,1, 8, 3 );
    cvShowImage( win_name.c_str(), image );  // 将图像显示到目标窗口中
	return;
}

void map_vis::point_to_px(int h,int w,int &y,int &x)//从坐标位置转成像素位置，超出显示范围的，坐标都转成原点
{
	int map_abs_x=center_x+h;
	int map_abs_y=center_y+w;

	x=int(map_abs_x/ (double)x_width *win_w);
    y=win_h - int(map_abs_y/(double)y_height* win_h);
}
map_vis::map_vis()
{
}
void map_vis::init(int h,int w,int init_carx,int init_cary,int window_h=700,int window_w=700)
{y_height=h;
	x_width=w;

	center_x=init_carx;
	center_y=init_cary;

	win_w=window_w;
	win_h=window_h;
	win_name=window_name;//win_name赋成宏定义的window_name
    cvNamedWindow(win_name.c_str(),CV_WINDOW_AUTOSIZE);
    
	////生成的图像位深度为8，通道数为1
	image = cvCreateImage( cvSize(win_w,win_h), 8, 3 ); // 创建结果图像
	cvZero( image ); // 初始化图像为全黑
    update_image();
}
map_vis::map_vis(int h,int w,int init_carx,int init_cary,int window_h=700,int window_w=700)
{
	y_height=h;
	x_width=w;

	center_x=init_carx;
	center_y=init_cary;

	win_w=window_w;
	win_h=window_h;
	win_name=window_name;//win_name赋成宏定义的window_name
    cvNamedWindow(win_name.c_str(),CV_WINDOW_AUTOSIZE);
    
	////生成的图像位深度为8，通道数为1
	image = cvCreateImage( cvSize(win_w,win_h), 8, 3 ); // 创建结果图像
	cvZero( image ); // 初始化图像为全黑
    update_image();
}

map_vis::~map_vis()
{
    cvReleaseImage(&image);
    cvDestroyWindow( win_name.c_str() );  // 释放窗口资源
}
