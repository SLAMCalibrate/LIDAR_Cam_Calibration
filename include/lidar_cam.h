#pragma once

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <velodyne_pointcloud/point_types.h>

typedef velodyne_pointcloud::PointXYZIR PointT;
typedef pcl::PointCloud<PointT> PointCloud;

struct Point_pair
{
    cv::Vec3f lidar_p;//(x,y,z)
    cv::Vec2f proj_p;//(r,c)
    cv::Vec4f img_p;//(r,c,grad_x,grad_y)
    Point_pair* next;
};//store as image

struct FRAME
{
    string idx;
    PointCloud::Ptr cloud_edge;
    Point_pair* point_pair;
    cv::Mat img_edge;
    FRAME* next;
};

void extract_edge(string file_path,PointCloud::Ptr c);
void extract_pcd_edge(string dir,PointCloud::Ptr combined_cloud);
void get_initial(cv::Mat R,cv::Mat T);
cv::Mat get_img_edge(string dir);
void match_lidar_cam(FRAME frame,cv::Mat R,cv::Mat T);
void update_RT(cv::Mat R, cv::Mat T);
cv::Vec4f find_closest(cv::Vec2f pix,cv::Mat img_edge);
void get_cam_mtx(cv::Mat K);
void LIDAR_cam_calib();

class ParameterReader
{
public:
    ParameterReader(string filename="/home/wayne/AirLab/LIDAR_CAM/src/parameters.txt")
    {

        ifstream fin(filename.c_str());
        if(!fin)
        {
            cerr<<"parameter file does not exist."<<endl;
            return;
        }
        while(!fin.eof())
        {
            string str;
            getline( fin, str );
            if (str[0] == '#')
            {
                continue;
            }

            int pos = str.find("=");
            if (pos == -1)
                continue;
            string key = str.substr( 0, pos );
            string value = str.substr( pos+1, str.length() );
            data[key] = value;

            if ( !fin.good() )
                break;
        }
    }
    string getData( string key )
    {
        map<string, string>::iterator iter = data.find(key);
        if (iter == data.end())
        {
            cerr<<"Parameter name "<<key<<" not found!"<<endl;
            return string("NOT_FOUND");
        }
        return iter->second;
    }
public:
    map<string, string> data;
};
