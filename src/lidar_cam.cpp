#include "lidar_cam.h"

using namespace cv;

void extract_edge(string file_path,PointCloud::Ptr c)
{
    PointCloud::Ptr cloud=PointCloud::Ptr(new PointCloud());
    if(pcl::io::loadPCDFile<PointT>(file_path,*cloud)==-1)
    {
	    cout<<"Error reading file "<<file_path<<endl;
	    return;
    }
       
    float pre_r=0;
    float pre_x=0;
    float pre_y=0;
    float pre_z=0;
    int count=0;
    for(int i=0;i<cloud->size();i++)
    {
	PointT pix=cloud->points[i];
	PointT p;
	
 	if(pre_r==pix.ring && sqrt((pix.x-pre_x)*(pix.x-pre_x)+(pix.y-pre_y)*(pix.y-pre_y)+(pix.z-pre_z)*(pix.z-pre_z))>5)
	{
	    p.z=pix.z;
	    p.x=pix.x;
	    p.y=pix.y;
	    
	    c->points.push_back(p);
	    count++;
	}
	
	pre_x=pix.x;
	pre_y=pix.y;
	pre_z=pix.z;
	pre_r=pix.ring;
	
    }
    c->height=1;
    c->width=count;
    c->is_dense=false;
}

void extract_pcd_edge(string dir,PointCloud::Ptr combined_cloud)
{
    vector<string> files=vector<string>();
    DIR *dp;
    struct dirent *dirp;
    
    if((dp  = opendir(dir.c_str())) == NULL) {
        cout << "Error"<<endl;
        return;
    }
    int count=0;
    while((dirp=readdir(dp))!=NULL)
    {
	if(string(dirp->d_name).find(".pcd")==string::npos)
	    continue;
	PointCloud::Ptr cloud=PointCloud::Ptr(new PointCloud());
    	extract_edge(dir+string(dirp->d_name),cloud);
	*combined_cloud+=*cloud;
	count+=1;
    }
    closedir(dp);
    
    combined_cloud->is_dense=false;
    //pcl::io::savePCDFile("/home/wayne/AirLab/LIDAR_CAM/pointcloud.pcd",*combined_cloud);
    //cout<<"Point Cloud Saved."<<endl;
    
    //return combined_cloud;
}

void get_cam_mtx(Mat K)
{
    ParameterReader pd;
    K.at<float>(0,0)=atof((pd.getData("camera.fx")).c_str());
    K.at<float>(1,1)=atof((pd.getData("camera.fy")).c_str());
    K.at<float>(0,2)=atof((pd.getData("camera.cx")).c_str());
    K.at<float>(1,2)=atof((pd.getData("camera.cy")).c_str());
    K.at<float>(2,2)=1;
}

void get_initial(Mat R,Mat T)
{
    ParameterReader pd;
    R.at<float>(0,0)=atof((pd.getData("R.00")).c_str());
    R.at<float>(0,1)=atof((pd.getData("R.01")).c_str());
    R.at<float>(0,2)=atof((pd.getData("R.02")).c_str());
    R.at<float>(1,0)=atof((pd.getData("R.10")).c_str());
    R.at<float>(1,1)=atof((pd.getData("R.11")).c_str());
    R.at<float>(1,2)=atof((pd.getData("R.12")).c_str());
    R.at<float>(2,0)=atof((pd.getData("R.20")).c_str());
    R.at<float>(2,1)=atof((pd.getData("R.21")).c_str());
    R.at<float>(2,2)=atof((pd.getData("R.22")).c_str());
    T.at<float>(0,0)=atof((pd.getData("T.00")).c_str());
    T.at<float>(1,0)=atof((pd.getData("T.10")).c_str());
    T.at<float>(2,0)=atof((pd.getData("T.20")).c_str());
}

static Mat CannyThreshold(Mat src,Mat src_gray,Mat dst,int, void*)
{
    Mat grad_x,grad_y,detected_edges;
    Mat edges=Mat::zeros(src.size(),CV_32FC3);
    ParameterReader pd;
    int lowThreshold = atoi((pd.getData("Canny_lowThreshold")).c_str());
    int maxThreshold = atoi((pd.getData("Canny_maxThreshold")).c_str());
    int kernel_size = atoi((pd.getData("Canny_kernel_size")).c_str());
    int grad_kernel = atoi((pd.getData("Gradient_kernel")).c_str());

    Canny( src_gray, detected_edges, lowThreshold, maxThreshold, kernel_size );
    Sobel(src_gray,grad_x,CV_32F,1,0,grad_kernel);
    Sobel(src_gray,grad_y,CV_32F,0,1,grad_kernel);
    
    for(int i=0;i<detected_edges.rows;i++)
    {
	for(int j=0;j<detected_edges.cols;j++)
	{
	    if(detected_edges.at<uint8_t>(i,j)!=0)
	    {
		Vec3f pix;
		pix.val[0]=grad_x.at<float>(i,j);
		pix.val[1]=grad_y.at<float>(i,j);
		pix.val[2]=1;
		edges.at<Vec3f>(i,j)=pix;
	    }
	    else
	    {
		Vec3f pix;
		pix.val[0]=0;
		pix.val[1]=0;
		pix.val[2]=0;
		edges.at<Vec3f>(i,j)=pix;
	    }
	}
    } 
    return edges;
}


/*edge stores as (grad_x,grad_y,0) for each edge pixel*/
Mat get_img_edge(string dir)
{    
    Mat src, src_gray,dst;
    src = imread( dir);
    if( src.empty() )
    {
    	std::cout << "Incorrect image directory!\n" << endl;
    	return Mat::zeros(Size(3008,4112),CV_32FC1);  	
    }
    dst.create( src.size(), src.type() );
    GaussianBlur(src,src,Size(3,3),0,0);
    cvtColor( src, src_gray, COLOR_BGR2GRAY );
    
    Mat edge=CannyThreshold(src,src_gray,dst,0, 0);
    //imwrite("/home/wayne/AirLab/LIDAR_CAM/src/edge.png",edge);
    return edge;
}

Vec4f find_closest(Vec2f pix,Mat img_edge)
{
    float r=pix[0];
    float c=pix[1];
    float max_dist=1000000;
    float max_r=0;
    float max_c=0;
    for(int i=int(r)-200;i<int(r)+200;i++)
    {
	for(int j=int(c)-200;j<int(c)+200;j++)
	{
	    float dist=sqrt((i-r)*(i-r)+(j-c)*(j-c));
	    if((i>0 && i<img_edge.rows && j>0 && j<img_edge.cols) && dist<max_dist && img_edge.at<Vec3f>(i,j)[2]==1)
	    {
		max_dist=dist;
		max_r=i;
		max_c=j;
	    }
	}
    }
    Vec4f closest_p;
    if(max_dist!=1000000)
    {
     	closest_p[0]=max_r;
    	closest_p[1]=max_c;
    	closest_p[2]=(img_edge.at<Vec3f>(max_r,max_c))[0];
    	closest_p[3]=(img_edge.at<Vec3f>(max_r,max_c))[1];    
    }
    else
    {
	closest_p[0]=-1;
        closest_p[1]=-1;
        closest_p[2]=0;
        closest_p[3]=0;
    }
    return closest_p;
}

void match_lidar_cam(FRAME frame,Mat R,Mat T,Mat K)
{
    Point_pair* pair_h;
    Point_pair pair;
    Point_pair* pair_cur;
    pair_cur=&pair;
    pair_h=&pair;
    int count=0;
    int row=3008;
    int col=4112;
    cout<<"PointCloud size: "<<frame.cloud_edge->size()<<endl;
    for(int i=0;i<frame.cloud_edge->size();i++)
    {
	PointT p=frame.cloud_edge->points[i];
    	Vec3f pix;
	pix[0]=p.x;
	pix[1]=p.y;
	pix[2]=p.z;
        Vec2f proj_p;
	Mat tmp=Mat::zeros(Size(1,3),CV_32FC1);
	tmp.at<float>(0,0)=p.x;
	tmp.at<float>(1,0)=p.y;
	tmp.at<float>(2,0)=p.z;

  	Mat proj_pix=K*(R*tmp+T);
	proj_pix /= proj_pix.at<float>(2,0);
	float r=proj_pix.at<float>(0,0);
	float c=proj_pix.at<float>(1,0);
  	if(r>row || r<0 || c>col || c<0) 
	    continue;
 	proj_p[0]=r;
	proj_p[1]=c;
	
	Vec4f closest_p=find_closest(proj_p,(frame.img_edge));
	//cout<<proj_p<<endl;
	//cout<<closest_p<<endl;
	//cout<<""<<endl;	
	pair_cur->lidar_p=pix;
	pair_cur->proj_p=proj_p;
	pair_cur->img_p=closest_p;
	Point_pair pair_next;
	pair_cur->next=&pair_next;
	pair_cur=pair_cur->next;
	
	count+=1;
    }
    cout<<"matched: "<<count<<endl;
    frame.point_pair=pair_h;
}

void update_RT(Mat R, Mat T)
{

}

string num_to_str(int num)
{
    stringstream ss;
    string i;
    ss<<num;
    i=ss.str();
    string result;
    if(num<10)
    {
	result="0"+i;
    }
    else
	result=i;
    return result;

}


void LIDAR_cam_calib()
{
    Mat R=Mat::zeros(Size(3,3),CV_32FC1);
    Mat K=Mat::zeros(Size(3,3),CV_32FC1);
    Mat T=Mat::zeros(Size(1,3),CV_32FC1);
   
    ParameterReader pd;
    get_initial(R,T);
    get_cam_mtx(K);
    string pc_dir_pre=pd.getData("PC_Dir").c_str();
    string img_dir_pre=pd.getData("IMG_Dir").c_str();
    int num_frame=atoi(pd.getData("Num_frame").c_str());
 
    int first=1; 
    FRAME f;
    FRAME* curr_f=&f;
    FRAME* frame_head;
    frame_head=&f;

    /*load all lidar and img, store in FRAME struct*/
    for(int i=1;i<=num_frame;i++)
    {
	string idx=num_to_str(i);
	string pc_dir=pc_dir_pre+"cloud_"+idx+"/";
	string img_dir=img_dir_pre+"image_"+idx+".png";
	curr_f->idx=idx;	
    	curr_f->cloud_edge=PointCloud::Ptr(new PointCloud());
    	extract_pcd_edge(pc_dir,curr_f->cloud_edge);
	curr_f->img_edge=get_img_edge(img_dir);
	if(i==num_frame)
    	    curr_f->next=NULL;
	else
	{
	    curr_f->next=(FRAME*)new FRAME;
	    curr_f=curr_f->next;
	}
    }
    cout<<"PointCloud Loaded."<<endl; 
    float err=0;
    float pre_err=-10000;
    int outer_count=0;
    int inner_count=0;
    int count_limit=1;
    int count=0;
    
    while(outer_count<count_limit)
    {
  
        FRAME* frame_p=frame_head;
	while(frame_p!=NULL)
 	{
	    cout<<frame_p->idx<<endl;
	    match_lidar_cam(*frame_p,R,T,K);//update point pair
	    frame_p=frame_p->next;	   
	}
	
	/*
        while(inner_count<count_limit)
	{
            err=0;
	    err+=1;//cal_cost_func(frame);
	    update_RT(R,T);
	}*/
	outer_count++;
    }
}





















