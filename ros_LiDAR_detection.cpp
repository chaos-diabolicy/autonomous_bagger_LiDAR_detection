#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>
#include <ctime>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <string>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/range_image/range_image.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/surface/mls.h>

#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/passthrough.h>
#include <ctime>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <algorithm>

using namespace std;

//定义全局变量global_z,global_y，用于接收主控发来的坐标数据
float global_z = 0;
float global_y = 0;

//定义全局变量x_average_1,初始化值为0，用于接收本程序计算得到的数据
float* x_average_1 = new float(0);

//创建高度信息发布器
ros::Publisher x_average_pub;

//创建点云信息发布器
ros::Publisher cloud_pub;

//创建错误信息发布器
ros::Publisher alert_msg_pub;

//订阅雷达点云回调函数
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& );

//查找匹配点云
vector<int>* selectMatchPoints(float& , float& , pcl::PointCloud<pcl::PointXYZ>::Ptr& );

//计算平均高度
float* computeAverageX(pcl::PointCloud<pcl::PointXYZ>::Ptr& , vector<int>* );

int main(int argc, char** argv)
{
    //初始化ros节点
    ros::init(argc, argv, "ros_visualization");

    //创建ros节点句柄
    ros::NodeHandle nh;

    //创建雷达点云订阅器，消息类型为sensor_msgs::PointCloud2，话题名为规定的/rslidar_points,队列长度为1
    ros::Subscriber cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/rslidar_points", 1, cloudCallback);

    //高度信息发布器,消息类型为std_msgs::Float32，话题名为x_average，队列长度为10
    x_average_pub = nh.advertise<std_msgs::Float32>("x_average", 10);

    //点云信息发布器,消息类型为sensor_msgs::PointCloud2，话题名为output_cloud，队列长度为10
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("output_cloud", 10);

    //错误信息发布器，消息类型为std_msgs::Int16，话题名为alert_msg，队列长度为1
    alert_msg_pub = nh.advertise<std_msgs::Int16>("alert_msg", 1);

    //loop
    ros::Rate loop_rate(20);

    //spinOnce命令，使订阅器开始订阅消息
    int count = 0;
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();

        //每进入一次回调函数count就加一，然后重新置零
        //如果不能进入cloudCallback回调函数，该循环一直进行，count将会变得不正常的大，以此判断是否进入了回调函数
        ++count;

        //判断count的大小,发现异常结束本次循环，发送警告消息
        if (count >= 200 && 0.0 == *x_average_1)
        {
            std_msgs::Int16 alert_msg;
            alert_msg.data = -101;
            alert_msg_pub.publish(alert_msg);
            ROS_INFO("The program didn't receive the lidar data,please check the lidar!\nAlert_msg = %d", alert_msg.data);
            continue;
        }
    }

    return 0;
}

//订阅雷达点云回调函数
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    //接收订阅器订阅的雷达点云,转换为pcl点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr primary_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*cloud_msg, *primary_cloud_ptr);

    //利用直通滤波器滤波，初步去除点云噪点
    double border = 15;

    //x方向滤波,-15m~15m
    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setInputCloud(primary_cloud_ptr);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(-border, border);
    pass_x.filter(*primary_cloud_ptr);

    //y方向滤波,-15m~15m
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setInputCloud(primary_cloud_ptr);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(-border, border);
    pass_y.filter(*primary_cloud_ptr);

    //z方向滤波,0~15m
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(primary_cloud_ptr);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(0, border);
    pass_z.filter(*primary_cloud_ptr);

    //离群点滤波，进一步去除离群点，排除干扰
    //创建离群点滤波实例化对象sor
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;

    //创建点云指针cloud_filtered_1储存过滤后的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_1(new pcl::PointCloud<pcl::PointXYZ>);

    //设置sor过滤器的参数
    sor.setInputCloud(primary_cloud_ptr);   //设置输入点云
    sor.setMeanK (20);                      //设置在进行统计时考虑查询点临近点数
    sor.setStddevMulThresh (1.0);           //设置判断是否为离群点的阈值
    sor.filter (*cloud_filtered_1);         //将过滤后的点云信息储存在cloud_filtered_1中

    //进一步对点云进行上采样处理
    //创建上采样滤波对象
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls_filter;

    //创建第二次滤波后的点云储存对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_2(new pcl::PointCloud<pcl::PointXYZ>);

    mls_filter.setInputCloud(cloud_filtered_1);
    //建立搜索对象
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
    mls_filter.setSearchMethod(tree);
    //设置搜索邻域的半径为3cm
    mls_filter.setSearchRadius(0.3);
    // Upsampling 采样的方法有 DISTINCT_CLOUD, RANDOM_UNIFORM_DENSITY
    mls_filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);
    // 采样的半径是
    mls_filter.setUpsamplingRadius(0.3);
    // 采样步数的大小
    mls_filter.setUpsamplingStepSize(0.2);
    //进行上采样，并将结果储存在cloud_filtered_2中
    mls_filter.process(*cloud_filtered_2);

    //执行点云查找，找到指定点的坐标
    float search_z_1 = 3.9;//根据结果来看，z坐标设置20不太行，点云太过稀疏；z值为12.5时，只有正值可以正常显示，初步估计是因为挖掘机臂挡住了那一部分点云
    float search_y_1 = 0;//在当前雷达坐标系下，z轴为前进方向，y轴为左右偏移，x轴为高度方向

    //主控发来的数据
    //请输入接收主控发来的数据的代码，请用global_z(前进方向)和global_y(左右方向)接收需要查找的坐标


    //接收代码编写完毕



    //将全局变量接收到的z，y坐标分别赋给search_z_1与search_y_1作为查找坐标
    //请输入赋值的代码


    //赋值代码编写完毕


    //如果全局变量没有获得数据，直接返回数据-100
    // if(global_z == 0 && global_y == 0)
    // {
    //     ROS_INFO("The program didn't find the coordinate infomation,please input the INFO");

    //     //程序没有接收到匹配点云必须的坐标信息,向主控返回-100
    //     std_msgs::Int16 msg;
    //     msg.data = -100;
    //     alert_msg_pub.publish(msg);
    //     ROS_INFO("返回消息msg = %d", msg.data);
    //     return;
    // }

    float search_z_2 = 5.3;
    float search_y_2 = 0;

    float search_z_3 = 6.7;
    float search_y_3 = 0;

    //创建vector容器，储存满足搜索条件的点的下标
    vector<int>* v1 = selectMatchPoints(search_z_1, search_y_1, cloud_filtered_2);

    //判断接收匹配点云下标的容器的情况，如果只有-102，则直接发送-102,结束本次回调
    if (v1->at(0) == -102)
    {
        std_msgs::Int16 msg;
        msg.data = v1->at(0);
        alert_msg_pub.publish(msg);
        ROS_INFO("Alert msg = %d", msg.data);
        return;
    }

    vector<int>* v2 = selectMatchPoints(search_z_2, search_y_2, cloud_filtered_2);
    vector<int>* v3 = selectMatchPoints(search_z_3, search_y_3, cloud_filtered_2);

    //计算平均高度
    // float* x_average_1 = new float(0);
    x_average_1 = computeAverageX(cloud_filtered_2, v1);
    cout << "平均高度x_average_1 = " << (*x_average_1 + 3.2) << endl;

    // float* x_average_2 = new float(0);
    // x_average_2 = computeAverageX(cloud_filtered_2, v2);
    // cout << "平均高度x_average_2 = " << (*x_average_2 + 3.2) << endl;

    // float* x_average_3 = new float(0);
    // x_average_3 = computeAverageX(cloud_filtered_2, v3);
    // cout << "平均高度x_average_3 = " << (*x_average_3 + 3.2) << endl;

    //生成包裹匹配点云的长方体框状点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cuboid_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    
    sensor_msgs::PointCloud2 cuboid_cloud;
    sensor_msgs::PointCloud2 output_cloud;

    pcl::toROSMsg(*primary_cloud_ptr, output_cloud);

    //发布点云数据
    cloud_pub.publish (output_cloud);

    //发布高度信息
    //初始化消息内容
    std_msgs::Float32 msg;
    msg.data = *x_average_1 + 3.2;

    //发布消息，并显示消息
    x_average_pub.publish(msg);
    ROS_INFO("msg = %f", msg.data);
}

//计算平均高度
float* computeAverageX(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_filtered_2, vector<int>* v)
{
    float* x_average = new float;
    float sum = 0;
    //所有y坐标加和
    for(vector<int>::const_iterator it = v->begin(); it != v->end(); ++it)
    {
        sum += cloud_filtered_2->points[*it].x;
    }

    cout << sum << endl;
    //求取平均值
    *x_average = (sum / v->size());

    return x_average;
}

//遍历整个点云，查找匹配点云
vector<int>* selectMatchPoints(float& search_z, float& search_y, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_filtered_2)
{
    //创建储存符合条件的点云下标的容器
    vector<int>* v = new vector<int>;

    //声明搜索半径
    float search_radius;

    //不同的z值导致不同的检索范围，现做如下区分
    if(search_z < 0)
    {
        cout << "Warning:cannot select the point with position z = " << search_z <<endl;
    }
    else if(search_z > 0 && search_z < 6)
    {
        search_radius = 0.1;
    }
    else
    {
        search_radius = 0.35;
    }

    int j = 0;
    //遍历整个点云数据
    for(int i = 0; i < cloud_filtered_2->size(); ++i)
    {
        //感觉上，如果使点云数据的坐标恰好等于需要查找的坐标，太过严苛，需要在查找点的已知坐标基础上划定一个范围
        bool z_match = (cloud_filtered_2->points[i].z < search_z + search_radius) && (cloud_filtered_2->points[i].z > search_z - search_radius);
        bool y_match = (cloud_filtered_2->points[i].y < search_y + search_radius) && (cloud_filtered_2->points[i].y > search_y - search_radius);

        //判断点是否在范围之中
        if(z_match && y_match)
        {
            cout << cloud_filtered_2->points[i].x << "   ";
            
            //储存符合条件的点云坐标进预先创建好的容器中
            v->push_back(i);
        }
    }

    //如果并未查找到匹配点，在容器v中加入-102这个值，由于点云下标是非负的，故-102的出现就代表了未查找到匹配点云
    if(v->size() == 0)
    {
        v->push_back(-102);
    }
    return v;
}
