#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <math.h>
#include "convert.hpp"
#include "gnss_stat.hpp"


double f = 1/298.257223563;
double a = 6378.137e3;   //地球半长轴
double b = a*(1-f);   //地球半短轴
double e2 = 1-pow(b,2)/pow(a,2);
struct my_pose
{
    double latitude;
    double longitude;
    double altitude;
};

double rad(double d)
{
    return d * M_PI / 180.0;
}

bool init;
nav_msgs::Path path_,path_2;
 
my_pose init_pose;
ros::Publisher pub,pub2;

void gpscallback(const sensor_msgs::NavSatFixConstPtr& gps_msg_ptr)
//引用传递，这时存放的是由主调函数放进来的实参变量的地址
{
    double gps_msg_lat = gps_msg_ptr->latitude;
    double gps_msg_long = gps_msg_ptr->longitude;
    double gps_msg_alt = gps_msg_ptr->altitude;
    if(!init)
    {
        init_pose.latitude = gps_msg_ptr->latitude;
        init_pose.longitude = gps_msg_ptr->longitude;
        init_pose.altitude = gps_msg_ptr->altitude;
        init = true;
    }
    else
    {
        //转换为弧度
        double radLat_init, radLat, radLong_init, radLong;
        radLat_init = rad(init_pose.latitude);
        radLong_init = rad(init_pose.longitude);
        radLat = rad(gps_msg_lat);   //接收的纬度
        radLong = rad(gps_msg_long);  //接收的经度
        
        double slat, clat, slong, clong;
        double x1, y1, z1, del_x, del_y, del_z;
        slat=sin(radLat); clat=cos(radLat); slong=sin(radLong); clong=cos(radLong);
        double N=a/sqrt(1-e2*pow(slat,2)); 
    // 84toECEF
        x1 = (N+gps_msg_alt) * clat * clong;
        y1 = (N+gps_msg_alt) * clat * slong;
        z1 = (N * (1 - e2) + gps_msg_alt) * slat;

    //参考点
        double xref, yref, zref;
        xref = (N + init_pose.altitude) * cos(radLat_init) * cos(radLong_init);
        yref = (N + init_pose.altitude) * cos(radLat_init) * sin(radLong_init);
        zref = (N * (1-e2) + init_pose.altitude) * sin(radLat_init);

    //ECEFtoENU
        del_x = x1 - xref;
        del_y = y1 - yref;
        del_z = z1 - zref;
        double del_E, del_N, del_U;
        // double E = 0; double N_ = 0; double U=0;
        double M[3*3];
        M[0] = -sin(radLong_init);                 M[3] = cos(radLong_init);                   M[6] = 0;
        M[1] = -cos(radLong_init) * sin(radLat_init); M[4] = -sin(radLong_init) * sin(radLat_init);  M[7] = cos(radLat_init);
        M[2] = cos(radLong_init) * sin(radLat_init);  M[5] = sin(radLong_init) * cos(radLat_init);   M[8] = sin(radLat_init);

        del_E = M[0] * del_x + M[3] * del_y + M[6] * del_z;
        del_N = M[1] * del_x + M[4] * del_y + M[7] * del_z;
        del_U = M[2] * del_x + M[5] * del_y + M[8] * del_z;
 
        //发布轨迹
        path_.header.frame_id = "path";
        path_.header.stamp = ros::Time::now();

        geometry_msgs::PoseStamped pose; //包含header(顺序id时间戳帧id)和位置以及四元数
        pose.header = path_.header;

        pose.pose.position.x = del_E;
        pose.pose.position.y = del_N;
        pose.pose.position.z = del_U;
        path_.poses.push_back(pose);     //push_bach函数用于在vector后添加新元素
        ROS_INFO("(x:%0.6f, y:%0.6f, z:%0.6f)", del_E, del_N, del_U);
        pub.publish(path_);
    }
}

sensor_msgs::NavSatFix nav_sat_fix_origin_;
GNSSStat convert( const sensor_msgs::NavSatFix  & nav_sat_fix_msg, 
                        CoordinateSystem coordinate_system)
{
    GNSSStat gnss_stat;
    
    // std::cerr << "[5]convert" << std::endl;
    // printf("nav_sat_fix_msg.latitude: %f\n", nav_sat_fix_msg.latitude);
    // printf("nav_sat_fix_msg.longitude: %f\n", nav_sat_fix_msg.longitude);
    // printf("nav_sat_fix_msg.altitudes: %f\n", nav_sat_fix_msg.altitude);
    // printf("nav_sat_fix_origin_.latitude: %f\n", nav_sat_fix_origin_.latitude);
    // printf("nav_sat_fix_origin_.longitude: %f\n", nav_sat_fix_origin_.longitude);
    // printf("nav_sat_fix_origin_.altitudes: %f\n", nav_sat_fix_origin_.altitude);
    if (coordinate_system == CoordinateSystem::UTM) {
      gnss_stat = NavSatFix2UTM(nav_sat_fix_msg);
    } else if (coordinate_system == CoordinateSystem::LOCAL_CARTESIAN_UTM) {
      gnss_stat =
        NavSatFix2LocalCartesianUTM(nav_sat_fix_msg, nav_sat_fix_origin_);
    } else if (coordinate_system == CoordinateSystem::MGRS) {
      gnss_stat = NavSatFix2MGRS(nav_sat_fix_msg, MGRSPrecision::_100MICRO_METER);
    } 
    else if (coordinate_system == CoordinateSystem::LOCAL_CARTESIAN_WGS84) {
      gnss_stat = NavSatFix2LocalCartesianWGS84(nav_sat_fix_msg, nav_sat_fix_origin_);
    } else {
      std::cout << "Unknown Coordinate System" << std::endl;
    }
    return gnss_stat;
}

void gpscallback_2(const sensor_msgs::NavSatFixConstPtr& gps_msg_ptr)
//引用传递，这时存放的是由主调函数放进来的实参变量的地址
{
    double xc = gps_msg_ptr->latitude;
    double yc = gps_msg_ptr->longitude;
    double zc = gps_msg_ptr->altitude;
    if(!init)
    {
        nav_sat_fix_origin_.latitude = gps_msg_ptr->latitude;
        nav_sat_fix_origin_.longitude = gps_msg_ptr->longitude;
        nav_sat_fix_origin_.altitude = gps_msg_ptr->altitude;
        init = true;
        
    }
    else
    {
        sensor_msgs::NavSatFix  nav_sat_fix_msg;
        nav_sat_fix_msg.latitude = gps_msg_ptr->latitude;
        nav_sat_fix_msg.longitude = gps_msg_ptr->longitude;
        nav_sat_fix_msg.altitude = gps_msg_ptr->altitude;

        const auto gnss_stat = convert(nav_sat_fix_msg, CoordinateSystem::LOCAL_CARTESIAN_UTM);

        std::cerr << "<-------------->"<< std::endl;
        std::cout << "latitude: " << nav_sat_fix_msg.latitude  
                  << " longitude: " << nav_sat_fix_msg.longitude 
                   << " altitude: " << nav_sat_fix_msg.altitude << std::endl;
        std::cerr << " [x]:" <<gnss_stat.x 
                  << " [y]:" <<gnss_stat.y
                   << " [z]:" <<gnss_stat.z<< std::endl;

            
        //发布轨迹
        path_2.header.frame_id = "gps";
        path_2.header.stamp = ros::Time::now();

        geometry_msgs::PoseStamped pose;
        pose.header = path_2.header;

        pose.pose.position.x = gnss_stat.x;
        pose.pose.position.y = gnss_stat.y;
        pose.pose.position.z = gnss_stat.z;
        path_2.poses.push_back(pose);     //push_bach函数用于在vector后添加新元素
        ROS_INFO("(x:%0.6f, y:%0.6f, z:%0.6f)", gnss_stat.x, gnss_stat.y, gnss_stat.z);
        pub2.publish(path_2);
    }
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "gps_bag_node");
    init = false;
    ros::NodeHandle n;
    ros::Subscriber pose_sub=n.subscribe("/fix", 10, gpscallback_2);
    pub = n.advertise<nav_msgs::Path>("gps_path", 10);
    pub2 = n.advertise<nav_msgs::Path>("gps_path_lib", 10);
    ros::spin();
    return 0;
}
