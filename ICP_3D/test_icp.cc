#include <iostream>
#include <string>
#include <Eigen/Core>
#include <sophus/so3.hpp>
#include <sophus/se3.hpp>
#include "sophus/so3.hpp"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/impl/pcl_base.hpp>

#include "icp_svd.h"
#include "sys_utils.h"

using namespace std;

// #include "ch7/icp_3d.h"
// #include "ch7/ndt_3d.h"
// #include "common/point_cloud_utils.h"
// #include "common/sys_utils.h"





  void icp_svd_align(const PointCloudType::Ptr target_cloud,
                     const PointCloudType::Ptr source_cloud,
                     Sophus::SE3d& initial_pose,
                    double min_error,
                    double max_iters,
                    double epsilon);
//高斯牛顿法对齐点云  
void icp_GSN_align(const PointCloudType::Ptr target_cloud,
                     const PointCloudType::Ptr source_cloud,
                     Sophus::SE3d& initial_pose,
                    double min_error,
                    double max_iters,
                    double epsilon);
void pointcloud_visualize(const PointCloudType::Ptr target_cloud,
                     const PointCloudType::Ptr source_cloud,
                     Eigen::Matrix4d T);

int main(int argc, char** argv) {

    string source_path  = "data/ch7/EPFL/kneeling_lady_source.pcd";
    string target_path  = "data/ch7/EPFL/kneeling_lady_target.pcd";
    string ground_truth_file = "data/ch7/EPFL/kneeling_lady_pose.txt";
    
    // EPFL 雕像数据集：./ch7/EPFL/aquarius_{sourcd.pcd, target.pcd}，真值在对应目录的_pose.txt中
    // EPFL 模型比较精细，配准时应该采用较小的栅格

    std::ifstream fin(ground_truth_file);
    Sophus::SE3d gt_pose;
    if (fin) {
        double tx, ty, tz, qw, qx, qy, qz;
        fin >> tx >> ty >> tz >> qw >> qx >> qy >> qz;
        fin.close();
        gt_pose = Sophus::SE3d(Eigen::Quaterniond(qw, qx, qy, qz), Eigen::Vector3d(tx, ty, tz));
    }

    // 加载 点云文件
    PointCloudType::Ptr source(new PointCloudType);
    PointCloudType::Ptr target(new PointCloudType);
    pcl::io::loadPCDFile(source_path, *source);
    pcl::io::loadPCDFile(target_path, *target);

    // 体素滤波  降采样点云
    pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
    double leaf_size = 0.01;
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);

    voxel_grid.setInputCloud(source);
    voxel_grid.filter(*source);

    voxel_grid.setInputCloud(target);
    voxel_grid.filter(*target);


 
    	//运行算法
	// ICP_SVD icp(0.01,50,0.0);
	// icp.setSourceCloud(source);
	// icp.setTargetCloud(target);
	// icp.pointcloudFilter();
	// icp.registration();
	// icp.saveICPCloud("icp_cloud.pcd");
	// icp.getTransformationMatrix();
	// icp.getScore();
	// icp.visualize();

	// system("pause");

    // {
    // Sophus::SE3d init_pose = Sophus::SE3d(Eigen::Quaterniond(1, 0, 0, 0), Eigen::Vector3d(0, 0, 0));
    
    // icp_svd_align(target,source,init_pose,1e-5,35,0); 
    // }

    {
    Sophus::SE3d init_pose = Sophus::SE3d(Eigen::Quaterniond(1, 0, 0, 0), Eigen::Vector3d(0, 0, 0));
    icp_GSN_align(target,source,init_pose,1e-2,35,1e-2); 
    cout<<"flage1: "<<endl;
    Eigen::Matrix3d R = init_pose.so3().matrix();//.Eigen::Matrix3d::Identity();
    Eigen::Vector3d t = init_pose.translation();
    Eigen::Matrix4d T_final ;
    T_final << R, t, 0, 0, 0, 1;

    std::cout<<"T_final: "<<T_final<<std::endl;

    pointcloud_visualize(target,source,T_final);
    }
    
    // std::cout<<std::endl<<std::endl << "PCL ICP point to point" <<std::endl;
    // /// PCL 库中的 ICP  算法
    // {
    //     pcl::IterativeClosestPoint<PointType, PointType> icp_pcl;
    //     icp_pcl.setInputSource(source);
    //     icp_pcl.setInputTarget(target);
    //     PointCloudType::Ptr output_pcl(new PointCloudType);//创建输出点云

    //     icp_pcl.align(*output_pcl);

    //     Sophus::SE3f T = Sophus::SE3f(icp_pcl.getFinalTransformation());//读取ICP计算值
    //     std::cout << "pose from icp pcl: " << T.so3().unit_quaternion().coeffs().transpose() << ", "
    //                 << T.translation().transpose()<<std::endl<<std::endl;

    //     output_pcl->height = 1;
    //     output_pcl->width = output_pcl->size();
    //     pcl::io::savePCDFileASCII("pcl_icp_trans.pcd", *output_pcl);
    
    //     // 计算估计位姿与真实值之间的误差
    //     double pose_error = (gt_pose.inverse() * T.cast<double>()).log().norm();
    //     std::cout << "ICP PCL pose error: " << pose_error<<std::endl;
    // }
    std::cout << "gt_pose  pose  : " << gt_pose.translation().transpose()<<std::endl;    

    // /// PCL NDT 作为备选
    // {
    //     pcl::NormalDistributionsTransform<PointType, PointType> ndt_pcl;
    //     ndt_pcl.setInputSource(source);
    //     ndt_pcl.setInputTarget(target);
    //     ndt_pcl.setResolution(0.2);
    //     PointCloudType::Ptr output_pcl(new PointCloudType);
    //     ndt_pcl.align(*output_pcl);
    //     Sophus::SE3f T = Sophus::SE3f(ndt_pcl.getFinalTransformation());
    //     std::cout << "pose from ndt pcl: " << T.so3().unit_quaternion().coeffs().transpose() << ", "
    //                 << T.translation().transpose() << ', trans: ' << ndt_pcl.getTransformationProbability()<<std::endl;
        
    //     output_pcl->height = 1;
    //     output_pcl->width = output_pcl->size();
    //     pcl::io::savePCDFileASCII("pcl_icp_trans.pcd", *output_pcl);
        
    //     std::cout << "score: " << ndt_pcl.getTransformationProbability()<<std::endl;
    //     //计算GT pose差异
    //     double pose_error = (gt_pose.inverse() * T.cast<double>()).log().norm();
    //     std::cout << "NDT PCL pose error: " << pose_error<<std::endl;

    // }

    return 0;
}

 

void icp_svd_align(const PointCloudType::Ptr target_cloud,
                     const PointCloudType::Ptr source_cloud,
                     Sophus::SE3d& initial_pose,
                    //const std::vector<Eigen::Vector3d>& source,
                    double min_error,
                    double max_iters,
                    double epsilon)
{
    std::cerr<<"------------------\n"<<std::endl;
    // 0 滤波 
    // 1、找到对应点对
    // 2、求解R,t
    // 3、判断是否收敛
  
    clock_t start = clock();


    //给初始值
    Eigen::Matrix3d R = initial_pose.so3().matrix();//.Eigen::Matrix3d::Identity();
    Eigen::Vector3d t = initial_pose.translation();
    Eigen::Matrix4d T_final ;
    T_final << R, t, 0, 0, 0, 1;

    const int min_pointsize = source_cloud->size();          // 最少点数量
    const double min_err2 = min_error*min_error;
    const double factor = 9.0;
    const int n_selected_points = (int)(min_pointsize*0.9);
    const int step = min_pointsize / n_selected_points;

    pcl::KdTreeFLANN<PointType>::Ptr kd_tree(new pcl::KdTreeFLANN<PointType>());
    kd_tree->setInputCloud(target_cloud);


    // 创建两个vector来存放匹配点对
    std::vector<Eigen::Vector3d> source_matched;
    std::vector<Eigen::Vector3d> target_matched;
    source_matched.reserve(n_selected_points);
    target_matched.reserve(n_selected_points);
    // 设置每一次迭代后的err
    double sqr_distance_th = std::numeric_limits<double>::max();                // 筛选匹配点的阈值
    double cur_error = INT_MAX;
    double last_error = INT_MAX;      
    int iters = 0;
    while (cur_error > min_error && iters < max_iters)
    {
        
        int count = 0;      // 匹配点对数
        source_matched.clear();// 清空匹配点对
        target_matched.clear();
        double err = 0;// 统计这次的点距离平方和

        for(size_t i_source = 0;i_source<source_cloud->size();i_source+=step)
        {
            // 将src点云变换到target坐标系下
            PointType p_src = source_cloud->points[i_source];
            Eigen::Vector4d p_tem = T_final*Eigen::Vector4d(p_src.x,p_src.y,p_src.z,1);          // 变换后的点
            PointType p_srcT;
            p_srcT.x = p_tem(0);
            p_srcT.y = p_tem(1);
            p_srcT.z = p_tem(2);
            // kd-tree查找
            std::vector<int>index(1);
            std::vector<float>distance(1);
            if(!kd_tree->nearestKSearch(p_srcT,1,index,distance))  
            {
                std::cerr << "ERROR: no points found.\n";
                return;
            }

            if((!index.empty() )&& (!distance.empty() ))
            {
                source_matched.push_back(Eigen::Vector3d(p_src.x,p_src.y,p_src.z));
                
                PointType p_tar = target_cloud->points[index[0]];
                target_matched.push_back(Eigen::Vector3d(p_tar.x,p_tar.y,p_tar.z));
                
                err+=sqrt(distance[0]);
                count++;
            }
        }   // for source.size()
        
        // 获取当前的误差以及误差变化量
        last_error = cur_error;// 更新上一次的误差
        cur_error = err/double(count);
        double error_change = last_error - cur_error;
        
        std::cout<<"iter "<<iters<<", match count: "<<count<<", cur_error: "
                <<cur_error<<", error_change: "<<error_change<<std::endl;
        // 若当前误差小，或者误差变化小，退出迭代
        if( error_change<epsilon)  
        {
            std::cerr<<"["<<iters<<"] coverage"<<std::endl;
            std::cerr<<"final cost: "<<cur_error<<", cost change: "<<error_change<<std::endl;
            break;
        }

        // step 2 计算R，t
        //      2.1 计算中心
        Eigen::Vector3d center_src(0.0,0.0,0.0);
        Eigen::Vector3d center_tar(0.0,0.0,0.0);
        for(size_t i = 0;i<source_matched.size();i++)        // 遍历匹配src中的每一个点
        {
            center_src+=source_matched.at(i);
            center_tar+=target_matched.at(i);
        }
        center_src/=double(source_matched.size());
        center_tar/=double(target_matched.size());

        //      2.2 求W
        Eigen::Matrix3d W(Eigen::Matrix3d::Identity());  // 记得矩阵W变量给初值，否则会报错！！！
        for(size_t i = 0;i<source_matched.size();i++)
        {
            W+=(source_matched.at(i)-center_src)*( (target_matched.at(i)-center_tar).transpose() );// 3*3的矩阵
        }

        //     2.3 求R,t
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(W,Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d U = svd.matrixU();
        Eigen::Matrix3d V = svd.matrixV();

        if (U.determinant()*V.determinant() < 0)
        {
            for (int x = 0; x < 3; ++x)
                V(x, 2) *= -1;
        }

        Eigen::Matrix3d R_new = V* U.transpose();
        Eigen::Vector3d t_new = center_tar - R_new*center_src;
        Eigen::Matrix4d T_new;
        T_new << R_new, t_new, 0, 0, 0, 1;
        T_final = T_new*T_final; //更新变换矩阵  

        iters++;  // 迭代        
    }       
    clock_t end = clock();
    double time = double(end-start)/CLOCKS_PER_SEC;
    std::cerr<<"spend time: "<<time*1000<<"ms"<<std::endl;

    std::cerr<<"T_final: "<<T_final<<std::endl;

    pointcloud_visualize(target_cloud,source_cloud,T_final);

}   
//高斯牛顿法对齐点云  
void icp_GSN_align(const PointCloudType::Ptr target_cloud,
                     const PointCloudType::Ptr source_cloud,
                     Sophus::SE3d& initial_pose,
                    double min_error,
                    double max_iters,
                    double epsilon)
{
    std::cerr<<"-----icp_GSN_align--------\n"<<std::endl;
    // 0 滤波 
    // 1、找到对应点对
    // 2、求解R,t
    // 3、判断是否收敛
    clock_t start = clock();


    //给初始值
    // Eigen::Matrix3d R = initial_pose.so3().matrix();//.Eigen::Matrix3d::Identity();
    // Eigen::Vector3d t = initial_pose.translation();
    // Eigen::Matrix4d T_final ;
    // T_final << R, t, 0, 0, 0, 1;
    Sophus::SE3d pose = initial_pose;


    const int min_pointsize = source_cloud->size();          // 最少点数量
    const double min_err2 = min_error*min_error;
    const double factor = 9.0;
    const int n_selected_points = (int)(min_pointsize*1);
    const int step = min_pointsize / n_selected_points;

    pcl::KdTreeFLANN<PointType>::Ptr kd_tree(new pcl::KdTreeFLANN<PointType>());
    kd_tree->setInputCloud(target_cloud);
    cout<<"flageA-2: "<<endl;
    std::vector<bool> effect_pts(n_selected_points, false);
    std::vector<Eigen::Matrix<double, 3, 6>> jacobians(n_selected_points);
    std::vector<Eigen::Vector3d> errors(n_selected_points);
    
    // 设置每一次迭代后的err
    double sqr_distance_th = std::numeric_limits<double>::max();                // 筛选匹配点的阈值
    double cur_error = INT_MAX;
    double last_error = INT_MAX;      
    int iters = 0;
    while (cur_error > min_error && iters < max_iters)
    {
        
        int count = 0;      // 匹配点对数
        jacobians.clear();
        errors.clear();
        effect_pts.clear();
 
        double err = 0;// 统计这次的点距离平方和

        for(size_t i_source = 0;i_source<source_cloud->size();i_source+=step)
        {
            // 将src点云变换到target坐标系下
            PointType p_src = source_cloud->points[i_source];
            Eigen::Vector3d q = Eigen::Vector3d(p_src.x,p_src.y,p_src.z); 
            Eigen::Vector3d qs = pose*q;         // 变换后的点
            PointType p_srcT;
            p_srcT.x = qs(0);
            p_srcT.y = qs(1);
            p_srcT.z = qs(2);
            // kd-tree查找
            std::vector<int>index(1);
            std::vector<float>distance(1);
            if(!kd_tree->nearestKSearch(p_srcT,1,index,distance))  
            {
                std::cerr << "ERROR: no points found.\n";
                return;
            }

            if((!index.empty() )&& (!distance.empty() ))
            { 
                PointType p_tar = target_cloud->points[index[0]];
                Eigen::Vector3d p(p_tar.x,p_tar.y,p_tar.z);
              
                Eigen::Vector3d e = p - qs;
                if(e.squaredNorm() > 1.0)
                {
                    effect_pts[count] = false;
                    //continue;
                }
                else
                    effect_pts[count] = true;

                Eigen::Matrix<double, 3, 6> J;
                J.block<3, 3>(0, 0) = pose.so3().matrix() * Sophus::SO3d::hat(q);
                J.block<3, 3>(0, 3) = -Eigen::Matrix<double, 3, 3>::Identity();

                jacobians[count] = J;
                errors[count] = e;

                err+=sqrt(distance[0]);
                count++;
            }
        }   // for source.size()
         
        Eigen::Matrix<double, 6, 6> H = Eigen::MatrixXd::Zero(6,6);
        Eigen::Matrix<double, 6, 1> Jerr = Eigen::MatrixXd::Zero(6,1);
        for(size_t idx = 0;idx<count;idx++)
        {    
            if(effect_pts[idx])
            {
                H = H + jacobians[idx].transpose() * jacobians[idx];
                Jerr= Jerr- jacobians[idx].transpose() * errors[idx];   
            }
        }       
        //std::cout<<"count "<<count<< std::endl; 
        //cout<<"H: "<<endl<<H<<endl;
        //cout<<"Jerr: "<<endl<<Jerr<<endl;
        Eigen::Matrix<double, 6, 1> dx = H.inverse() * Jerr;
        pose.so3() = pose.so3() * Sophus::SO3d::exp(dx.head<3>());
        pose.translation() += dx.tail<3>();



        // 获取当前的误差以及误差变化量
        last_error = cur_error;// 更新上一次的误差
        cur_error = err/double(count);
        double error_change = last_error - cur_error;

        std::cout<<"iter "<<iters<<", match count: "<<count<<", cur_error: "
                <<cur_error<<", error_change: "<<error_change<<std::endl;
        // 若当前误差小，或者误差变化小，退出迭代
        if( error_change<epsilon)  
        {
            std::cerr<<"["<<iters<<"] coverage"<<std::endl;
            std::cerr<<"final cost: "<<cur_error<<", cost change: "<<error_change<<std::endl;
            //break;
        }
        if (dx.norm() < epsilon) {
            std::cerr << "converged, dx = " << dx.transpose()<<std::endl;
            break;
        }

        // Eigen::Matrix3d R_new = Sophus::SO3d::exp(dx.head<3>()).matrix();
        // Eigen::Vector3d t_new = dx.tail<3>();
        // Eigen::Matrix4d T_new;
        // T_new << R_new, t_new, 0, 0, 0, 1;
        // T_final = T_new*T_final; //更新变换矩阵  

        iters++;  // 迭代        
    }   
    initial_pose = pose;    
    clock_t end = clock();
    double time = double(end-start)/CLOCKS_PER_SEC;
    std::cout<<"spend time: "<<time*1000<<"ms"<<std::endl;
}
//原始点云绿色   目标点云红色 匹配好的点云蓝色
void pointcloud_visualize(const PointCloudType::Ptr target_cloud,
                     const PointCloudType::Ptr source_cloud,
                     Eigen::Matrix4d T)
{
    std::cerr<<"flage0.1 "<<std::endl;
	pcl::visualization::PCLVisualizer viewer("ricp_GSN Viewer");
    std::cerr<<"flage0.2 "<<std::endl;

    PointCloudType::Ptr icp_cloud(new PointCloudType());

    pcl::transformPointCloud(*source_cloud, *icp_cloud, T);
   
	pcl::visualization::PointCloudColorHandlerCustom<PointType> src_h(source_cloud, 0, 255, 0); 	//原始点云绿色
	pcl::visualization::PointCloudColorHandlerCustom<PointType> tgt_h(target_cloud, 255, 0, 0); 	//目标点云红色
	pcl::visualization::PointCloudColorHandlerCustom<PointType> final_h(icp_cloud, 0, 0, 255); 	//匹配好的点云蓝色

	viewer.setBackgroundColor(255, 255, 255);
	viewer.addPointCloud(source_cloud, src_h, "source cloud");
	viewer.addPointCloud(target_cloud, tgt_h, "target cloud");
	viewer.addPointCloud(icp_cloud, final_h, "result cloud");

	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		usleep(100000);
	}
    //delete viewer;
}
