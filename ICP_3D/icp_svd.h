/******************************************************************************** 

*********************************************************************************/

#ifndef ICP_SVD_H_
#define ICP_SVD_H_

#include <iostream>
#include <fstream>
#include <ctime>
#include <cmath>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h> 
#include <pcl/correspondence.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;
 
class ICP_SVD
{
public:
        /**
         * @brief ICP_SVD ���캯��
         */
        ICP_SVD(float error,int iters,float eps);

	~ICP_SVD();

        /**
         * @brief setSourceCloud 设置src点云
         * @param cloud 输入点云
         */
	void setSourceCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

        /**
         * @brief setTargetCloud 设置目标点云
         * @param cloud Ŀ�����
         */
	void setTargetCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

        /**
         * @brief downsample 点云滤波
         */
	void pointcloudFilter();

        /**
         * @brief registration ��׼
         */
	void registration();

        /**
         * @brief saveICPCloud 保存点云
         * @param filename 保存点云的路径
         */
	void saveICPCloud(const std::string filename);

        /**
         * @brief getTransformationMatrix  
         */
	void getTransformationMatrix();

        /**
         * @brief getScore  
         */
	void getScore();

        /**
         * @brief visualize  
         */
	void visualize();

private:
	pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud,target_cloud,
                                             source_cloud_downsampled,target_cloud_downsampled,
                                             icp_cloud;

	float leaf_size,min_error,max_iters,epsilon;

	Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();


        /**
         * @brief calNearestPointPairs  利用KD树加速搜索
         * @param H  
         * @param source_cloud 源点云
         * @param target_cloud 目标点云
         * @param target_cloud_mid �м����
         * @param kdtree 构建的KD树
         * @param error 匹配误差
         */
        void calNearestPointPairs(Eigen::Matrix4f H, pcl::PointCloud<pcl::PointXYZI>::Ptr& source_cloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr& target_cloud,
                                pcl::PointCloud<pcl::PointXYZI>::Ptr& target_cloud_mid, pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree, double &error);

};

#endif // ICP_SVD_H
