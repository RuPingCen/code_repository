/******************************************************************************** 
** @Copyright(c) $year$ $registered organization$ All Rights Reserved.
** @auth： crp
** @date： 2024/04/30
** @desc： 利用pcl的kd加速匹配，对两帧点云进行匹配，利用SVD求解优化
** @Ref : 参考：https://github.com/taifyang/MyICP/blob/main/main.cpp
*********************************************************************************/

#include "icp_svd.h"
 

ICP_SVD::ICP_SVD(float error,int iters,float eps)
{
	min_error = error;
	max_iters = iters;
	epsilon = eps;
	leaf_size = 0.01;
}

ICP_SVD::~ICP_SVD()
{
}

void ICP_SVD::setSourceCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) 
{
	source_cloud = cloud;
}

void ICP_SVD::setTargetCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
	target_cloud = cloud;
}
 
void ICP_SVD::pointcloudFilter()
{
	pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
	voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);

	voxel_grid.setInputCloud(source_cloud);
	source_cloud_downsampled.reset(new pcl::PointCloud<pcl::PointXYZI>);
	voxel_grid.filter(*source_cloud_downsampled);

	voxel_grid.setInputCloud(target_cloud);
	target_cloud_downsampled.reset(new pcl::PointCloud<pcl::PointXYZI>);
	voxel_grid.filter(*target_cloud_downsampled);

	std::cout << "voxel filter size *cloud_src_o from " << source_cloud->size() << " to " << source_cloud_downsampled->size() << endl;
	std::cout << "voxel filter size *cloud_tgt_o from " << target_cloud->size() << " to " << target_cloud_downsampled->size() << endl;
}

void ICP_SVD::registration()
{
	std::cout << "icp registration start..." << std::endl;

	Eigen::Matrix3f R_12 = Eigen::Matrix3f::Identity();
	Eigen::Vector3f T_12 = Eigen::Vector3f::Zero();
	Eigen::Matrix4f H_12 = Eigen::Matrix4f::Identity();

	pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud_mid(new pcl::PointCloud<pcl::PointXYZI>());

	//建立kd树
	pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>);
	kdtree->setInputCloud(target_cloud_downsampled);

	double error = INT_MAX, score = INT_MAX;
	Eigen::Matrix4f H_final = H_12;
	int iters = 0;

	//开始迭代，直到满足条件
	while (error > min_error && iters < max_iters)
	{
		iters++;
		double last_error = error;

		//计算最邻近点对
		calNearestPointPairs(H_12, source_cloud_downsampled, target_cloud_downsampled, target_cloud_mid, kdtree, error);

		if (last_error - error < epsilon)
			break;

		// //
		// //计算点云中心坐标
		// Eigen::Vector4f source_centroid, target_centroid_mid;
		// pcl::compute3DCentroid(*source_cloud_downsampled, source_centroid);
		// pcl::compute3DCentroid(*target_cloud_mid, target_centroid_mid);

		// //去中心化
		// Eigen::MatrixXf souce_cloud_demean, target_cloud_demean;
		// pcl::demeanPointCloud(*source_cloud_downsampled, source_centroid, souce_cloud_demean);
		// pcl::demeanPointCloud(*target_cloud_mid, target_centroid_mid, target_cloud_demean);

		// //计算W=q1*q2^T
		// Eigen::Matrix3f W = (souce_cloud_demean*target_cloud_demean.transpose()).topLeftCorner(3, 3);
		 
		Eigen::Vector3f center_src(0.0,0.0,0.0);
		Eigen::Vector3f center_tar(0.0,0.0,0.0);
		for(size_t i = 0;i<target_cloud_mid->size();i++)        // 遍历匹配src中的每一个点
		{
			PointType p_src = source_cloud_downsampled->points[i];
			PointType p_tar = target_cloud_mid->points[i];

			center_src+=Eigen::Vector3f(p_src.x,p_src.y,p_src.z);
			center_tar+=Eigen::Vector3f(p_tar.x,p_tar.y,p_tar.z);
		}
		center_src/=double(target_cloud_mid->size());
		center_tar/=double(target_cloud_mid->size());

		// 2.2 求W
		Eigen::Matrix3f W(Eigen::Matrix3f::Identity());  // 记得矩阵W变量给初值，否则会报错！！！
		for(size_t i = 0;i<target_cloud_mid->size();i++)
		{
			PointType p_src = source_cloud_downsampled->points[i];
			PointType p_tar = target_cloud_mid->points[i];
			W+=(Eigen::Vector3f(p_src.x,p_src.y,p_src.z)-center_src)
				*((Eigen::Vector3f(p_tar.x,p_tar.y,p_tar.z)-center_tar).transpose() );// 3*3的矩阵
		}

		//SVD分解得到新的旋转矩阵和平移矩阵
		Eigen::JacobiSVD<Eigen::Matrix3f> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::Matrix3f U = svd.matrixU();
		Eigen::Matrix3f V = svd.matrixV();

		if (U.determinant()*V.determinant() < 0)
		{
			for (int x = 0; x < 3; ++x)
				V(x, 2) *= -1;
		}

		R_12 = V* U.transpose();
		//T_12 = target_centroid_mid.head(3) - R_12*source_centroid.head(3);
		T_12 = center_tar - R_12*center_src;
		H_12 << R_12, T_12, 0, 0, 0, 1;
		H_final = H_12*H_final; //更新变换矩阵

		std::cout << "iters:"  << iters << "  "<< "error:" << error << std::endl;
	}
	transformation_matrix << H_final;
}
void ICP_SVD::calNearestPointPairs(Eigen::Matrix4f H,
						pcl::PointCloud<pcl::PointXYZI>::Ptr& source_cloud, 
						const pcl::PointCloud<pcl::PointXYZI>::Ptr& target_cloud,
						pcl::PointCloud<pcl::PointXYZI>::Ptr& target_cloud_mid, 
						pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree, double &error)
{
	double err = 0.0;
	pcl::transformPointCloud(*source_cloud, *source_cloud, H);
	std::vector<int>indexs(source_cloud->size());

 	#pragma omp parallel for reduction(+:err) // 
	for (int i = 0; i < source_cloud->size(); ++i)
	{
		std::vector<int>index(1);
		std::vector<float>distance(1);
		kdtree->nearestKSearch(source_cloud->points[i], 1, index, distance);
		err = err + sqrt(distance[0]);
		indexs[i] = index[0];
	}

	pcl::copyPointCloud(*target_cloud, indexs, *target_cloud_mid);
	error = err / source_cloud->size();
}

 


void ICP_SVD::saveICPCloud(const std::string filename)
{
	icp_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::transformPointCloud(*source_cloud, *icp_cloud, transformation_matrix); //点云变换
	pcl::io::savePCDFileBinary(filename, *icp_cloud);
}

void ICP_SVD::getTransformationMatrix()
{
	std::cout << "transformation_matrix:" << std::endl << transformation_matrix << std::endl;
}

void ICP_SVD::getScore()
{
	double fitness_score = 0.0;
	pcl::KdTreeFLANN <pcl::PointXYZI> kdtree;
	kdtree.setInputCloud(target_cloud);

#pragma omp parallel for reduction(+:fitness_score) //采用openmmp加速
	for (int i = 0; i < icp_cloud->points.size(); ++i)
	{
		std::vector<int> nn_indices(1);
		std::vector<float> nn_dists(1);
		kdtree.nearestKSearch(icp_cloud->points[i], 1, nn_indices, nn_dists);
		fitness_score += nn_dists[0];
	}

	std::cout << "icp svd score:" << std::endl << fitness_score / icp_cloud->points.size() << std::endl;
}

void ICP_SVD::visualize()
{
	pcl::visualization::PCLVisualizer viewer("registration Viewer");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> src_h(source_cloud, 0, 255, 0); 	//原始点云绿色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> tgt_h(target_cloud, 255, 0, 0); 	//目标点云红色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> final_h(icp_cloud, 0, 0, 255); 	//匹配好的点云蓝色

	viewer.setBackgroundColor(255, 255, 255);
	viewer.addPointCloud(source_cloud, src_h, "source cloud");
	viewer.addPointCloud(target_cloud, tgt_h, "target cloud");
	viewer.addPointCloud(icp_cloud, final_h, "result cloud");

	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		usleep(100000);
	}
}



