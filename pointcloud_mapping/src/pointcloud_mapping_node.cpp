/*
 * 点云转发节点 - ROS1实现
 * 功能：订阅输入点云话题，转发到指定输出话题
 * 依赖：sensor_msgs, pcl_ros, pcl_conversions
*/
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>//以< x, y, z,curvature >形式定义一个新的点
{
using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
public:
    MyPointRepresentation ()
    {
        nr_dimensions_ = 4;
        //定义尺寸值
    }
    virtual void copyToFloatArray (const PointNormalT &p, float * out) const //覆盖copyToFloatArray 方法来定义我们的特征矢量
    {
        out[0] = p.x;
        out[1] = p.y;
        out[2] = p.z;
        out[3] = p.curvature;
    }
};

class PointCloudBridge {
public:
    PointCloudBridge() : nh_("~") {
        // 参数初始化
        nh_.param<std::string>("input_topic", input_topic_, "/lslidar_point_cloud");
        nh_.param<std::string>("output_topic", output_topic_, "/output_cloud");
        nh_.param<double>("publish_rate", publish_rate_, 10.0);

        // 订阅者初始化
        sub_ = nh_.subscribe(input_topic_, 1, &PointCloudBridge::cloudCallback, this);

        // 发布者初始化
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);
        
        ROS_INFO("Initialized with parameters:");
        ROS_INFO("Input Topic: %s", input_topic_.c_str());
        ROS_INFO("Output Topic: %s", output_topic_.c_str());
        ROS_INFO("Publish Rate: %.1f Hz", publish_rate_);

        //*global_map = new PointCloud;
        target.reset(new pcl::PointCloud<pcl::PointXYZ>);
        source.reset(new pcl::PointCloud<pcl::PointXYZ>);
        global_map.reset(new pcl::PointCloud<pcl::PointXYZ>);
    }
    void LidarAlignWithPCLNDT (const pcl::PointCloud<PointT>::Ptr source, const pcl::PointCloud<PointT>::Ptr target, 
                    PointCloud::Ptr output_pcl, Eigen::Matrix4f &final_transform ,bool downsample = false)
    {
            
        PointCloud::Ptr src (new PointCloud);
        PointCloud::Ptr tgt (new PointCloud);
        pcl::VoxelGrid<PointT> grid;//下采样
        if (downsample)//downsample 为真即对 src tgt 下采样以减少点数提高效率
        {
            grid.setLeafSize (0.05, 0.05, 0.05);
            grid.setInputCloud (source);
            grid.filter (*src);
            grid.setInputCloud (target);
            grid.filter (*tgt);
        }
        else//为假则直接使用原点云
        {
            src = source;
            tgt = target;
        }


        /// PCL 库中的 ICP  算法
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_pcl;
        icp_pcl.setInputSource(src);
        icp_pcl.setInputTarget(tgt);
         // 初始变换（假设无先验信息）
        final_transform = Eigen::Matrix4f::Identity();
        icp_pcl.align(*output_pcl, final_transform);
        final_transform = icp_pcl.getFinalTransformation();
     
        // /// PCL NDT  
        // pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_pcl;
        // ndt_pcl.setTransformationEpsilon(0.01);
        // ndt_pcl.setStepSize(0.1);
        // ndt_pcl.setResolution(0.2);
        // ndt_pcl.setMaximumIterations(35);
        // ndt_pcl.setInputSource(src);
        // ndt_pcl.setInputTarget(tgt);
        // // 初始变换（假设无先验信息）
        // final_transform = Eigen::Matrix4f::Identity();
        // ndt_pcl.align(*output_pcl, final_transform);
        // final_transform = ndt_pcl.getFinalTransformation();
        //std::cout << "score: " << ndt_pcl.getTransformationProbability()<<std::endl;

        output_pcl->height = 1;
        output_pcl->width = output_pcl->size();
     
        

        std::cout << "final_transform: " << final_transform.matrix()<<std::endl;

        //计算GT pose差异
        //double pose_error = (gt_pose.inverse() * T.cast<double>()).log().norm();
       // std::cout << "NDT PCL pose error: " << pose_error<<std::endl;

    }
    void pairAlign (const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, 
                    PointCloud::Ptr output, Eigen::Matrix4f &final_transform,
                         bool downsample = false)
//匹配一对点云数据集并且返回结果,cloud_src 和 tgt 为源、目标点云， output 为输出的配准结果的源点云， final_transform为源点云和目标点云间转换矩阵
    {
        PointCloud::Ptr src (new PointCloud);
        PointCloud::Ptr tgt (new PointCloud);
        pcl::VoxelGrid<PointT> grid;//下采样
        if (downsample)//downsample 为真即对 src tgt 下采样以减少点数提高效率
        {
            grid.setLeafSize (0.05, 0.05, 0.05);
            grid.setInputCloud (cloud_src);
            grid.filter (*src);
            grid.setInputCloud (cloud_tgt);
            grid.filter (*tgt);
        }
        else//为假则直接使用原点云
        {
            src = cloud_src;
            tgt = cloud_tgt;
        }


        PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);//创建用于存储有法线和曲率的点云
        PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);
        pcl::NormalEstimation<PointT, PointNormalT> norm_est;// 计算曲面法线和曲率
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>());//法线估计时搜索临近点
        norm_est.setSearchMethod (tree);
        norm_est.setKSearch (30);//搜索半径
        norm_est.setInputCloud (src);//输入点云
        norm_est.compute (*points_with_normals_src);//计算
        pcl::copyPointCloud (*src, *points_with_normals_src);//复制
        norm_est.setInputCloud (tgt);//同上
        norm_est.compute (*points_with_normals_tgt);
        pcl::copyPointCloud (*tgt, *points_with_normals_tgt);
        MyPointRepresentation point_representation;
        float alpha[4] = {1.0, 1.0, 1.0, 1.0};//相同权重
        point_representation.setRescaleValues (alpha);//调整'curvature'尺寸权重以便使它和 x, y, z平衡
        pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;//创建 ICP 对象
        reg.setTransformationEpsilon (1e-6);//设收敛阈值
        reg.setMaxCorrespondenceDistance (0.1); //设两对应间最大距离为 10 厘米，需根据数据集大小调整
        reg.setPointRepresentation (boost::make_shared<const MyPointRepresentation>(point_representation)); //设置点表示
        reg.setInputCloud (points_with_normals_src);
        reg.setInputTarget (points_with_normals_tgt);
        Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity (), prev, targetToSource;//4x4 变换矩阵
        PointCloudWithNormals::Ptr reg_global_map = points_with_normals_src;//存储匹配结果的点云指针
        reg.setMaximumIterations (2);//设最大迭代次数
        for (int i = 0; i < 30; ++i)//迭代
        {
            PCL_INFO ("Iteration Nr. %d.\n", i);
            points_with_normals_src = reg_global_map;//更新源点云为上次匹配结果reg.setInputCloud (points_with_normals_src);
            reg.align (*reg_global_map);
            Ti = reg.getFinalTransformation () * Ti;//在每一个迭代之间累积转换
            //如果这次转换和之前转换之间的差异小于阈值，则通过减小最大对应距离来改善程序
            if (fabs ((reg.getLastIncrementalTransformation () - prev).sum ()) < reg.getTransformationEpsilon ())
            {
                reg.setMaxCorrespondenceDistance (reg.getMaxCorrespondenceDistance () - 0.001);
                prev = reg.getLastIncrementalTransformation ();
            }
            //showCloudsRight(points_with_normals_tgt, points_with_normals_src); //可视化当前状态
        }


        targetToSource = Ti.inverse(); //得到目标点云到源点云的变换
        //把目标点云转换回源框架
        pcl::transformPointCloud (*cloud_tgt, *output, targetToSource);

        /* 添加源点云到转换目标 */
        *output += *cloud_src;
        final_transform = targetToSource;
    }
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        try {
            // 数据验证
            if (msg->data.empty()) {
                ROS_WARN_THROTTLE(5, "Received empty point cloud");
                return;
            }
  
            target->clear();
            pcl::fromROSMsg(*msg, *target);

            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*target, *target, indices);
            
            if(source->empty())
            {
                std::cout<<"first lidar fram"<<std::endl;
                source->clear();
                pcl::copyPointCloud(*target, *source);
                global_map->clear();
                pcl::copyPointCloud(*target, *global_map);
                return ;
            }
            //--------------------------------
  
            //showCloudsLeft(source, target);//左侧窗口显示点云
            PointCloud::Ptr temp (new PointCloud);
            Eigen::Matrix4f Ti; 
            std::cout<< "Aligning source pointcloude size():  "<< source->points.size ()<< "  target pointcloude size():  "<< target->points.size ()<<std::endl;
            LidarAlignWithPCLNDT(source, target, temp, Ti, false);//匹配并返回匹配后的点云及变换矩阵
            std::cout<<"Aligning lidar frame successed"<<std::endl;

            GlobalTransform = Ti * GlobalTransform;//更新全局变换
            temp->clear();
            pcl::transformPointCloud (*target, *temp, GlobalTransform.inverse(),true); //把当前的两两配对转换到全局变换

            *global_map += *temp;
 
            // 发布消息
            sensor_msgs::PointCloud2 output_msg;
            pcl::toROSMsg(*global_map, output_msg);
            pub_.publish(output_msg);
            ROS_INFO("Published point cloud with %d points", global_map->size());

            source->clear();
            pcl::copyPointCloud(*target, *source);
             
        }
        catch (const std::exception & e) {
            ROS_ERROR("Standard exception: %s", e.what());
        }
    }

public:
    double publish_rate_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr source,target;
    PointCloud::Ptr global_map ;//global_map 用于存储匹配后的点云
    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity ();
    
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    
    std::string input_topic_;
    std::string output_topic_;


    // 可选：添加自定义数据处理函数
    // bool perform_filtering(pcl::PointCloud<pcl::PointXYZ>& cloud) {
    //     // 实现体素网格滤波等算法
    //     return true;
    // }
};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "point_cloud_bridge");
    ros::NodeHandle nh;

    // 创建节点实例
    PointCloudBridge bridge;

    // 自定义循环频率
    ros::Rate rate(bridge.publish_rate_);

    ROS_INFO("Point cloud bridge node started");
    
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}