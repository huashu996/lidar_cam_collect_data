#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/console/time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <functional>
#include <tf/transform_broadcaster.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
template<typename PointType>
class GroundSeg:public pcl::PCLBase<PointType>
{
public:
    GroundSeg(){
    }
 
    void seg(pcl::PointIndices::Ptr& ground_indices,pcl::PointIndices::Ptr&  no_ground_indices) {
        bool init_ret = this->initCompute();
        if(init_ret == false) {
            return;
        }
        pcl::console::TicToc tt;
        typename pcl::search::KdTree<PointType>::Ptr kdtree_ptr(new pcl::search::KdTree<PointType>);
        pcl::PointCloud<pcl::Normal>::Ptr normal_ptr(new pcl::PointCloud<pcl::Normal>);
        pcl::PointIndices::Ptr indices(new pcl::PointIndices());
        indices->indices.reserve(this->getInputCloud()->size());
        {
            tt.tic();
            pcl::NormalEstimationOMP<PointType,pcl::Normal> normal_est_omp;
            normal_est_omp.setInputCloud(this->getInputCloud());
            //normal_est_omp.setIndices(this->getIndices());
            normal_est_omp.setSearchMethod(kdtree_ptr);
            normal_est_omp.setViewPoint(1e9,1e9,1e9);
            normal_est_omp.setKSearch(35); //neighbour size
            normal_est_omp.compute(*normal_ptr);
            {
                for(int i = 0; i < normal_ptr->points.size();++i) {
                    const auto normal = normal_ptr->at(i);
                    const double abs_normal_z = std::abs(normal.normal_z);
                    if(std::isnormal(abs_normal_z) && abs_normal_z > 0.5) {
                        indices->indices.push_back(i);
                    }
                }
            }
            std::cout <<__FUNCTION__ << ":" <<__LINE__<<",calc normal cost " << tt.toc() << " ms" << std::endl;
        }
#if 1
        {
            tt.tic();
            pcl::SACSegmentation<pcl::PointXYZRGBNormal> seg;
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            //pcl::PointIndices::Ptr ground_inliers(new pcl::PointIndices);
 
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setDistanceThreshold(0.3);
            seg.setInputCloud(this->getInputCloud());
            seg.setIndices(indices);
            //std::cout << __FUNCTION__ << __LINE__ << std::endl;
            seg.segment(*ground_indices,*coefficients);
            {
                pcl::ExtractIndices<PointType> extract;   //点提取对象
                extract.setInputCloud(this->getInputCloud());
                extract.setIndices(ground_indices);
                extract.setNegative(true);//设置成true是保存滤波后剩余的点，false是保存在区域内的点
                extract.filter(no_ground_indices->indices);
            }
            std::cout <<__FUNCTION__ << ":" <<__LINE__<<",ransac seg cost " << tt.toc() << " ms" << std::endl;
        }
#endif
        this->deinitCompute();
        //std::cout << __FUNCTION__ << __LINE__ <<"end"<< std::endl;
    }
};
 
int main(int argc, char** argv) {
    ros::init(argc, argv, "n_ground_seg");
 
    ros::NodeHandle node;
    ros::Publisher a_pub = node.advertise<sensor_msgs::PointCloud2>("seg_pointcloud",1);
    ros::Publisher g_pub = node.advertise<sensor_msgs::PointCloud2>("ground_pointcloud",1);
    ros::Publisher ng_pub = node.advertise<sensor_msgs::PointCloud2>("noground_pointcloud",1);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr trans_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointIndices::Ptr  ground_indices(new pcl::PointIndices);
    pcl::PointIndices::Ptr  no_ground_indices(new pcl::PointIndices);
    GroundSeg<pcl::PointXYZRGBNormal> ground_seg;
    sensor_msgs::PointCloud2 msg_all;
    sensor_msgs::PointCloud2 msg_g;
    sensor_msgs::PointCloud2 msg_ng;
    const boost::function<void (const boost::shared_ptr<sensor_msgs::PointCloud2 const>&)> callback =[&](sensor_msgs::PointCloud2::ConstPtr msg_pc_ptr) {
        pcl::fromROSMsg(*msg_pc_ptr, *cloud);  //ROS消息转换
        //坐标系转换
        // 创建点云对象　指针
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		// 源点云读取　获取　后
		pcl::RadiusOutlierRemoval<pcl::PointXYZRGBNormal> outrem;  //创建滤波器
		outrem.setInputCloud(cloud);    //设置输入点云
		outrem.setRadiusSearch(0.1f);    //设置半径为0.8的范围内找临近点
		outrem.setMinNeighborsInRadius (10);//设置查询点的邻域点集数小于2的删除
		// apply filter
		outrem.filter (*cloud_filtered);//执行条件滤波  在半径为0.8 在此半径内必须要有两个邻居点，此点才会保存
        //雷达周围点云去除
		pcl::PassThrough<pcl::PointXYZRGBNormal> pass;
        pass.setInputCloud (cloud_filtered);
        pass.setFilterFieldName ("x");
        pass.setFilterLimits (0, 0.3);
        pass.setFilterLimitsNegative (true);
        pass.filter (*new_cloud);
        
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		// 在 X 轴上定义一个 2.5 米的平移.
		transform.translation() << 0.0, 0.0, 0.0;
		// 和前面一样的旋转; Z 轴上旋转 theta 弧度
		float theta = -M_PI/2; // 弧度角
		//float theta = 0; // 弧度角
		transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitY()));
		// 打印变换矩阵
		printf ("\nMethod #2: using an Affine3f\n");
		std::cout << transform.matrix() << std::endl;
		pcl::transformPointCloudWithNormals(*new_cloud, *trans_cloud, transform);//用pcl不要用pcl_ros
		//点云分割
        ground_seg.setInputCloud(trans_cloud);
        ground_seg.seg(ground_indices, no_ground_indices);
        std::cerr << "ground points: " << ground_indices->indices.size() << std::endl;
        std::cerr << "no_ground points: " << no_ground_indices->indices.size() << std::endl;
        for (const auto& idx : ground_indices->indices) {
            auto& point = trans_cloud->at(idx);
            point.r = 0;
            point.g = 255;
            point.b = 0;
            cloud1->points.push_back(point); 
        }
        for (const auto& idx : no_ground_indices->indices) {
            auto& point = trans_cloud->at(idx);
            point.r = 255;
            point.g = 0;
            point.b = 0;
            cloud2->points.push_back(point); 
        }

        
        //发布地面点
        pcl::toROSMsg(*cloud1,msg_g);
        msg_g.header.frame_id = "livox_frame";
        g_pub.publish(msg_g);
        
        //发布非地面点
        pcl::toROSMsg(*cloud2,msg_ng);
        msg_ng.header.frame_id = "livox_frame";
        ng_pub.publish(msg_ng);
        //发布全部点云
        pcl::toROSMsg(*trans_cloud,msg_all);
        msg_all.header.frame_id = "livox_frame";
        a_pub.publish(msg_all);
        //之后要清空
        cloud1->clear();
        cloud2->clear();
        cloud_filtered->clear();
        new_cloud->clear();
        trans_cloud->clear();
    };
    ros::Subscriber pc_sub = node.subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 1, callback);
    ros::spin();
    return 0;
}
