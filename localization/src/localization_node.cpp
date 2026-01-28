#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <algorithm> // 用于字符串处理

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl_ros/transforms.hpp> 
#include <pclomp/ndt_omp.h>

using namespace std::chrono_literals;

class LocalizationNode : public rclcpp::Node {
public:
    LocalizationNode() : Node("ndt_localization_node") {
        // ================= 参数声明 =================
        this->declare_parameter<std::string>("map_path", "/home/user/akm_ws2/map/3floor.pcd");
        this->declare_parameter<std::string>("lidar_topic", "/livox/center/lidar"); 
        this->declare_parameter<double>("ndt_resolution", 1.0);
        this->declare_parameter<double>("ndt_step_size", 0.1);
        
        std::string map_path;
        this->get_parameter("map_path", map_path);
        
        // 🔥 修改 1: 提取地图文件名 (用于显示)
        // 逻辑：找到最后一个 '/' 的位置，截取后面的部分
        size_t last_slash_idx = map_path.find_last_of("/\\");
        if (std::string::npos != last_slash_idx) {
            current_map_name_ = map_path.substr(last_slash_idx + 1);
        } else {
            current_map_name_ = map_path;
        }

        std::string lidar_topic;
        this->get_parameter("lidar_topic", lidar_topic);
        double ndt_res, ndt_step;
        this->get_parameter("ndt_resolution", ndt_res);
        this->get_parameter("ndt_step_size", ndt_step);

        // ================= 加载地图 =================
        pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        if (pcl::io::loadPCDFile(map_path, *map_cloud) == -1) {
            RCLCPP_ERROR(this->get_logger(), "🔥 无法加载地图文件: %s", map_path.c_str());
            exit(1);
        }
        RCLCPP_INFO(this->get_logger(), "✅ 地图 [%s] 加载成功，点数: %lu", current_map_name_.c_str(), map_cloud->size());

        // ================= 配置 NDT_OMP =================
        ndt_.setResolution(ndt_res);          // 网格大小
        ndt_.setStepSize(ndt_step);           // 步长
        ndt_.setTransformationEpsilon(0.01);  // 收敛阈值
        ndt_.setNeighborhoodSearchMethod(pclomp::DIRECT7); // 极速搜索模式
        ndt_.setNumThreads(4);                // 启用4核并行
        ndt_.setInputTarget(map_cloud);       // 设置目标地图

        // ================= 初始化 TF =================
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // ================= 订阅雷达 =================
        sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic, rclcpp::SensorDataQoS(), 
            std::bind(&LocalizationNode::lidar_callback, this, std::placeholders::_1));
        
        // ================= 订阅初始位姿 (RViz) =================
        sub_initial_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10,
            std::bind(&LocalizationNode::initial_pose_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "🚀 NDT 定位节点启动成功！当前地图: %s", current_map_name_.c_str());
    }

private:
    // ================= 初始位姿回调函数 =================
    void initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "收到 RViz 初始位置: x=%.2f, y=%.2f", 
            msg->pose.pose.position.x, msg->pose.pose.position.y);

        Eigen::Isometry3d target_pose;
        tf2::fromMsg(msg->pose.pose, target_pose);

        Eigen::Isometry3d current_odom_base;
        try {
            geometry_msgs::msg::TransformStamped t_odom_base = 
                tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
            current_odom_base = tf2::transformToEigen(t_odom_base);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "设置初始位置失败: 无法获取 odom->base_link: %s", ex.what());
            return;
        }

        Eigen::Matrix4d new_map_to_odom = target_pose.matrix() * current_odom_base.inverse().matrix();

        {
            std::lock_guard<std::mutex> lock(mtx_);
            map_to_odom_matrix_ = new_map_to_odom;
        }
        
        RCLCPP_INFO(this->get_logger(), "初始位置重置成功！");
    }

    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // 1. 接收原始数据
        pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *raw_cloud);

        // 源头过滤 (盲区剔除)
        pcl::PointCloud<pcl::PointXYZ>::Ptr clean_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        float min_dist_sq = 0.25 * 0.25; 

        for (const auto& pt : raw_cloud->points) {
            float dist_sq = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
            if (dist_sq > min_dist_sq) {
                if (pcl::isFinite(pt)) {
                    clean_cloud->push_back(pt);
                }
            }
        }
        
        clean_cloud->width = clean_cloud->points.size();
        clean_cloud->height = 1;
        clean_cloud->is_dense = true;
        clean_cloud->header = raw_cloud->header; 

        // 2. 变换到 base_link
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_base_link(new pcl::PointCloud<pcl::PointXYZ>());
        try {
            if (!pcl_ros::transformPointCloud("base_link", *clean_cloud, *cloud_in_base_link, *tf_buffer_)) {
                return; 
            }
        } catch (tf2::TransformException &ex) {
            return;
        }

        // 3. 降采样
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setLeafSize(0.5, 0.5, 0.5);
        voxel_grid.setInputCloud(cloud_in_base_link);
        voxel_grid.filter(*filtered_cloud);

        // 4. 获取初始猜测 (Initial Guess)
        Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
        
        try {
            geometry_msgs::msg::TransformStamped t_odom_base;
            t_odom_base = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
            Eigen::Isometry3d iso_odom_base = tf2::transformToEigen(t_odom_base);
            
            {
                std::lock_guard<std::mutex> lock(mtx_);
                initial_guess = (map_to_odom_matrix_ * iso_odom_base.matrix()).cast<float>();
            }
            
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "无法获取里程计: %s", ex.what());
            return;
        }

        // 5. 执行 NDT 匹配
        pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        ndt_.setInputSource(filtered_cloud);
        ndt_.align(*output_cloud, initial_guess); 

        // 6. 处理匹配结果
        if (ndt_.hasConverged()) {
            Eigen::Matrix4d t_map_base = ndt_.getFinalTransformation().cast<double>();
            
            try {
                geometry_msgs::msg::TransformStamped t_odom_base = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
                Eigen::Matrix4d matrix_odom_base = tf2::transformToEigen(t_odom_base).matrix();
                
                {
                    std::lock_guard<std::mutex> lock(mtx_);
                    map_to_odom_matrix_ = t_map_base * matrix_odom_base.inverse();
                
                    geometry_msgs::msg::TransformStamped tf_msg;
                    tf_msg.header.stamp = this->get_clock()->now();
                    tf_msg.header.frame_id = "map";
                    tf_msg.child_frame_id = "odom";
                    
                    Eigen::Quaterniond q(map_to_odom_matrix_.block<3, 3>(0, 0));
                    tf_msg.transform.translation.x = map_to_odom_matrix_(0, 3);
                    tf_msg.transform.translation.y = map_to_odom_matrix_(1, 3);
                    tf_msg.transform.translation.z = map_to_odom_matrix_(2, 3);
                    tf_msg.transform.rotation.x = q.x();
                    tf_msg.transform.rotation.y = q.y();
                    tf_msg.transform.rotation.z = q.z();
                    tf_msg.transform.rotation.w = q.w();

                    tf_broadcaster_->sendTransform(tf_msg);
                }
                
                // 🔥 修改 2: 在日志中包含地图名 (使用 current_map_name_)
                // 使用 RCLCPP_INFO_THROTTLE (每1秒打印一次，防止刷屏太快看不清)
                // 如果您想每一帧都看，就去掉 _THROTTLE 和 1000
                if (ndt_.getFitnessScore() > 1.0) {
                     RCLCPP_WARN(this->get_logger(), "[%s] NDT 分数较高: %.4f", current_map_name_.c_str(), ndt_.getFitnessScore());
                } else {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                        "[%s] NDT 正常: %.4f", current_map_name_.c_str(), ndt_.getFitnessScore());
                }

            } catch (tf2::TransformException &ex) {
                RCLCPP_ERROR(this->get_logger(), "TF 失败: %s", ex.what());
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "[%s] NDT 匹配未收敛！", current_map_name_.c_str());
        }
    }

    pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initial_pose_; 
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    std::mutex mtx_; 
    Eigen::Matrix4d map_to_odom_matrix_ = Eigen::Matrix4d::Identity();

    // 🔥 新增：用于存储地图文件名的成员变量
    std::string current_map_name_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizationNode>());
    rclcpp::shutdown();
    return 0;
}