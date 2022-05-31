#ifndef ODOMETRY_HPP_
#define ODOMETRY_HPP_

#include "visibility.h"
#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <ld_slam_msg/msg/map_array.hpp>
#include "lidar_undistortion.hpp"

#include <pclomp/ndt_omp.h>
#include <pclomp/ndt_omp_impl.hpp>
#include <pclomp/voxel_grid_covariance_omp.h>
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>
#include <pclomp/gicp_omp.h>
#include <pclomp/gicp_omp_impl.hpp>

#include "laser_geometry/laser_geometry.hpp"

#include <mutex>
#include <thread>
#include <future>
#include <cstdlib>

#include <pcl_conversions/pcl_conversions.h>

namespace ldslam
{
  class Odometry: public rclcpp::Node
  {
  public:
      COMPOSITION_PUBLIC
      explicit Odometry(const rclcpp::NodeOptions & options);

  private:
      std::string robot_name_;

      rclcpp::Clock clock_;
      tf2_ros::Buffer tfbuffer_;
      tf2_ros::TransformListener listener_;
      tf2_ros::TransformBroadcaster broadcaster_;

      std::string global_frame_id_;
      std::string robot_frame_id_;
      std::string odom_frame_id_;

      pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr registration_;

      std::thread threads_;

      std::mutex mtx_;
      pcl::PointCloud < pcl::PointXYZI > targeted_cloud_;
      rclcpp::Time last_map_time_;
      bool mapping_flag_ {false};
      bool is_map_updated_ {false};
      std::thread mapping_thread_;
      std::packaged_task < void() > mapping_task_;
      std::future < void > mapping_future_;

      geometry_msgs::msg::PoseStamped corrent_pose_stamped_;
      ld_slam_msg::msg::MapArray map_array_msg_;
      nav_msgs::msg::Path path_;

      rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr initial_pose_sub_;
      rclcpp::Subscription< sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
      rclcpp::Subscription< sensor_msgs::msg::LaserScan>::SharedPtr input_laser_scan_sub_;
      rclcpp::Subscription< sensor_msgs::msg::PointCloud2>::SharedPtr input_cloud_sub_;
      
      rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
      rclcpp::Publisher<ld_slam_msg::msg::MapArray>::SharedPtr map_array_pub_;
      rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;


      laser_geometry::LaserProjection projector_;

      void setParams();
      void initializePubSub();
      void setInitialPose();

      
      void receiveCloud(
        const pcl::PointCloud < pcl::PointXYZI > ::ConstPtr & input_cloud_ptr,
        const rclcpp::Time stamp);

      void receiveImu(const sensor_msgs::msg::Imu imu_msg);
      
      void publishMapAndPose(
        const pcl::PointCloud < pcl::PointXYZI > ::ConstPtr & cloud_ptr,
        const Eigen::Matrix4f final_transformation,
        const rclcpp::Time stamp
      );
      
      Eigen::Matrix4f getTransformation(const geometry_msgs::msg::Pose pose);
      
      void publishMap();
      
      void updateMap(
        const pcl::PointCloud < pcl::PointXYZI > ::ConstPtr cloud_ptr,
        const Eigen::Matrix4f final_transformation,
        const geometry_msgs::msg::PoseStamped corrent_pose_stamped
      );

      void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
      void initial_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
      void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
      void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

      bool initial_pose_received_ {false};
      bool initial_cloud_received_ {false};

      // setting parameter
      std::string registration_method_;
      double trans_for_mapupdate_;
      double vg_size_for_input_;
      double vg_size_for_map_;
      bool use_min_max_filter_ {false};
      double scan_min_range_ {0.1};
      double scan_max_range_ {100.0};
      double map_publish_period_;
      int num_targeted_cloud_;

      double ndt_resolution;
      int ndt_num_threads;
      double gicp_corr_dist_threshold;

      bool set_initial_pose_ {false};
      bool use_odom_ {false};
      bool use_imu_ {false};
      bool debug_flag_ {false};

      // map
      Eigen::Vector3d previous_position_;
      double trans_;
      double latest_distance_ {0};

      // initial_pose
      double initial_pose_x_;
      double initial_pose_y_;
      double initial_pose_z_;
      double initial_pose_qx_;
      double initial_pose_qy_;
      double initial_pose_qz_;
      double initial_pose_qw_;

      // odom
      Eigen::Matrix4f previous_odom_mat_ {Eigen::Matrix4f::Identity()};

      // imu
      double scan_period_ {0.1};
      LidarUndistortion lidar_undistortion_;

  };
}

#endif  // ODOMETRY_HPP_
