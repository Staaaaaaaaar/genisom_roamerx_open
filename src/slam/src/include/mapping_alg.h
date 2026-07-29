/**
 * @file mapping_alg.h
 * @brief
 * @author Liuzhao Li (liliuzhao@jushenzhiren.com)
 * @version 1.0
 * @date 2025-07-31
 * @copyright Copyright (C) 2025 具身智人(北京)科技有限公司
 */

#pragma once
#include "common/state_mode.h"
#include "ikd_tree/ikd_tree.h"
#include "pcd2grid.h"
#include "process/imu_process.h"
#include "process/lidar_process.h"
#include "so3_math.h"

#include <Eigen/Core>
#include <chrono>
#include <csignal>
#include <fstream>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <math.h>
#include <mtk_iekf/esekfom/esekfom.hpp>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <robots_dog_msgs/srv/map_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <thread>
#include <unordered_map>
#include <unistd.h>
#include <visualization_msgs/msg/marker.hpp>
namespace robot::slam
{
    class MappingAlg : public rclcpp::Node
    {
    public:
        MappingAlg(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        ~MappingAlg();

        void run();

        void reset();

    private:
        double get_time_sec(const builtin_interfaces::msg::Time& time);

        rclcpp::Time get_ros_time(double timestamp);

        void init();

        void pointsBody2World(PointType const* const pi, PointType* const po);

        void pointsBody2Imu(PointType const* const pi, PointType* const po);

        void points_cache_collect();

        void lasermap_fov_segment();

        void lidarCallBack(const sensor_msgs::msg::PointCloud2::UniquePtr msg);

        void imuCallBack(const sensor_msgs::msg::Imu::UniquePtr msg_in);

        bool syncData(MeasureGroup& meas);

        void map_incremental();

        void pubWorldPoints(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull);

        void pubBodyPoints(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body);

        void pubMapPoints(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap);

        void accumulateMapPoints(const PointCloudType& cloud);

        CloudPtr snapshotAccumulatedMap();

        void resetSyncBuffersLocked();

        void stateCallBack(
            robots_dog_msgs::srv::MapState::Request::SharedPtr request, robots_dog_msgs::srv::MapState::Response::SharedPtr response);

        void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped,
            std::unique_ptr<tf2_ros::TransformBroadcaster>&                               tf_br);

        void publish_path(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath);

        void map_publish_callback();

        void h_share_model(state_ikfom& s, esekfom::dyn_share_datastruct<double>& ekfom_data);

        template <typename T>
        void pointBodyToWorld(const Eigen::Matrix<T, 3, 1>& pi, Eigen::Matrix<T, 3, 1>& po)
        {
            Vec3d p_body(pi[0], pi[1], pi[2]);
            Vec3d p_global(state_point.rot * (state_point.offset_R_L_I * p_body + state_point.offset_T_L_I) + state_point.pos);

            po[0] = p_global(0);
            po[1] = p_global(1);
            po[2] = p_global(2);
        }

        template <typename T>
        void set_posestamp(T& out)
        {
            Vec3d base_position;
            Mat3d base_rotation;
            get_base_pose(base_position, base_rotation);
            const Quatd base_orientation(base_rotation);

            out.pose.position.x    = base_position.x();
            out.pose.position.y    = base_position.y();
            out.pose.position.z    = base_position.z();
            out.pose.orientation.x = base_orientation.x();
            out.pose.orientation.y = base_orientation.y();
            out.pose.orientation.z = base_orientation.z();
            out.pose.orientation.w = base_orientation.w();
        }

        void get_base_pose(Vec3d& base_position, Mat3d& base_rotation) const;

        inline double QuaternionToYaw(double x, double y, double z, double w)
        {
            return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
        }


    private:
        bool extrinsic_est_en = true, path_en = false;
        bool odom_pub_en = true, tf_pub_en = false;
        bool save_map_en = false;

        float       DET_RANGE              = 300.0f;
        const float MOV_THRESHOLD          = 1.5f;
        double      time_diff_lidar_to_imu = 0.0;
        double      save_map_voxel_size    = 0.2;

        std::mutex              mtx_buffer;
        std::mutex              map_storage_mutex;
        std::condition_variable sig_buffer;
        std::string             root_dir_ = ROOT_DIR;
        std::string             lid_topic, imu_topic;
        std::string             odom_topic = "/slam_odom";
        std::string             odom_frame_id = "map", odom_child_frame_id = "body";
        std::string             tf_frame_id = "map", tf_child_frame_id = "body";
        std::string             data_path_;

        double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
        double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
        double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
        double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
        int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0;
        int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0;
        bool   lidar_pushed = false, flg_first_scan = true, flg_EKF_inited = false;
        bool   pub_world_points_flag_ = false, pub_body_points_flag_ = false;
        bool   is_first_lidar = true;
        bool   map_limit_warned = false;

        int64_t max_lidar_buffer_size = 50;
        int64_t max_imu_buffer_size   = 2000;
        int64_t max_path_poses        = 10000;
        int64_t save_map_max_points   = 2000000;

        Pcd2GridOptions           pcd2pgm_options_;
        std::shared_ptr<Pcd2Grid> pcd2grid_ptr_;

        std::vector<float>         res_last;
        std::vector<uint8_t>       point_selected_surf;
        std::vector<vector<int>>  pointSearchInd_surf;
        std::vector<BoxPointType> cub_needrm;
        std::vector<PointVector>  Nearest_Points;
        std::vector<double>       extrinT;
        std::vector<double>       extrinR;
        std::vector<double>       base_to_imu_translation_param;
        std::vector<double>       base_to_imu_rotation_param;
        std::deque<double>        time_buffer;
        std::deque<CloudPtr>      lidar_buffer;
        std::deque<ImuMessagePtr> imu_buffer;

        CloudPtr featsFromMap     = CloudPtr(new PointCloudType());
        CloudPtr feats_undistort  = CloudPtr(new PointCloudType());
        CloudPtr feats_down_body  = CloudPtr(new PointCloudType());
        CloudPtr feats_down_world = CloudPtr(new PointCloudType());
        CloudPtr normvec          = CloudPtr(new PointCloudType());
        CloudPtr laserCloudOri    = CloudPtr(new PointCloudType());
        CloudPtr corr_normvect    = CloudPtr(new PointCloudType());
        CloudPtr _featsArray      = CloudPtr(new PointCloudType());

        pcl::VoxelGrid<PointType> downSizeFilterSurf;
        pcl::VoxelGrid<PointType> downSizeFilterMap;

        KD_TREE<PointType> ikdtree;

        Vec3d euler_cur;
        Vec3d position_last   = Zero3d;
        Vec3d Lidar_T_wrt_IMU = Zero3d;
        Mat3d Lidar_R_wrt_IMU = Eye3d;
        Vec3d base_to_imu_translation = Zero3d;
        Mat3d base_to_imu_rotation    = Eye3d;

        MeasureGroup                                 Measures;
        esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
        state_ikfom                                  state_point;
        vect3                                        pos_lid;

    public:
        void finish();

    private:
        std::shared_ptr<Preprocess> p_pre = std::make_shared<Preprocess>();
        std::shared_ptr<ImuProcess> p_imu = std::make_shared<ImuProcess>();

    private:
        nav_msgs::msg::Path             path;
        nav_msgs::msg::Odometry         odomAftMapped;
        geometry_msgs::msg::Quaternion  geoQuat;
        geometry_msgs::msg::PoseStamped msg_body_pose;

        BoxPointType LocalMap_Points;
        bool         Localmap_Initialized = false;

        double lidar_mean_scantime = 0.0;
        int    scan_num            = 0;

        struct VoxelKey
        {
            int64_t x;
            int64_t y;
            int64_t z;

            bool operator==(const VoxelKey& other) const
            {
                return x == other.x && y == other.y && z == other.z;
            }
        };

        struct VoxelKeyHash
        {
            std::size_t operator()(const VoxelKey& key) const
            {
                std::size_t seed = std::hash<int64_t>{}(key.x);
                seed ^= std::hash<int64_t>{}(key.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
                seed ^= std::hash<int64_t>{}(key.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
                return seed;
            }
        };

        std::unordered_map<VoxelKey, PointType, VoxelKeyHash> accumulated_map;

        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pubLaserCloudFull_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pubLaserCloudFull_body_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pubLaserCloudMap_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr          pubOdomAftMapped_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr              pubPath_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr         sub_imu_ptr_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_ptr_;

        rclcpp::Service<robots_dog_msgs::srv::MapState>::SharedPtr state_service_;

        std::atomic<SlamState> state_{ SlamState::STABLE };

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        rclcpp::TimerBase::SharedPtr                   map_pub_timer_;

        bool effect_pub_en = false, map_pub_en = false;
    };

}  // namespace robot::slam
