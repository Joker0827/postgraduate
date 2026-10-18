#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/eigen.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <vector>
#include <yaml-cpp/yaml.h>
#include "sensor_simulator.cuh"
#include <chrono>
#include "maps.hpp"
#include "simulator_config.hpp"

using namespace raycast;

class SensorSimulator : public rclcpp::Node {
public:
    SensorSimulator() : Node("sensor_simulator") {
        const auto config_file = declare_parameter<std::string>("config_file", simulator_config::defaultConfig());
        YAML::Node config = simulator_config::load(config_file);
        ensureCudaDevice();
        // 读取camera参数
        camera = new CameraParams();
        camera->fx = config["camera"]["fx"].as<float>();
        camera->fy = config["camera"]["fy"].as<float>();
        camera->cx = config["camera"]["cx"].as<float>();
        camera->cy = config["camera"]["cy"].as<float>();
        camera->image_width = config["camera"]["image_width"].as<int>();
        camera->image_height = config["camera"]["image_height"].as<int>();
        camera->max_depth_dist = config["camera"]["max_depth_dist"].as<float>();
        camera->normalize_depth = config["camera"]["normalize_depth"].as<bool>();
        float pitch = config["camera"]["pitch"].as<float>() * M_PI / 180.0;
        quat_bc = Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY());

        // 读取lidar参数
        lidar = new LidarParams();
        lidar->vertical_lines = config["lidar"]["vertical_lines"].as<int>();
        lidar->vertical_angle_start = config["lidar"]["vertical_angle_start"].as<float>();
        lidar->vertical_angle_end = config["lidar"]["vertical_angle_end"].as<float>();
        lidar->horizontal_num = config["lidar"]["horizontal_num"].as<int>();
        lidar->horizontal_resolution = config["lidar"]["horizontal_resolution"].as<float>();
        lidar->max_lidar_dist = config["lidar"]["max_lidar_dist"].as<float>();

        render_lidar = config["render_lidar"].as<bool>();
        render_depth = config["render_depth"].as<bool>();
        float depth_fps = config["depth_fps"].as<float>();
        float lidar_fps = config["lidar_fps"].as<float>();
        if (depth_fps <= 0 || lidar_fps <= 0) {
            throw std::runtime_error("depth_fps and lidar_fps must be positive");
        }
        depth_pub_duration = rclcpp::Duration::from_seconds(1.0 / depth_fps);
        lidar_pub_duration = rclcpp::Duration::from_seconds(1.0 / lidar_fps);
        
        std::string ply_file = config["ply_file"].as<std::string>();
        std::string odom_topic = config["odom_topic"].as<std::string>();
        std::string depth_topic = config["depth_topic"].as<std::string>();
        std::string lidar_topic = config["lidar_topic"].as<std::string>();

        // 读取地图参数
        bool use_random_map = config["random_map"].as<bool>();
        float resolution = config["resolution"].as<float>();
        int occupy_threshold = config["occupy_threshold"].as<int>();
        pcl_pub = create_publisher<sensor_msgs::msg::PointCloud2>("/mock_map", rclcpp::QoS(1).transient_local().reliable());
        int seed = config["seed"].as<int>();
        int sizeX = config["x_length"].as<int>();
        int sizeY = config["y_length"].as<int>();
        int sizeZ = config["z_length"].as<int>();
        int type = config["maze_type"].as<int>();
        double scale = 1 / resolution;
        sizeX = sizeX * scale;
        sizeY = sizeY * scale;
        sizeZ = sizeZ * scale;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        if (use_random_map) {
            printf("1.Generate Random Map... \n");
            mocka::Maps::BasicInfo info;
            info.sizeX      = sizeX;
            info.sizeY      = sizeY;
            info.sizeZ      = sizeZ;
            info.seed       = seed;
            info.scale      = scale;
            info.cloud      = cloud;

            mocka::Maps map;
            map.setParam(config);
            map.setInfo(info);
            map.generate(type);
        }
        else {
            printf("1.Reading Point Cloud %s... \n", ply_file.c_str());
            if (pcl::io::loadPLYFile(ply_file, *cloud) == -1) {
                throw std::runtime_error("Cannot read PLY file: " + ply_file);
            }
        }
        pcl::toROSMsg(*cloud, output);
        output.header.frame_id = "world";
        output.header.stamp = now();
        pcl_pub->publish(output);

        std::cout<<"Pointloud size:"<<cloud->points.size()<<std::endl;
        printf("2.Mapping... \n");
        grid_map = new GridMap(cloud, resolution, occupy_threshold);
        
        next_depth_pub_time = now();
        next_lidar_pub_time = now();

        // ROS
        image_pub_ = create_publisher<sensor_msgs::msg::Image>(depth_topic, rclcpp::SensorDataQoS());
        point_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(lidar_topic, rclcpp::SensorDataQoS());
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, rclcpp::SensorDataQoS(),
            std::bind(&SensorSimulator::odomCallback, this, std::placeholders::_1));

        printf("3.Simulation Ready! \n");
    }

    ~SensorSimulator() override {
        if (grid_map) {
            grid_map->freeGridMap();
            delete grid_map;
        }
        delete camera;
        delete lidar;
    }

    void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

    void renderDepthCallback(const builtin_interfaces::msg::Time& stamp);

    void renderLidarCallback(const builtin_interfaces::msg::Time& stamp);

private:
    bool render_depth{false};
    bool render_lidar{false};
    Eigen::Quaternionf quat;
    Eigen::Quaternionf quat_bc, quat_wc;
    Eigen::Vector3f pos;

    CameraParams* camera;
    LidarParams* lidar;
    GridMap* grid_map{nullptr};
    sensor_msgs::msg::PointCloud2 output;
    
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_, pcl_pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::string body_frame_{"quadrotor"};

    rclcpp::Time next_depth_pub_time{0, 0, RCL_ROS_TIME}, next_lidar_pub_time{0, 0, RCL_ROS_TIME};
    rclcpp::Duration depth_pub_duration{0, 0}, lidar_pub_duration{0, 0};
    double depth_time{0.0}, lidar_time{0.0};
    int depth_count{0}, lidar_count{0};
    // mocka::Maps map;
};



void SensorSimulator::renderDepthCallback(const builtin_interfaces::msg::Time& stamp) {
    if (!render_depth)
        return;

    auto start = std::chrono::high_resolution_clock::now();

    cudaMat::SE3<float> T_wc(quat_wc.w(), quat_wc.x(), quat_wc.y(), quat_wc.z(), pos.x(), pos.y(), pos.z());
    cv::Mat depth_image;
    renderDepthImage(grid_map, camera, T_wc, depth_image);
    
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    depth_time += elapsed.count();
    depth_count++;
    // std::cout << "生成图像耗时: " << elapsed.count() << " 秒" << std::endl;

    sensor_msgs::msg::Image ros_image;
    cv_bridge::CvImage cv_image;
    cv_image.header.stamp = stamp;
    cv_image.header.frame_id = "camera_link";
    cv_image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    cv_image.image = depth_image;
    cv_image.toImageMsg(ros_image);
    image_pub_->publish(ros_image);
}

void SensorSimulator::renderLidarCallback(const builtin_interfaces::msg::Time& stamp) {
    if (!render_lidar)
        return;

    auto start = std::chrono::high_resolution_clock::now();

    cudaMat::SE3<float> T_wc(quat.w(), quat.x(), quat.y(), quat.z(), pos.x(), pos.y(), pos.z());
    pcl::PointCloud<pcl::PointXYZ> lidar_points;
    renderLidarPointcloud(grid_map, lidar, T_wc, lidar_points);
    
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    lidar_time += elapsed.count();
    lidar_count++;
    // std::cout << "生成雷达耗时: " << elapsed.count() << " 秒" << std::endl;

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(lidar_points, output);
    output.header.stamp = stamp;
    output.header.frame_id = body_frame_;
    point_cloud_pub_->publish(output);
}

void SensorSimulator::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    if (!msg->child_frame_id.empty()) {
        body_frame_ = msg->child_frame_id;
    }
    quat.x() = msg->pose.pose.orientation.x;
    quat.y() = msg->pose.pose.orientation.y;
    quat.z() = msg->pose.pose.orientation.z;
    quat.w() = msg->pose.pose.orientation.w;
    quat_wc = quat * quat_bc;

    pos.x() = msg->pose.pose.position.x;
    pos.y() = msg->pose.pose.position.y;
    pos.z() = msg->pose.pose.position.z;

    const rclcpp::Time tnow = now();

    // 避免仿真odom消息中断，导致时间差太大
    if (fabs((tnow - next_depth_pub_time).seconds()) > 10 * depth_pub_duration.seconds())
        next_depth_pub_time = tnow;
    if (fabs((tnow - next_lidar_pub_time).seconds()) > 10 * lidar_pub_duration.seconds())
        next_lidar_pub_time = tnow;

    if (tnow >= next_depth_pub_time){
        next_depth_pub_time += depth_pub_duration;
        renderDepthCallback(msg->header.stamp);
    }
    if (tnow >= next_lidar_pub_time){
        next_lidar_pub_time += lidar_pub_duration;
        renderLidarCallback(msg->header.stamp);
    }
    const rclcpp::Duration render_duration = now() - tnow;
    if (render_duration > depth_pub_duration || render_duration > lidar_pub_duration){
        // Performance reference: should take < 1 ms on 3060 GPU & Ubuntu 20.04
        RCLCPP_WARN(get_logger(), "Current Rendering time: %.2f ms, delay too much!", 1000 * render_duration.seconds());
        std::cout << "Average Depth Rendering time: " << (depth_time / (depth_count + 1e-8)) * 1000 << " ms" << std::endl;
        std::cout << "Average Lidar Rendering time: " << (lidar_time / (lidar_count + 1e-8)) * 1000 << " ms" << std::endl;
    }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<SensorSimulator>());
    } catch (const std::exception& error) {
        RCLCPP_FATAL(rclcpp::get_logger("sensor_simulator"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
