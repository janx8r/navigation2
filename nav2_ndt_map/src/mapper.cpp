#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.h>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/pose.h>

#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <laser_geometry/laser_geometry.hpp>
#include <std_srvs/srv/empty.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/ndt.h>
#include <pcl_ros/point_cloud.hpp>
#include <pcl_ros/transforms.hpp>
#include <pcl/io/pcd_io.h>

#include <nav2_ndt_map/ros_param.hpp>

#include "nav2_ndt_map/ndt_map_2d.hpp"
#include <random>
#include <math.h>


using namespace std::chrono_literals;


class Mapper : public rclcpp::Node
{

protected:
  // Params
  std::shared_ptr<RosParam<bool>> publish_ndt, publish_pcl, downsample_pcl, filter_ndt, filter_pcl_param, record_map_point_now;
  std::shared_ptr<RosParam<std::string>> pcl_topic, laser_topic;
  std::shared_ptr<RosParam<std::string>> pcl_map_pub_topic, ndt_map_pub_topic;
  std::shared_ptr<RosParam<std::string>> robot_frame, odom_frame, map_frame;
  std::shared_ptr<RosParam<std::string>> output_pcd_file, output_ndt_file, input_ndt_file;
  std::shared_ptr<RosParam<double>> downsample_size_pcl;
  std::shared_ptr<RosParam<double>> x_min, x_max, y_min, y_max, delta;
  std::shared_ptr<RosParam<int>> ndt_auto_update_period, pcl_auto_update_period, jump_around_period;

  // Publisher and subscriber
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_map_pub;
  rclcpp::Publisher<nav2_msgs::msg::NDTMapMsg>::SharedPtr ndt_map_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr jump_around_pub;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  

  // Services
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_ndt_map;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_pcl_map;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr update_ndt_map_srv;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr update_pcl_srv;

  // Timer
  rclcpp::TimerBase::SharedPtr auto_update_timer, jump_around_timer, reactivate_timer;
  // rclcpp::TimerBase::SharedPtr pcl_auto_update_timer;

  // Transforms
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf2_buffer;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster;

  // Laserscan to Pointcloud projector
  laser_geometry::LaserProjection projector;

  NdtMap* ndt_map;
  NdtMap* ndt_map2;
  pcl::PointCloud<pcl::PointXYZ> map_pcl;
  pcl::PointCloud<pcl::PointXYZ> sensor_data_pcl_buffer;

  std::mutex ndt_map_mtx_, message_m;
  bool ignore_pcls, joystick_button;


public:
  Mapper() : Node("mapper")
  {

    // Reconfigurable Parameters
    pcl_map_pub_topic = std::make_shared<RosParam<std::string>>(this, "pcl_map_pub_topic", "pcl_map", "Topic, auf dem die pcl-Karte gepublisht wird.", [this]()
                        { init_publishers(); });
    ndt_map_pub_topic = std::make_shared<RosParam<std::string>>(this, "ndt_map_pub_topic", "ndt_map", "Topic, auf dem die ndt-Karte gepublisht wird.", [this]()
                        { init_publishers(); });
    pcl_topic = std::make_shared<RosParam<std::string>>(this, "pcl_topic", "pcl_scan", "Topic, von dem die Sensor-Pointcloud subscribt wird.", [this]()
                        { init_subscribers(); });
    laser_topic = std::make_shared<RosParam<std::string>>(this, "laser_topic", "laser_scan", "Topic, von dem der Laserscan subscribt wird.", [this]()
                        { init_subscribers(); });
    robot_frame = std::make_shared<RosParam<std::string>>(this, "robot_frame", "base_link", "Framename der Roboterbasis", [this]()
                        { init_transforms(); });
    odom_frame = std::make_shared<RosParam<std::string>>(this, "odom_frame", "odom", "Framename der Odometrie", [this]()
                        { init_transforms(); });
    map_frame = std::make_shared<RosParam<std::string>>(this, "map_frame", "map", "Framename der Map", [this]()
                        { init_transforms(); });

    record_map_point_now = std::make_shared<RosParam<bool>>(this, "record_map_point_now", false, "Flag, um Mappunkte zu sammeln.");

    output_pcd_file = std::make_shared<RosParam<std::string>>(this, "output_pcd_file", "map.pcd", "Dateiname unter der die pcd-map gespeichert werden soll.");
    output_ndt_file = std::make_shared<RosParam<std::string>>(this, "output_ndt_file", "map.ndt", "Dateiname unter der die ndt-map gespeichert werden soll.");
    input_ndt_file = std::make_shared<RosParam<std::string>>(this, "input_ndt_file", "map_komplett_30.ndt", "Lade diese Map am Anfang und erweitere sie um Messungen");

    publish_ndt = std::make_shared<RosParam<bool>>(this, "publish_ndt", false, "Soll die ndt-Map gepublisht werden?");
    publish_pcl = std::make_shared<RosParam<bool>>(this, "publish_pcl", false, "Soll die Pointcloud-Map gepublisht werden?");
    downsample_pcl = std::make_shared<RosParam<bool>>(this, "downsample_pcl", true, "Soll die Pointcloud-Map vor dem publishen gefiltert werden?"); //unbenutzt
    downsample_size_pcl = std::make_shared<RosParam<double>>(this, "downsample_size_pcl", 0.01, "Voxelgröße zum downsamplen der pcl");

    filter_ndt = std::make_shared<RosParam<bool>>(this, "filter_ndt", true, "Soll die PCL gefiltert werden, bevor damit die NDT-Map geupdatet wird?");
    filter_pcl_param = std::make_shared<RosParam<bool>>(this, "filter_pcl", true, "Soll die PCL gefiltert werden, bevor damit die PCL geupdatet wird?");
    ndt_auto_update_period = std::make_shared<RosParam<int>>(this, "ndt_auto_update_period", 10000, "Periode des regelmäßigen Updates der NDT-Karte in ms. 0 zum deaktivieren.", [this]()
                        { init_timers(); });
    // pcl_auto_update_period = std::make_shared<RosParam<int>>(this, "pcl_auto_update_period", 10000, "Periode des regelmäßigen Updates der PCL in ms. 0 zum deaktivieren.", [this]()
    //                     { init_timers(); });

    x_min = std::make_shared<RosParam<double>>(this, "x_min", -15.0, "Initial map size (in metres)");
    x_max = std::make_shared<RosParam<double>>(this, "x_max", 15.0, "Initial map size (in metres)");
    y_min = std::make_shared<RosParam<double>>(this, "y_min", -15.0, "Initial map size (in metres)");
    y_max = std::make_shared<RosParam<double>>(this, "y_max", 15.0, "Initial map size (in metres)");
    delta = std::make_shared<RosParam<double>>(this, "delta", 0.15, "Initial size of each cell (in metres)");


    ignore_pcls = false;
    jump_around_period = std::make_shared<RosParam<int>>(this, "jump_around_period", 0, "Periode der automatischen herumspringen des Roboters zum Mappen der Karte in ms. 0 zum deaktivieren.", [this]()
                        { init_timers(); });


    


    // Transforms
    tf2_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf2_listener = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer);
    tf2_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    

    init_publishers();
    init_transforms();

    // lade eine Karte von der Festplatte oder erstelle eine neue leere Karte
    ndt_map = new NdtMap(map_frame->get_val(), this->get_node_clock_interface());
    if(ndt_map->load_from_file(input_ndt_file->get_val())){
      update_ndt_map();
    }
    else{
      ndt_map->init(x_min->get_val(), x_max->get_val(), y_min->get_val(), y_max->get_val(), delta->get_val());
    }

    init_subscribers();
    init_timers();




    // Services
    save_ndt_map = this->create_service<std_srvs::srv::Empty>("save_ndt_map", [this](std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res)
                      { save_ndt_map_callback(req, res); });
    save_pcl_map = this->create_service<std_srvs::srv::Empty>("save_pcl_map", [this](std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res)
                      { save_pcl_map_callback(req, res); });

    update_ndt_map_srv = this->create_service<std_srvs::srv::Empty>("update_ndt_map", [this](std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res)
                      { update_ndt_map_callback(req, res); });

    update_pcl_srv = this->create_service<std_srvs::srv::Empty>("update_pcl", [this](std::shared_ptr<std_srvs::srv::Empty::Request> req, std::shared_ptr<std_srvs::srv::Empty::Response> res)
                      { update_pcl_callback(req, res); });
  }


  //'map' - 'ROBOT_0' - 'hokuyo_gt'
  //'map' - 'odom' - 'base_link' - 'hokuyo'

  ~Mapper()
  {
  }
  
  void init_publishers()
  {
    pcl_map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(pcl_map_pub_topic->get_val(), 10);
    ndt_map_pub = this->create_publisher<nav2_msgs::msg::NDTMapMsg>(ndt_map_pub_topic->get_val(), rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    jump_around_pub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
  }

  void init_subscribers()
  {
    laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(laser_topic->get_val(), 10, [this](sensor_msgs::msg::LaserScan msg)
                      { laser_callback(msg); });
    pcl_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(pcl_topic->get_val(), 10, [this](sensor_msgs::msg::PointCloud2 msg)
                      { pcl_callback(msg); });
    joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, [this](sensor_msgs::msg::Joy msg)
                      { joy_callback(msg); });
  }

  void init_timers()
  {
    if(ndt_auto_update_period->get_val() > 0)
      auto_update_timer = this->create_wall_timer(std::chrono::milliseconds(ndt_auto_update_period->get_val()), [this](){update();});
    else
      if(auto_update_timer){
        auto_update_timer->cancel();
      }

    if(jump_around_period->get_val() > 0)
      jump_around_timer = this->create_wall_timer(std::chrono::milliseconds(jump_around_period->get_val()), [this](){jump_around();});
    else
      if(jump_around_timer){
        jump_around_timer->cancel();
      }



    // if(pcl_auto_update_period->get_val() > 0)
    //   pcl_auto_update_timer = this->create_wall_timer(std::chrono::milliseconds(pcl_auto_update_period->get_val()), [this](){update_pcl();});
    // else
    //   if(pcl_auto_update_timer){
    //     pcl_auto_update_timer->cancel();
    //   }
  }

  void init_transforms(){

    //tf2_buffer->waitForTransform("/world", tf_pose_frame_, time,ros::Duration(1.0));

    // statische TF abrufen und speichern, z.B Laser zu Robot

  }

  void laser_callback(const sensor_msgs::msg::LaserScan &laser_msg)
  {
    // Laserscan to pcl
    sensor_msgs::msg::PointCloud2 cloud_msg;
    message_m.lock();
    projector.projectLaser(laser_msg, cloud_msg);
    message_m.unlock();
    pcl_callback(cloud_msg);
  }

  void pcl_callback(const sensor_msgs::msg::PointCloud2 &input)
  {
    if(!ignore_pcls && (record_map_point_now->get_val() || joystick_button))
      build_map_gt(input);
  }

  void joy_callback(const sensor_msgs::msg::Joy &input)
  {
    bool btn = input.buttons[0];
    if(joystick_button != btn){
      RCLCPP_INFO_STREAM(this->get_logger(), "Joystick Button: " << btn);
      joystick_button = (bool)(input.buttons[0]);
    }
    
    
  }

  void build_map_gt(const sensor_msgs::msg::PointCloud2 &input)
  {
    // ROS msg in pcl PonitCloud konvertieren
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> sensor_points_sensorTF_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(input, *sensor_points_sensorTF_ptr);

    // Transformation vom sensor_frame ins map_frame abfragen
    const std::string sensor_frame = input.header.frame_id;
    geometry_msgs::msg::TransformStamped TF_sensor_to_map;
    // std::cout << "map_frame: " << map_frame << "\t sensor_frame" << sensor_frame << std::endl;
    bool got_sensor_to_map_tf = get_transform(map_frame->get_val(), sensor_frame, &TF_sensor_to_map, input.header.stamp);


    if (got_sensor_to_map_tf)
    {
      // convert TF-message to transform-matrix
      const Eigen::Affine3d sensor_to_map_affine = tf2::transformToEigen(TF_sensor_to_map);
      const Eigen::Matrix4f sensor_to_map_matrix = sensor_to_map_affine.matrix().cast<float>();

      // transform sensordata to map_frame
      boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> sensor_points_mapTF_ptr(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::transformPointCloud(*sensor_points_sensorTF_ptr, *sensor_points_mapTF_ptr, sensor_to_map_matrix);

      // add sensordata to pcl-map
      //map_pcl += *sensor_points_mapTF_ptr;
      sensor_data_pcl_buffer += *sensor_points_mapTF_ptr;

    }
  }


  void save_pcl_map_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                         std::shared_ptr<std_srvs::srv::Empty::Response> res)
  {
    (void) req; //to supress unused warning
    (void) res;
    filter_pcl();
    RCLCPP_INFO(this->get_logger(), "Saving current pcl-map to map directory %s", output_pcd_file->get_val().c_str());
    pcl::io::savePCDFileASCII(output_pcd_file->get_val(), map_pcl);
  }

  void save_ndt_map_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                         std::shared_ptr<std_srvs::srv::Empty::Response> res)
  {
    (void) req; //to supress unused warning
    (void) res;
    ndt_map->save_to_file(output_ndt_file->get_val());

    delete ndt_map;
    ndt_map = new NdtMap(map_frame->get_val(), this->get_node_clock_interface());

    ndt_map->load_from_file(output_ndt_file->get_val());
  }


  double random_number(double min, double max)
  {
    // use thread_local to make this function thread safe
    thread_local static std::mt19937 mt{std::random_device{}()};
    thread_local static std::uniform_real_distribution<double> dist;
    using pick = std::uniform_real_distribution<double>::param_type;

    return dist(mt, pick(min, max));
  }


  void reactivate(){
    ignore_pcls = false;
    reactivate_timer->cancel();
  }

  void jump_around(){

    ignore_pcls = true;
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.frame_id = "map";
    pose.pose.pose.position.x = random_number(x_min->get_val(), x_max->get_val());
    pose.pose.pose.position.y = random_number(y_min->get_val(), y_max->get_val());
    pose.pose.pose.position.z = 0.0909;
    double yaw = random_number(-M_PI/2, M_PI/2);
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    tf2::convert(q, pose.pose.pose.orientation);

    RCLCPP_INFO_STREAM(this->get_logger(), "Jump to pose " << pose.pose.pose.position.x << ", " << pose.pose.pose.position.y << ", " << yaw);
    jump_around_pub->publish(pose);

    reactivate_timer = this->create_wall_timer(std::chrono::milliseconds(500), [this](){reactivate();});
  }


  void update(){
    update_ndt_map();
    //update_pcl();
    sensor_data_pcl_buffer.clear();
  }

  void update_ndt_map(){
    if(filter_ndt->get_val()){
      RCLCPP_INFO(this->get_logger(), "Filtering PCL before updating NDT map...");
      filter_pcl();
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Start updating NDT map with " << sensor_data_pcl_buffer.size() << " points...");
    ndt_map->update(sensor_data_pcl_buffer);
    
    RCLCPP_INFO(this->get_logger(), "NDT map update done!");

    if(publish_ndt->get_val()){
      RCLCPP_INFO(this->get_logger(), "Publishing NDT map now.");
      nav2_msgs::msg::NDTMapMsg map_ndt_msg;
      ndt_map->to_message(&map_ndt_msg);
      ndt_map_pub->publish(map_ndt_msg);
    }
  }

  void update_pcl(){
    // add sensordata to pcl-map
    map_pcl += sensor_data_pcl_buffer;

    if(filter_pcl_param->get_val()){
      RCLCPP_INFO(this->get_logger(), "Filtering PCL...");
      filter_pcl();
    }

    if(publish_pcl->get_val()){
      RCLCPP_INFO(this->get_logger(), "Publishing PCL now.");
      sensor_msgs::msg::PointCloud2 map_pcl_msg;
      pcl::toROSMsg(map_pcl, map_pcl_msg);
      map_pcl_msg.header.stamp = this->get_clock()->now();
      map_pcl_msg.header.frame_id = map_frame->get_val();
      pcl_map_pub->publish(map_pcl_msg);
    }
  }


  void update_ndt_map_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                         std::shared_ptr<std_srvs::srv::Empty::Response> res)
  {
    (void) req; //to supress unused warning
    (void) res;
    this->update_ndt_map();
  }

  void update_pcl_callback(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                         std::shared_ptr<std_srvs::srv::Empty::Response> res)
  {
    (void) req; //to supress unused warning
    (void) res;
    this->update_pcl();
  }


  void filter_downsample_pcl(double leaf_size)
  {
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(map_pcl.makeShared());
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    sor.filter(map_pcl);
  }

  void filter_statistical_pcl()
  {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_stat;
    sor_stat.setInputCloud(map_pcl.makeShared());
    sor_stat.setMeanK(50);
    sor_stat.setStddevMulThresh(1.0);
    sor_stat.filter(map_pcl);
  }

  void filter_pcl()
  {
    std::cerr << "PointCloud before filtering: " << map_pcl.width * map_pcl.height
              << " data points (" << pcl::getFieldsList(map_pcl) << ")." << std::endl;
    filter_downsample_pcl(downsample_size_pcl->get_val());
    //filter_statistical_pcl();
    std::cerr << "PointCloud after filtering: " << map_pcl.width * map_pcl.height
              << " data points (" << pcl::getFieldsList(map_pcl) << ")." << std::endl;
  }


  bool
  get_transform(
      const std::string &target_frame, const std::string &source_frame,
      geometry_msgs::msg::TransformStamped *transform_stamped_ptr, rclcpp::Time time_stamp)
  {
    if (target_frame == source_frame)
    {
      transform_stamped_ptr->header.stamp = time_stamp;
      transform_stamped_ptr->header.frame_id = target_frame;
      transform_stamped_ptr->child_frame_id = source_frame;
      transform_stamped_ptr->transform.translation.x = 0.0;
      transform_stamped_ptr->transform.translation.y = 0.0;
      transform_stamped_ptr->transform.translation.z = 0.0;
      transform_stamped_ptr->transform.rotation.x = 0.0;
      transform_stamped_ptr->transform.rotation.y = 0.0;
      transform_stamped_ptr->transform.rotation.z = 0.0;
      transform_stamped_ptr->transform.rotation.w = 1.0;
      return true;
    }

    try
    {
      *transform_stamped_ptr = tf2_buffer->lookupTransform(
          target_frame, source_frame, time_stamp, 2000ms);
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      RCLCPP_WARN(this->get_logger(), "Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

      transform_stamped_ptr->header.stamp = time_stamp;
      transform_stamped_ptr->header.frame_id = target_frame;
      transform_stamped_ptr->child_frame_id = source_frame;
      transform_stamped_ptr->transform.translation.x = 0.0;
      transform_stamped_ptr->transform.translation.y = 0.0;
      transform_stamped_ptr->transform.translation.z = 0.0;
      transform_stamped_ptr->transform.rotation.x = 0.0;
      transform_stamped_ptr->transform.rotation.y = 0.0;
      transform_stamped_ptr->transform.rotation.z = 0.0;
      transform_stamped_ptr->transform.rotation.w = 1.0;
      return false;
    }
    return true;
  }

  bool get_transform(
      const std::string &target_frame, const std::string &source_frame,
      geometry_msgs::msg::TransformStamped *transform_stamped_ptr)
  {
    return get_transform(target_frame, source_frame, transform_stamped_ptr, tf2_ros::toRclcpp(tf2::TimePointZero));
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mapper>());
  rclcpp::shutdown();
  return 0;
}
