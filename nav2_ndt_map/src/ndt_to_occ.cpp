#include "rclcpp/rclcpp.hpp"
#include <nav2_msgs/msg/ndt_map_msg.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_msgs/msg/voxel_grid.hpp>

using std::placeholders::_1;

class NdtMapToOccMap : public rclcpp::Node
{
public:
  NdtMapToOccMap()
      : Node("ndt_to_occ")
  {
    publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    voxel_publisher_ = this->create_publisher<nav2_msgs::msg::VoxelGrid>("/voxel_map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    subscription_ = this->create_subscription<nav2_msgs::msg::NDTMapMsg>(
        "/ndt_map", 10, std::bind(&NdtMapToOccMap::callback, this, _1));
  }

private:

  void ndt_to_voxel(const nav2_msgs::msg::NDTMapMsg::SharedPtr msg) const{
    nav2_msgs::msg::VoxelGrid voxel_msg;

    voxel_msg.header = msg->header;
    voxel_msg.origin.x = msg->x_cen;
    voxel_msg.origin.y = msg->y_cen;
    voxel_msg.origin.z = msg->z_cen;
    voxel_msg.resolutions.x = msg->x_cell_size;
    voxel_msg.resolutions.y = msg->y_cell_size;
    voxel_msg.resolutions.z = msg->z_cell_size;
    voxel_msg.size_x = msg->x_size / msg->x_cell_size;
    voxel_msg.size_y = msg->y_size / msg->y_cell_size;
    voxel_msg.size_z = msg->z_size / msg->z_cell_size;

    for (long unsigned int itr = 0; itr < msg->cells.size(); itr++)
    {
      voxel_msg.data.push_back(msg->cells[itr].occupancy * 255);
    }

    voxel_publisher_->publish(voxel_msg);

  }

  void ndt_to_occmap(const nav2_msgs::msg::NDTMapMsg::SharedPtr msg) const{
    nav_msgs::msg::OccupancyGrid occ_msg;

    occ_msg.header = msg->header;
    occ_msg.info.map_load_time = this->now();
    occ_msg.info.resolution = msg->x_cell_size;
    occ_msg.info.width = msg->x_size / msg->x_cell_size;
    occ_msg.info.height = msg->y_size / msg->y_cell_size;
    occ_msg.info.origin.position.x = msg->x_cen - msg->x_size/2;
    occ_msg.info.origin.position.y = msg->y_cen - msg->y_size/2;
    occ_msg.info.origin.position.z = msg->z_cen - msg->z_size/2;

    // std::cout << "x: " << msg->x_size / msg->x_cell_size << std::endl;
    // std::cout << "y: " << msg->y_size / msg->y_cell_size << std::endl;
    // std::cout << "z: " << msg->z_size / msg->z_cell_size << std::endl;
    // std::cout << "size: " << msg->cells.size() << std::endl;

    int x_size = msg->x_size / msg->x_cell_size;
    int y_size = msg->y_size / msg->y_cell_size;
    int z_size = msg->z_size / msg->z_cell_size;

    std::vector<signed char> map2d(x_size*y_size, 0);
    
    double x_off = msg->x_cen - msg->x_size / 2;
    double y_off = msg->y_cen - msg->y_size / 2;
    for (long unsigned int itr = 0; itr < msg->cells.size(); itr++)
    {
      // //occ_msg.data.push_back(msg->cells[itr].occupancy * 255);
      // int ind_x = (msg->cells[itr].center_x - x_off) / msg->x_cell_size;
      // int ind_y = (msg->cells[itr].center_y - y_off) / msg->y_cell_size;

      // if(ind_x < 0 || ind_x > x_size || ind_y < 0 || ind_y > y_size){
      //   RCLCPP_INFO_STREAM(this->get_logger(),"index error: ind_x = " << ind_x << ", ind_y = " << ind_y);
      // }
      // int ind = ind_x + ind_y*x_size; 
      // RCLCPP_INFO_STREAM(this->get_logger(),"ind = " << ind);

      if(msg->cells[itr].has_gaussian){
        // map2d[ind] = 127;
        occ_msg.data.push_back(100);
      }
        
      else{
        // map2d[ind] = 0;
        occ_msg.data.push_back(0);
      }
        
     
    }
    //occ_msg.data = map2d;

    //occ_msg.data[0] = 127;

    publisher_->publish(occ_msg);

  }
  void callback(const nav2_msgs::msg::NDTMapMsg::SharedPtr msg) const
  {
    ndt_to_voxel(msg);
    ndt_to_occmap(msg);
  }

  rclcpp::Subscription<nav2_msgs::msg::NDTMapMsg>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp::Publisher<nav2_msgs::msg::VoxelGrid>::SharedPtr voxel_publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NdtMapToOccMap>());
  rclcpp::shutdown();
  return 0;
}
