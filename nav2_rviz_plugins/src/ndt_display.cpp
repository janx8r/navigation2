#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <tf2_ros/transform_listener.h>

#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/frame_manager_iface.hpp>

#include "nav2_rviz_plugins/ndt_visual.hpp"
#include "nav2_rviz_plugins/ndt_display.hpp"



namespace nav2_rviz_plugins{
  
  NDTDisplay::NDTDisplay(){
    //ROS_ERROR("BUILDING OBJECT");
    std::cout << "\033[1;31BUILDING OBJECT\033[0m\n";
    color_property_ = new rviz_common::properties::ColorProperty( "Color", QColor( 204, 51, 204 ),
                                               "Color to draw the acceleration arrows.",
                                               this, SLOT( updateColorAndAlpha() ));

    alpha_property_ = new rviz_common::properties::FloatProperty( "Alpha", 1.0,
                                               "0 is fully transparent, 1.0 is fully opaque.",
                                               this, SLOT( updateColorAndAlpha() ));

    history_length_property_ = new rviz_common::properties::IntProperty( "History Length", 1,
                                                      "Number of prior measurements to display.",
                                                      this, SLOT( updateHistoryLength() ));
    history_length_property_->setMin( 1 );
    history_length_property_->setMax( 100000 );
  }
  void NDTDisplay::onInitialize(){
    MFDClass::onInitialize();
  }

  NDTDisplay::~NDTDisplay(){
  }
  void NDTDisplay::reset(){
    MFDClass::reset();
    visuals_.clear();
  }
  void NDTDisplay::updateColorAndAlpha(){
    float alpha=alpha_property_->getFloat();
    Ogre::ColourValue color=color_property_->getOgreColor();
    for(size_t i=0;i<visuals_.size();i++){
      visuals_[i]->setColor(color.r,color.g,color.b,alpha);
    }
  }
  
  void NDTDisplay::updateHistoryLength()
{
	//ROS_INFO_STREAM("history received: " << this->history_length_property_->getInt());
  std::cout << "history received: " << this->history_length_property_->getInt() << std::endl;
  
}

  // void NDTDisplay::processMessage( const nav2_msgs::msg::NDTMapMsg::ConstPtr& msg ){
void NDTDisplay::processMessage(const nav2_msgs::msg::NDTMapMsg::ConstSharedPtr msg){
    //ROS_ERROR("MESSAGE RECIVED");
    //std::cout << "MESSAGE RECIVED" << std::endl;
    Ogre::Quaternion orientation;
    Ogre::Vector3 position;
	
    if(history_length_property_->getInt() <= 1){
      visuals_.clear();
    }
	
    if( !context_->getFrameManager()->getTransform( msg->header.frame_id,msg->header.stamp,position, orientation)){
      std::cout << "Error transforming from frame " << msg->header.frame_id.c_str() << " to frame " << qPrintable( fixed_frame_ ) << std::endl;
      return;
    }
    for(long unsigned int itr=0;itr<msg->cells.size();itr++){
      if(msg->cells[itr].has_gaussian == true){
        boost::shared_ptr<NDTVisual> visual;
        visual.reset(new NDTVisual(context_->getSceneManager(), scene_node_));

        bool two_d = msg->z_cell_size == 0.0 && msg->x_cell_size > 0.0  && msg->x_cell_size == msg->y_cell_size;
        bool three_d = msg->x_cell_size == msg->y_cell_size && msg->y_cell_size == msg->z_cell_size && msg->x_cell_size > 0.0;


        if(!two_d && !three_d){ 
          std::cout << "ERROR: SOMETHING IS WRONG WITH THE VOXEL:" << std::endl;
          std::cout << "msg->x_cell_size = " << msg->x_cell_size << std::endl;
          std::cout << "msg->y_cell_size = " << msg->y_cell_size << std::endl;
          std::cout << "msg->z_cell_size = " << msg->z_cell_size << std::endl;
          return;
        }

        if(two_d){
          visual->setCell2D(msg->cells[itr]);
        }
        else{
          visual->setCell(msg->cells[itr]);
        }
        
        visual->setFramePosition(position);
        visual->setFrameOrientation(orientation);
        float alpha = alpha_property_->getFloat();
        Ogre::ColourValue color=color_property_->getOgreColor();
        visual->setColor(color.r,color.g,color.b,alpha);
        visuals_.push_back(visual);
      }
    }
  }
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(nav2_rviz_plugins::NDTDisplay,rviz_common::Display)

