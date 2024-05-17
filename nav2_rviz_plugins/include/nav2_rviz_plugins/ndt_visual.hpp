#ifndef NDT_VISUAL_H
#define NDT_VISUAL_H
#include <boost/shared_ptr.hpp>

// #include <OgreVector3.h>
// #include <OgreSceneNode.h>
// #include <OgreSceneManager.h>

#include <nav2_msgs/msg/ndt_map_msg.hpp>
#include <nav2_msgs/msg/ndt_cell_msg.hpp>

#include <rviz_rendering/objects/shape.hpp>

namespace Ogre{
  //class Vector3;
  class Quaternion;
}

namespace rviz{
  class Shape;
}
namespace nav2_rviz_plugins{

  class NDTVisual{
  public:
    NDTVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );
    virtual ~NDTVisual();
    void setCell(nav2_msgs::msg::NDTCellMsg msg);
    void setCell2D(nav2_msgs::msg::NDTCellMsg msg);
    void setFramePosition(const Ogre::Vector3& position);
    void setFrameOrientation(const Ogre::Quaternion& orientation);
    void setColor( float r, float g, float b, float a );
  private:
    boost::shared_ptr<rviz_rendering::Shape> NDT_elipsoid_;
    Ogre::SceneNode* frame_node_;
    Ogre::SceneManager* scene_manager_;
  };
}
#endif 
