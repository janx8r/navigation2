#ifndef NDT_DISPLAY_H
#define NDT_DISPLAY_H

#include <boost/circular_buffer.hpp>

#include <nav2_msgs/msg/ndt_map_msg.hpp>

#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>

namespace Ogre
{
  class SceneNode;
}

namespace rviz
{
  class ColorProperty;
  class FloatProperty;
  class IntProperty;
}

namespace nav2_rviz_plugins{

  class NDTVisual;

  class NDTDisplay: public rviz_common::MessageFilterDisplay<nav2_msgs::msg::NDTMapMsg>{
    Q_OBJECT
    public:

    NDTDisplay();
    virtual ~NDTDisplay();

  protected:
    virtual void onInitialize();

    virtual void reset();

  private Q_SLOTS:
    void updateColorAndAlpha();
    void updateHistoryLength();

  private:
	  
	  /**
	   * @brief transform a NDTmap msg into a vector of NDTVisual, one per cell. If history is set to one or lower, it only display the last NDTMap. Otherwise, it keeps all cells through time
	   */
    void processMessage(const nav2_msgs::msg::NDTMapMsg::ConstSharedPtr msg);

    std::vector<boost::shared_ptr<NDTVisual> > visuals_;

    rviz_common::properties::ColorProperty* color_property_;
    rviz_common::properties::FloatProperty* alpha_property_;
    rviz_common::properties::IntProperty* history_length_property_;
  };
}

#endif 

