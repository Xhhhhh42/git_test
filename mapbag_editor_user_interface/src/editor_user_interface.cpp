#include "mapbag_editor_user_interface/editor_user_interface.h"

#include <ros/package.h>

namespace mapbag_editor_user_interface
{
QString EditorUserInterface::getPathToQml()
{
  std::string path = ros::package::getPath("mapbag_editor_user_interface");
  return QString::fromStdString( path ) + "/ui/main.qml";
}
} // namespace mapbag_editor_user_interface

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( mapbag_editor_user_interface::EditorUserInterface, rviz::Display )
