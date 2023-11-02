#include "mapbag_editor_server/mapbag_editor_server.h"

#include <grid_map_msgs/GridMap.h>
#include <hector_std_msgs/StringService.h>
#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>


using Scalar = float;

using namespace hector_std_msgs;
using namespace mapbag_editor_server;
using namespace visualization_msgs;
using namespace visualization_msgs;
using namespace interactive_markers;

void setColor( visualization_msgs::Marker &marker, uint8_t r, uint8_t g, uint8_t b,
                      uint8_t a = 255 )
{
  marker.color.r = r / 255.f;
  marker.color.g = g / 255.f;
  marker.color.b = b / 255.f;
  marker.color.a = a / 255.f;
}

void setScale( visualization_msgs::Marker &marker, double x, double y, double z )
{
  marker.scale.x = x;
  marker.scale.y = y;
  marker.scale.z = z;
}

ros::Publisher pub_marker_pose;
visualization_msgs::InteractiveMarker interactive_marker;
std::shared_ptr<interactive_markers::InteractiveMarkerServer> marker_server;
geometry_msgs::Point msg_pre;

void submapPosCallback( const geometry_msgs::Pose::ConstPtr& msg )
{
  interactive_marker.pose = *msg;
  interactive_marker.pose.position.z += 0.2;
  interactive_marker.pose.orientation.w = 1;
  marker_server->insert( interactive_marker );
  marker_server->applyChanges();
}

bool msgchanged( const geometry_msgs::Point& msg ) {
  if( msg_pre.x == 0 && msg_pre.y == 0 && msg_pre.z == 0 ) {
    msg_pre = msg;
    return true;
  } else if( abs( msg_pre.x - msg.x ) >= 0.001 || abs( msg_pre.y - msg.y ) >= 0.001 || abs( msg_pre.z - msg.z ) >= 0.001 ){
    return true;
  } else return false;
}

void processFeedback( const InteractiveMarkerFeedbackConstPtr &feedback ) {  
  geometry_msgs::Point msg = feedback->pose.position;
  if( msgchanged( msg ) ) { 
    pub_marker_pose.publish( msg );
    msg_pre = msg;
  }
}

int main( int argc, char **argv )
{
  ros::init( argc, argv, "mapbag_editor_server_node" );
  ros::NodeHandle nh;
  ros::NodeHandle pnh( "~" );

  std::string elevation_map_layer = pnh.param<std::string>( "grid_map_layer", "" );

  MapbagEditorServer<Scalar> server( nh, pnh ); 

  ros::Subscriber submap_pos_sub = nh.subscribe( "submap_posi", 1, submapPosCallback );
  pub_marker_pose = nh.advertise<geometry_msgs::Point>( "submap_new_center", 1, true );
  marker_server = std::make_shared<interactive_markers::InteractiveMarkerServer>( "submap_interactive_marker_server" );
  interactive_marker.name = "submap_marker";
  interactive_marker.header.frame_id = "world";
  interactive_marker.scale = 1;
  interactive_marker.pose.position.z = 0;
  interactive_marker.pose.orientation.w = 1;
  {
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE;
    setColor( marker, 20, 200, 20, 128 );
    setScale( marker, .05, .05, .05 );
    visualization_msgs::InteractiveMarkerControl control;
    control.always_visible = false;
    control.markers.push_back( marker );
    interactive_marker.controls.push_back( control );
  }
  {
    visualization_msgs::InteractiveMarkerControl control;
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    control.orientation.x = 1;
    control.orientation.w = 1;
    interactive_marker.controls.push_back( control );
  }
  {
    visualization_msgs::InteractiveMarkerControl control;
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    control.orientation.y = 1;
    control.orientation.w = 1;
    interactive_marker.controls.push_back( control );
  }
  {
    visualization_msgs::InteractiveMarkerControl control;
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    control.orientation.z = 1;
    control.orientation.w = 1;
    interactive_marker.controls.push_back( control );
  }

  Pose<Scalar> marker_pose;
  marker_server->insert( interactive_marker, &processFeedback );
  marker_server->applyChanges();

  ros::spin();

  return 0;
}

