#include "mapbag_editor_user_interface/polygon_tool.h"
#include "mapbag_editor_user_interface/adaptive_medien_filter.h"

#include <iostream>
#include <cmath>

#include <rviz/properties/bool_property.h>
#include <rviz/mesh_loader.h>
#include <rviz/display_context.h>
#include <rviz/selection/selection_manager.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/geometry.h>

#include <hector_world_heightmap_ros/message_conversions/map.h>
#include <geometry_msgs/Pose.h>

#include <OgreSceneManager.h>
#include <OgreEntity.h>
#include <Eigen/Dense>
#include <ros/console.h>
#include <ros/package.h>

#include <QVector3D>


namespace mapbag_editor_user_interface
{

const char *POLYGONPOINT_PREVIEW_RESOURCE = "package://mapbag_editor_user_interface/media/red_flag.dae";
const char *POLYGONPOINT_MARKER_RESOURCE = "package://mapbag_editor_user_interface/media/arrow_marker.dae";
const float POLYGONPOINT_PREVIEW_SCALE = 0.1f;

using namespace std;
using namespace hector_math;
using namespace hector_world_heightmap;
using namespace hector_world_heightmap_ros::message_conversions;
using namespace hector_world_heightmap::integrators;

PolygonTool::PolygonTool()
{
  shortcut_key_ = 's';
  polygonpoint_3d_proporty_ = new rviz::BoolProperty( "3D Raycast", true,
                                          "If true, will use scene manager to find intersection with compatible displayed entities. "
                                          "If false, will only check for intersection with ground plane.",
                                          this->getPropertyContainer());
}


PolygonTool::~PolygonTool()
{
  clearPolygonpoints();
  if( polygonpoint_preview_node_ != nullptr ) destroyPolygonpointNode( polygonpoint_preview_node_ );
  if( polygonpoint_active_node_ != nullptr ) destroyPolygonpointNode( polygonpoint_active_node_ );
}


void PolygonTool::onInitialize() 
{
  rviz::InteractionTool::onInitialize();
  preview_mesh_ = rviz::loadMeshFromResource( POLYGONPOINT_PREVIEW_RESOURCE );
  marker_mesh_ = rviz::loadMeshFromResource( POLYGONPOINT_MARKER_RESOURCE );

  if ( preview_mesh_.isNull() || marker_mesh_.isNull() )
  {
    ROS_ERROR_NAMED( "mapbag_editor_user_interface", "Failed to load polygon tool preview/marker mesh." );
    return;
  }

  polygonpoint_preview_node_ = createPolygonpointNode();
  polygonpoint_preview_node_->setVisible( false );
  editor_mode_ = MODE_POLYGON;
  automatisch_mode_ = Aktivieren;

  nh_ = ros::NodeHandle();
  resolution_sub_ = nh_.subscribe<std_msgs::Float64>(
      "/mapbag_editor_server_node/mapbag_resolution", 1, [&]( const std_msgs::Float64::ConstPtr &resulotion ) { resolutionCallback( *resulotion ); });
  frame_sub_ = nh_.subscribe<std_msgs::String>(
      "/mapbag_editor_server_node/mapbag_frame", 1, [&]( const std_msgs::String::ConstPtr &frame ) { frameCallback( *frame ); });
  submap_sub_ = nh_.subscribe<grid_map_msgs::GridMap>(
      "/mapbag_editor_server_node/submap", 5, [&]( const grid_map_msgs::GridMap::ConstPtr &map ) { submapSubCallback( *map ); });
  submap_ref_sub_ = nh_.subscribe<grid_map_msgs::GridMap>(
      "/mapbag_editor_server_node/submap_ref", 5, [&]( const grid_map_msgs::GridMap::ConstPtr &map_ref ) { submapRefSubCallback( *map_ref ); });
  newcenter_sub_ = nh_.subscribe<geometry_msgs::Point>(
      "/submap_new_center", 5, [&]( const geometry_msgs::Point::ConstPtr &center ) { newCenterSubCallback( *center ); });
  mapToServer_pub_ = nh_.advertise<grid_map_msgs::GridMap>( "changedsubmap", 1, true );
  mapInTool_pub_ = nh_.advertise<grid_map_msgs::GridMap>( "submap_ref", 1, true );
  mapVerschieben_pub_ = nh_.advertise<grid_map_msgs::GridMap>( "verschiebsubmap", 1, true );
  pub_submap_pos_ = nh_.advertise<geometry_msgs::Pose>( "submap_posi", 1, true );

  skalenfaktor_ = 50;
  depth_ = 100;
}


void PolygonTool::activate() 
{
  if( polygonpoint_preview_node_ == nullptr ) return;
  if( editor_mode_ == MODE_POLYGON ) polygonpoint_preview_node_ ->setVisible( true, false );
}


void PolygonTool::deactivate() 
{
  if( polygonpoint_preview_node_ == nullptr ) return;
  polygonpoint_preview_node_ ->setVisible( false );
}


Ogre::SceneNode *PolygonTool::createPolygonpointNode()
{
  Ogre::SceneNode *node = scene_manager_->getRootSceneNode()->createChildSceneNode();
  Ogre::Entity *entity = scene_manager_->createEntity( preview_mesh_ );
  node->attachObject( entity );
  node->setScale( 10 * POLYGONPOINT_PREVIEW_SCALE, 10 * POLYGONPOINT_PREVIEW_SCALE, 10 * POLYGONPOINT_PREVIEW_SCALE );

  Ogre::SceneNode *marker_node = node->createChildSceneNode();
  entity = scene_manager_->createEntity( marker_mesh_ );
  marker_node->attachObject( entity );
  marker_node->setVisible( false );
  marker_node->setScale( 3 * POLYGONPOINT_PREVIEW_SCALE, 3 * POLYGONPOINT_PREVIEW_SCALE, 3 * POLYGONPOINT_PREVIEW_SCALE );

  return node;
}


void PolygonTool::destroyPolygonpointNode( Ogre::SceneNode *node )
{
  node ->removeAndDestroyAllChildren();
  scene_manager_ ->getRootSceneNode() ->removeChild( node );
  delete node;
}


int PolygonTool::processMouseEvent( rviz::ViewportMouseEvent &event ) 
{
  if ( editor_mode_ == MODE_POLYGON ) {
    if( polygonpoint_preview_node_ == nullptr ) return rviz::InteractionTool::processMouseEvent( event );

    //Perform raycast
    polygonpoint_preview_node_ ->setVisible( false ); // Disable for raycast
    Ogre::Vector3 intersection;
    if( !polygonpoint_3d_proporty_ ->getBool() || !context_ ->getSelectionManager()->get3DPoint( event.viewport, event.x, event.y, intersection ))
    {
      //Fall back to plane intersection
      Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.f );
      if( !rviz::getPointOnPlaneFromWindowXY( event.viewport, ground_plane, event.x, event.y, intersection ))
      { return Render; }
    }

    // Show preview node at current intersection unless we are in the process of placing a polygonpoint
    polygonpoint_preview_node_ ->setVisible( polygonpoint_active_node_ == nullptr, false );
    polygonpoint_preview_node_ ->setPosition( intersection );
    
    if( event.leftDown() )
    {
      polygonpoint_active_node_ = createPolygonpointNode( intersection );
      return Render;
    }

    if( event.rightDown())
    {
      if( polygonpoint_active_node_ != nullptr )
      {
        destroyPolygonpointNode( polygonpoint_active_node_ );
        polygonpoint_active_node_ = nullptr;
      }

      // Right click removes last waypoint
      if( polygonpoint_nodes_.empty()) return 0;
      removePolygonpoint( static_cast<int>( polygonpoint_nodes_.size() - 1 ));
      return Render;
    }

    if( polygonpoint_active_node_ != nullptr )
    {
      const Ogre::Vector3 &polygonpoint_position = polygonpoint_active_node_ ->getPosition();
      
      if ( event.leftUp())
      {
        polygonpoint_nodes_.push_back( polygonpoint_active_node_ );
        QVariantMap polygonpoint;
        polygonpoint.insert( "nummer", polygonpoints_.size()+1 );
        polygonpoint.insert( "position", QVector3D( polygonpoint_position.x, polygonpoint_position.y, polygonpoint_position.z ));
        polygonpoints_.push_back( polygonpoint );

        emit polygonpointAdded( polygonpoint );
        emit polygonpointsChanged();

        polygonpoint_active_node_ = nullptr;
      } 
      return Render;
    }

  // } else if( editor_mode_ == MODE_EDITOR || editor_mode_ == MODE_PRIMITIVE_ELEMENT ){
  //   if( editor_mode_ == MODE_EDITOR && map_ == nullptr ) return Render;
  //   if( editor_mode_ == MODE_PRIMITIVE_ELEMENT && map_ref_ == nullptr ) return Render;
  } else if( editor_mode_ == MODE_EDITOR ){
    // if( map_ == nullptr ) return Render;
    Ogre::Vector3 intersection;
    static Ogre::Vector3 start_position;
    Ogre::Camera* camera = event.viewport->getCamera(); 
    // hector_math::Vector2f origin_posi;

    if( event.leftDown())
    {
      Ogre::Vector3 viewportCoordinates = Ogre::Vector3( event.x, event.y, 0 );
      Ogre::Vector3 worldCoordinates = camera->getCameraToViewportRay(
        viewportCoordinates.x / event.viewport->getActualWidth(), viewportCoordinates.y / event.viewport->getActualHeight()).getPoint( depth_ );
      start_position = worldCoordinates;

      //Initial sub-maps retained
      if( editor_mode_ == MODE_EDITOR ) { map_original_ = map_->map(); }
      else if( editor_mode_ == MODE_PRIMITIVE_ELEMENT ) { map_original_ = map_ref_->map(); } 
      return Render;
    }

    if( !start_position.isZeroLength() ){
      Ogre::Vector3 viewportCoordinates = Ogre::Vector3( event.x, event.y, 0 );
      Ogre::Vector3 worldCoordinates = camera->getCameraToViewportRay(
        viewportCoordinates.x / event.viewport->getActualWidth(), viewportCoordinates.y / event.viewport->getActualHeight() ).getPoint( depth_ );
      intersection = worldCoordinates;
      Ogre::Real offset_z = intersection.z - start_position.z;
      Eigen::Ref<hector_math::GridMap<float>> map_to_editieren = ( editor_mode_ == MODE_EDITOR )? map_->map() : map_ref_->map();
      hector_world_heightmap::integrators::HeightmapIntegrator<float> heightmapIntegrator( whm_ );

      if ( std::abs( offset_z ) > 1E-6 ) { 
        map_to_editieren = map_original_ + offset_z /  skalenfaktor_;

        if( editor_mode_ == MODE_EDITOR ) {
          map_ = std::make_shared<Heightmap<float>>( map_to_editieren, map_->resolution(), map_->origin(), 
                                                     map_->frame(), map_->timestamp() );
          heightmapIntegrator.integrate( map_, integrators::IntegratorMode::SourceKnown );                                          
        } else if( editor_mode_ == MODE_PRIMITIVE_ELEMENT ) {
          // typename hector_world_heightmap::Heightmap<float>::Ptr map_pri_changed;
          map_ref_ = std::make_shared<Heightmap<float>>( map_to_editieren, map_ref_->resolution(), map_ref_->origin(), 
                                                         map_ref_->frame(), map_ref_->timestamp() );
          // heightmapIntegrator.integrate( map_ref_, integrators::IntegratorMode::SourceKnown );
          // whm_->removeMap( whm_->getMap( 0 ) );
          whm_ = make_shared<MapBag<float>>( resolution_ );
          whm_->setFrame( map_ref_->frame() );
          whm_->addMap( make_shared<Map<float>>
                      ( map_ref_->map(), Vector3<float>( map_ref_->origin()[0], map_ref_->origin()[1], 0 ), map_ref_->resolution(), 0, 0 ));
        }

        tempo_ = QVector3D( 0, 0, tempo_ref_.z() + offset_z /  skalenfaktor_ );   
        emit tempoChanged();
        publishToolmapInformation( whm_->getMap( 0 ));
      }
      
      if ( event.leftUp())
      {
        start_position = Ogre::Vector3::ZERO;
        tempo_ref_ = tempo_;
      } 
    }
    return Render;
  } else if ( editor_mode_ == MODE_VERSCHIEBEN ){
    return Render;
  }

  return rviz::InteractionTool::processMouseEvent( event );
}


void PolygonTool::mouseEventhelper( rviz::ViewportMouseEvent &event, typename hector_world_heightmap::Heightmap<float>::Ptr &map ) {

}


int PolygonTool::processKeyEvent( QKeyEvent *event, rviz::RenderPanel *panel ) 
{
  return rviz::InteractionTool::processKeyEvent( event, panel );
}


QVariantList PolygonTool::polygonpoints() const
{
  return polygonpoints_;
}


QString PolygonTool::frame() const
{
  return context_->getFixedFrame();
}


QVector3D PolygonTool::tempo() const
{
  return tempo_;
}


Q_INVOKABLE void PolygonTool::clearPolygonpoints()
{
  for( const auto &node : polygonpoint_nodes_ ) { destroyPolygonpointNode( node ); }
  polygonpoint_nodes_.clear();
  polygonpoints_.clear();
  emit polygonpointsChanged();
}


Q_INVOKABLE void PolygonTool::removePolygonpoint( int index )
{
  if ( index >= polygonpoints_.size() || index < 0 )
  {
    ROS_ERROR_STREAM_NAMED( "mapbag_editor_user_interface", "Index" <<  index << "out of bounds.");
    return;
  }

  Ogre::SceneNode *node = polygonpoint_nodes_.at( index );
  destroyPolygonpointNode( node );
  polygonpoint_nodes_.erase( polygonpoint_nodes_.begin() + index );
  QVariantMap polygonpoint = polygonpoints_[ index ].toMap();
  polygonpoints_.removeAt( index );
  emit polygonpointRemoved( index, polygonpoint );
  emit polygonpointsChanged();
}


Q_INVOKABLE void PolygonTool::addPolygonpoint( float x, float y, float z )
{
  const QVector3D &position = QVector3D( x, y, z ); 
  createPolygonpointNode( Ogre::Vector3{ position.x(), position.y(), position.z() } );
  QVariantMap polygonpoint;
  polygonpoint.insert( "nummer", polygonpoints_.size()+1);
  polygonpoint.insert( "position", position);
  polygonpoints_.push_back( polygonpoint );
}


Q_INVOKABLE void PolygonTool::changeEditorMode( int editor_mode )
{
  if( editor_mode != 0 && editor_mode != 1 && editor_mode != 2 && editor_mode != 3 ) return;
  if( editor_mode == 0 ){
    editor_mode_ = MODE_POLYGON;
  } else if ( editor_mode == 1 ){
    polygonpoint_preview_node_ ->setVisible( false );
    editor_mode_ = MODE_EDITOR;
  } else if ( editor_mode == 2 ){
    polygonpoint_preview_node_ ->setVisible( false );
    editor_mode_ = MODE_PRIMITIVE_ELEMENT;
  } else if ( editor_mode == 3 ){
    polygonpoint_preview_node_ ->setVisible( false );
    editor_mode_ = MODE_VERSCHIEBEN;
    if( map_ != nullptr ) {
      geometry_msgs::Pose center_pose;
      center_pose.position.x = map_->origin()[0];
      center_pose.position.y = map_->origin()[1];
      center_pose.position.z = map_->origin()[2];
      pub_submap_pos_.publish( center_pose );
    }
  }
}


Q_INVOKABLE void PolygonTool::changeAutomatischMode( int automatisch_mode )
{
  if( automatisch_mode != 0 && automatisch_mode != 1 ) return;
  if( automatisch_mode == 0 ){
    automatisch_mode_ = Aktivieren;
  } else if ( automatisch_mode == 1 ){
    automatisch_mode_ = Deaktivieren;
  } 
}


Q_INVOKABLE void PolygonTool::setMarker( int selected_Polygon_nummer )
{
  if( selected_Polygon_nummer <= 0 || selected_Polygon_nummer >  polygonpoints_.size() ) return; 

  for( const auto &node : polygonpoint_nodes_ )
  {
    auto *marker_node = dynamic_cast<Ogre::SceneNode *>( node->getChild( 0 ));
    marker_node->setVisible( false );
  }  
  auto *marker_node = dynamic_cast<Ogre::SceneNode *>( polygonpoint_nodes_[selected_Polygon_nummer - 1]->getChild( 0 ));
  marker_node->setVisible( true );

  emit polygonpointsChanged();
}


/** Modify the elevation value of the sub-map.
 * @param mode Three types of height changes can be accomplished depending on the settings: 
 *             additive, uniform or delete.
 * @param height target height value.
 */
Q_INVOKABLE void PolygonTool::setHeight( int mode, float height ) 
{
  Eigen::Ref<hector_math::GridMap<float>> map_to_editieren = map_->map();

  if( mode == 1 ) {
    //Additive Mode
    map_to_editieren += height; 
    tempo_ = QVector3D( 0, 0, tempo_.z() + height );   
    emit tempoChanged();

  } else if( mode == 0 ) {
    //Uniform Mode : Change only values that are not NaN
    map_to_editieren = map_to_editieren.array().isNaN().select( map_to_editieren, height );
    tempo_ = QVector3D( 0, 0, height );   
    emit tempoChanged();

  } else if( mode == -1 ) {
    //Delete Mode 
    map_to_editieren.array().setConstant( std::numeric_limits<float>::quiet_NaN() );
    tempo_ = QVector3D( 0, 0, std::numeric_limits<float>::quiet_NaN() ); 
    emit tempoChanged();
  }

  map_ = std::make_shared<Heightmap<float>>(
      map_to_editieren, map_->resolution(), map_->origin(), map_->frame(),
      map_->timestamp() );
  hector_world_heightmap::integrators::HeightmapIntegrator<float> heightmapIntegrator( whm_ );
  heightmapIntegrator.integrate( map_, integrators::IntegratorMode::SourceKnown );
  publishToolmapInformation( whm_->getMap( 0 ));  
  
}


/** Drop previous changes to the sub-map and send an empty map to RViz.
 */
Q_INVOKABLE void PolygonTool::dropChanges()
{
  publishToolmapInformation( mapref_copy_ );
}


Q_INVOKABLE void PolygonTool::deleteSubmap() 
{
  using namespace hector_world_heightmap::math;

  const float resolution = whm_->resolution();
  hector_math::Vector2<float> hm_pos = map_->origin();
  auto hm = map_->map();
  hector_math::Vector2<Eigen::Index> hm_size;
  hm_size << hm.rows(), hm.cols();

  Eigen::Index row_center, col_center;
  typename Map<float>::Ptr map = whm_->getMap( 0 );
  coordinateToIndex<float>( map->origin_.topRows( 2 ), map->map_.rows(), map->map_.cols(),
                             hm_pos, row_center, col_center, resolution );
  Eigen::Index start_row = row_center - hm_size.x() / 2;
  Eigen::Index start_col = col_center - hm_size.y() / 2;

  const auto &source = hm;
  auto &&target = map->map_.block( start_row, start_col, hm_size.x(), hm_size.y() );

  Eigen::Index rows = target.rows();
  Eigen::Index cols = target.cols();

  for ( Eigen::Index col = 0; col < cols; ++col ) {
    for ( Eigen::Index row = 0; row < rows; ++row ) {
      float value = source( row, col );
      if ( std::isnan( value ) )
        continue;
      target( row, col ) = std::numeric_limits<float>::quiet_NaN();
    }
  }
  publishToolmapInformation( whm_->getMap( 0 ));

  tempo_ = QVector3D( 0, 0, 0 );   
  emit tempoChanged();
}


/** Clear the current Submap.
 */
Q_INVOKABLE void PolygonTool::clearmap() 
{
  tempo_ = QVector3D( 0, 0, 0 );   
  emit tempoChanged();
  clearPolygonpoints();
  changeEditorMode( 0 );
  map_ = nullptr;
  map_ref_ = nullptr;
  mapref_copy_ = nullptr;
}


Q_INVOKABLE void PolygonTool::publishToServer()
{
  if( whm_->getMap( 0 ) == nullptr ) return;
  grid_map_msgs::GridMap msg = heightmapToMsg<float>( whm_->getMap( 0 ) );
  mapToServer_pub_.publish( msg );
  publishEmpty();
}


void PolygonTool::publishEmpty() 
{
  if( mapref_copy_ == nullptr ) return;
  Eigen::Ref<hector_math::GridMap<float>> map_empty = mapref_copy_->map();
  map_empty.array().setConstant( std::numeric_limits<float>::quiet_NaN() );
  mapref_copy_ = std::make_shared<Heightmap<float>>(
      map_empty, mapref_copy_->resolution(), mapref_copy_->origin(), mapref_copy_->frame(),
      mapref_copy_->timestamp() );
  publishToolmapInformation( mapref_copy_ );
  tempo_ = QVector3D( 0, 0, 0 );   
  emit tempoChanged();
}


/** Clear the sub-map translation operations so far.
 */
Q_INVOKABLE void PolygonTool::submapVerschiebenClear()
{
  clearPolygonpoints();
  verschieben_func_called_ = false;
  last_center_.setZero();;
  map_ = nullptr;
  map_ref_ = nullptr;
  publishEmpty();
}


/** Save the sub-map translation operations so far( send it to Server ).
 */
Q_INVOKABLE void PolygonTool::submapVerschiebenSave() 
{
  grid_map_msgs::GridMap msg = heightmapToMsg<float>( map_ );
  mapVerschieben_pub_.publish( msg );
  publishEmpty();
  clearmap(); 
}


/** Save the newly added primitive element bodies( send it to Server ).
 */
Q_INVOKABLE void PolygonTool::primitiveSave() 
{
  grid_map_msgs::GridMap msg = heightmapToMsg<float>( whm_->getMap( 0 ) );
  mapVerschieben_pub_.publish( msg );
  publishEmpty();
  clearmap(); 
}


Q_INVOKABLE void PolygonTool::primitiveClear() 
{
  publishEmpty();
  clearmap(); 
}


//Test_primitive_function
Q_INVOKABLE void PolygonTool::primitive( std::vector<int> mode, std::vector<double> center, std::vector<int> size )
{
  hector_math::GridMap<float> primitiv_pub;
  int length = size[0];
  int width = size[1];
  int pri_mode = mode[0];
  double height_tri = center[2];
  Vector2<float> center_xy = { center[0], center[1] };
  if( pri_mode == 0 ) 
  {
    //sphere_test
    int radius = mode[2];
    hector_math::GridMap<float> primitiv(( 1 + 2 * radius ), ( 1 + 2 * radius ));
    int rows = primitiv.rows();
    int cols = primitiv.cols();
    primitiv.array().setConstant( std::numeric_limits<float>::quiet_NaN() );
    // primitiv.array().setConstant( 0.2 );

    // **** Varient 1 : Geometric method
    // for ( int i = 0; i < rows; i++ ) {
    //   for ( int j = 0; j < cols; j++ ) {
    //     if( std::abs( std::abs( j - radius ) - std::sqrt( pow( radius, 2 ) - pow( i - radius, 2 ))) >= 0.5
    //         &&  pow( i - radius, 2 ) + pow( j - radius, 2 ) > pow( radius, 2) ) 
    //       { primitiv( i, j ) = std::numeric_limits<float>::quiet_NaN(); }
    //   }
    // }
    // **************

    // Varinet 2 : BresenhamCircle Algorithmus
    std::vector<std::pair<Eigen::Index,Eigen::Index>> circle_indices;
    bresenhamCircle( radius, circle_indices );
    for( const auto& element : circle_indices ) { primitiv( element.first, element.second ) = 0.2f; }
    for ( int i = 0; i < rows; i++ ) {
      for ( int j = 0; j < cols; j++ ) {
        if( pow( i - radius, 2 ) + pow( j - radius, 2 ) <= pow( radius, 2 ) ) 
          { primitiv( i, j ) = 0.2f; }
      }
    }
    primitiv_pub = primitiv;
  } else if( pri_mode == 1 ) {
    // cuboid_test
    int height = size[2];
    hector_math::GridMap<float> primitiv( length, width );
    int rows = primitiv.rows();
    int cols = primitiv.cols();
    primitiv.array().setConstant( resolution_ * height );
    // for ( int i = 0; i < rows; i++ ) {
    //   for ( int j = 0; j < cols; j++ ) {
    //     if (i == 0 || i == rows - 1 || j == 0 || j == cols - 1) {
    //       primitiv(i, j) = 0.0f;
    //     }
    //   }
    // }
    primitiv_pub = primitiv;

  } else if( pri_mode == 2 ) {
    // cube_test
    hector_math::GridMap<float> primitiv( length, length );
    int rows = primitiv.rows();
    int cols = primitiv.cols();
    primitiv.array().setConstant( resolution_ * length );
    primitiv_pub = primitiv;

  } else if( pri_mode == 3 ) {
    // tri_test
    // variable both : represents the direction in which the Tri. Prism is tilted
    //                 0 ( Only X Direction ); 1 ( Only Y Direction ); 2( Both )
    int both = mode[1];
    int x_rise = mode[2];
    int y_rise = size[2];
    hector_math::GridMap<float> primitiv( length, width );
    int rows = primitiv.rows();
    int cols = primitiv.cols();
    float val = 0;
    // const double PI = 3.14159265358979323846;
    // float tanValue = tan( height_tri * ( PI / 180.0 ));
    primitiv.array().setConstant( val );

    if( both == 0 ) {
      int step = x_rise ? 1 : -1;
      int start = x_rise ? 1 : rows - 2;
      float tanValue = height_tri / rows;
      for ( int i = start; i >= 0 && i < rows; i += step ) {
        val += resolution_ * tanValue;
        for ( int j = 0; j < cols; j++ ) {
          primitiv( i, j ) = val;
        }
      } 
    } else if( both == 1 ) {
      int step = y_rise ? 1 : -1;
      int start = y_rise ? 1 : cols - 2;
      float tanValue = height_tri / cols;
      for ( int i = start; i >= 0 && i < cols; i += step ) {
        val += resolution_ * tanValue;
        for ( int j = 0; j < rows; j++ ) {
          primitiv( j , i ) = val;
        }
      } 
    } else if( both == 2 ) {
      int step = x_rise ? 1 : -1;
      int start = x_rise ? 1 : rows - 2;
      float tanValue_x = height_tri / rows;
      float tanValue_y = height_tri / cols;
      for ( int i = start; i >= 0 && i < rows; i += step ) {
        val += resolution_ * tanValue_x;
        for ( int j = 0; j < cols; j++ ) {
          primitiv( i, j ) = val;
        }
      }

      int direc = y_rise? 0 : cols - 1;
      for( int i = 0; i < rows; i++ ) {
        primitiv( i, direc ) = 0;
      }
      
      val = 0;
      step = y_rise ? 1 : -1;
      start = y_rise ? 1 : cols - 2;
      for ( int i = start; i >= 0 && i < cols; i += step ) {
        val += resolution_ * tanValue_y;
        for ( int j = 0; j < rows; j++ ) {
          primitiv( j , i ) = min( val, primitiv( j, i ));
        }
      }
    }  
    primitiv_pub = primitiv;
  }
  
  std::shared_ptr<hector_world_heightmap::Heightmap<float>> map_ref_primitiv;
  map_ref_primitiv = std::make_shared<Heightmap<float>>(
      primitiv_pub, resolution_, center_xy, world_frame_, static_cast<long long>( ros::Time::now().toNSec() ) );
  map_ = map_ref_primitiv;
  map_ref_ = map_ref_primitiv;
  mapref_copy_ = map_ref_primitiv;
  whm_ = make_shared<MapBag<float>>( resolution_ );
  whm_->setFrame( world_frame_ );
  whm_->addMap( make_shared<Map<float>>
                      ( map_ref_->map(), Vector3<float>( map_ref_->origin()[0], map_ref_->origin()[1], 0 ), resolution_, 0, 0 ));
  publishToolmapInformation( whm_->getMap( 0 ));
  if( whm_ != nullptr && whm_->getMap( 0 ) != nullptr ) {
    geometry_msgs::Pose center_pose;
    center_pose.position.x = whm_->getMap( 0 )->origin()[0];
    center_pose.position.y = whm_->getMap( 0 )->origin()[1];
    center_pose.position.z = whm_->getMap( 0 )->origin()[2];
    pub_submap_pos_.publish( center_pose );
  }
}


/** 
 */
Q_INVOKABLE void PolygonTool::smooth_filter()
{
  Eigen::Ref<hector_math::GridMap<float>> map_to_editieren = map_->map();
  hector_math::GridMap<float> gridMapCopy( map_to_editieren );
  hector_math::GridMap<float> smoothed_map = adaptiveMeanFilter( gridMapCopy, 3, 7 );
  // Eigen::MatrixXf& underlyingMap = map_to_editieren.get();
  map_ = std::make_shared<Heightmap<float>>(
      smoothed_map, map_->resolution(), map_->origin(), map_->frame(),
      map_->timestamp() );
  hector_world_heightmap::integrators::HeightmapIntegrator<float> heightmapIntegrator( whm_ );
  heightmapIntegrator.integrate( map_, integrators::IntegratorMode::SourceKnown );
  publishToolmapInformation( whm_->getMap( 0 )); 
}

/** Publishing Submaps in Tools to RViz.
 * @param map Submap in PolygonTool.
 */
void PolygonTool::publishToolmapInformation( const typename hector_world_heightmap::HeightmapRef<float>::ConstPtr &map )
{
  if( map == nullptr ) return;
  grid_map_msgs::GridMap msg = heightmapToMsg<float>( map );
  mapInTool_pub_.publish( msg );
}


Ogre::SceneNode *PolygonTool::createPolygonpointNode( const Ogre::Vector3 &position )
{
  Ogre::SceneNode *node = createPolygonpointNode();
  node ->setVisible( true, false );
  node ->setPosition( position );

  auto *marker_node = dynamic_cast<Ogre::SceneNode *>( node->getChild( 0 ));
  marker_node->setPosition( Ogre::Vector3{ 0, 0, 0.4 } );
  marker_node->setOrientation( Ogre::Quaternion( std::cos( std::atan2( 1, 1 ) ), 0, 0, std::sin( std::atan2( 1, 1 ) )));

  return node;
}


/** Callback functions for handling topic ("/mapbag_editor_server_node/mapbag_resolution") data updates.
 * @param resulotion Resulotion data of Mapbag sent by MapbagEditorServer.
 */
void PolygonTool::resolutionCallback( const std_msgs::Float64 &resulotion )
{
  resolution_ = resulotion.data;
}


/** Callback functions for handling topic ("/mapbag_editor_server_node/mapbag_frame") data updates.
 * @param resulotion Frame data of Mapbag sent by MapbagEditorServer.
 */
void PolygonTool::frameCallback( const std_msgs::String &frame )
{
  world_frame_ = frame.data;
}


/** Callback functions for handling topic ("/mapbag_editor_server_node/submap") data updates.
 * @param msg Map data sent by MapbagEditorServer.
 */
void PolygonTool::submapSubCallback( const grid_map_msgs::GridMap &msg )
{
  submapSubscribe( msg, map_ );
}


/** Callback functions for handling topic ("/mapbag_editor_server_node/submap_ref") data updates.
 * @param msg Map data sent by MapbagEditorServer.
 */
void PolygonTool::submapRefSubCallback( const grid_map_msgs::GridMap &ref_msg )
{
  submapSubscribe( ref_msg, map_ref_ );
  mapref_copy_ = std::make_shared<hector_world_heightmap::Heightmap<float>>( *map_ref_ );

  //Initialization of Mapbag whm_
  auto resolution = map_ref_->resolution();
  whm_ = make_shared<MapBag<float>>( resolution );
  whm_->setFrame( map_ref_->frame() );
  whm_->addMap( make_shared<Map<float>>
                      ( map_ref_->map(), Vector3<float>( map_ref_->origin()[0], map_ref_->origin()[1], 0 ), resolution, 0, 0 ));
  publishToolmapInformation( whm_->getMap( 0 ));
  if( whm_ != nullptr && whm_->getMap( 0 ) != nullptr ) {
    geometry_msgs::Pose center_pose;
    center_pose.position.x = whm_->getMap( 0 )->origin()[0];
    center_pose.position.y = whm_->getMap( 0 )->origin()[1];
    center_pose.position.z = whm_->getMap( 0 )->origin()[2];
    pub_submap_pos_.publish( center_pose );
  }

  tempo_ = QVector3D( 0, 0, 0 );
  tempo_ref_ = QVector3D( 0, 0, 0 );
  emit tempoChanged();
}


/** Subscribe to the map data( "msg" ) sent by the server and save it in param "map" of tool.
 * @param msg Map data sent by MapbagEditorServer.
 * @param map Maps in PolygonTool.
 */
void PolygonTool::submapSubscribe( const grid_map_msgs::GridMap &msg, hector_world_heightmap::Heightmap<float>::Ptr &map )
{
  map = nullptr;
  size_t index = 0;
  hector_math::GridMap<float> submap_in_tool = hector_math::msgToGridMap<float>( msg.data[index], msg.outer_start_index,
                                                           msg.inner_start_index );
  assert( std::abs( msg.info.pose.position.z ) < 1E-4 && "Expected grid map pose z to be 0." );
  Vector2<float> position( msg.info.pose.position.x, msg.info.pose.position.y ); 

  map = std::make_shared<Heightmap<float>>(
      submap_in_tool, msg.info.resolution, position, msg.info.header.frame_id,
      static_cast<long long>( msg.info.header.stamp.toNSec() ));
}


/** 
 * @param radius 
 * @param indices 
 */
void PolygonTool::bresenhamCircle( const int &radius, std::vector<std::pair<Eigen::Index,Eigen::Index>> &indices )
{
  int x= 0;
  int y = radius;
  int decision = 3 - ( 2 * radius );

  while ( x<=y ) {
    if ( decision <= 0 ) { decision += ( 4 * x ) + 6; }
    else {
      decision += ( 4 * x ) - ( 4 * y ) + 10;  
      y--;  
    }
    x++;
    indices.push_back( std::make_pair( radius + x, radius + y ));
    indices.push_back( std::make_pair( radius - x, radius + y ));
    indices.push_back( std::make_pair( radius + x, radius - y ));
    indices.push_back( std::make_pair( radius - x, radius - y ));
    indices.push_back( std::make_pair( radius + y, radius + x ));
    indices.push_back( std::make_pair( radius - y, radius + x ));
    indices.push_back( std::make_pair( radius + y, radius - x ));
    indices.push_back( std::make_pair( radius - y, radius - x ));
  }
}

/** Callback functions for handling topic ("/submap_new_center") data updates.
 * @param center new center location data sent by InteractiveMarkerServer.
 */
void PolygonTool::newCenterSubCallback( const geometry_msgs::Point &center )
{
  if( map_ == nullptr ) return;

  hector_math::Vector3<float> new_pos = { static_cast<float>( center.x ), static_cast<float>( center.y ), static_cast<float>( center.z ) };
  if( last_center_.isZero() ) { last_center_ = new_pos; } 
  else 
  {
    if( new_pos[0] != last_center_[0] || new_pos[1] != last_center_[1] ) {
      Eigen::Ref<hector_math::GridMap<float>> map_verschieben = map_->map();
      
      map_ = std::make_shared<Heightmap<float>>(
          map_verschieben, map_->resolution(), new_pos.template topRows<2>(), map_->frame(),
          map_->timestamp() );
    }  
    if( abs( new_pos[2] - last_center_[2] ) > 1E-4 ) {
      const Eigen::ArrayXXf map_original = map_->map();
      Eigen::Ref<hector_math::GridMap<float>> map_to_editieren = map_->map();
      map_to_editieren = map_original + ( new_pos[2] - last_center_[2] ); 
      map_ = std::make_shared<Heightmap<float>>(
                map_to_editieren, map_->resolution(), map_->origin(), map_->frame(),
                map_->timestamp() ); 
    }
    last_center_ = new_pos;
  }
  publishToolmapInformation( map_ );
  
  // **************** Synchronized updated map_ref_
  // Eigen::Ref<hector_math::GridMap<float>> map_ref_verschieben = map_ref_->map();
  // typename Map<float>::Ptr map = whm_->getMap( 0 );
  // auto resolution = whm_->getMap( 0 )->resolution();
  // auto frame = whm_->getMap( 0 )->frame();
  // whm_ = make_shared<MapBag<float>>( resolution );
  // whm_->setFrame( frame );
  // // whm_->removeMap( map );
  // whm_->addMap( make_shared<Map<float>>
  //                     ( map_ref_verschieben, Vector3<float>( new_pos[0], new_pos[1], 0 ), map_ref_->resolution(), 0, 0 ));
  // mapref_copy_ = std::make_shared<Heightmap<float>>(
  //     map_ref_verschieben, map_ref_->resolution(), new_pos, map_ref_->frame(),
  //     map_ref_->timestamp() );
  // publishToolmapInformation( whm_->getMap( 0 )); 
  // ************************************************   
}

} //namespace mapbag_editor_user_interface

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS( mapbag_editor_user_interface::PolygonTool, rviz::Tool )
