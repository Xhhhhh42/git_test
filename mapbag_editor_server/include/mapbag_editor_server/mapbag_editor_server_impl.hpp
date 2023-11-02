#ifndef MAPBAG_EDITOR_SERVER_MAPBAG_EDITOR_SERVER_IMPL_HPP
#define MAPBAG_EDITOR_SERVER_MAPBAG_EDITOR_SERVER_IMPL_HPP

#include "mapbag_editor_server.h"
#include "polygon_indices_iterator.h"
#include "polygonCheck.h"

#include <hector_world_heightmap/io.h>
#include <hector_world_heightmap/map_bag.h>
#include <hector_math_ros/message_conversions/geometry_msgs.h>
#include <hector_math/iterators/polygon_iterator.h>
#include <memory>

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <unordered_map>

namespace mapbag_editor_server
{
using namespace hector_world_heightmap;
using namespace hector_world_heightmap_ros::message_conversions;
using namespace hector_math;
using namespace std;

template<typename Scalar>
MapbagEditorServer<Scalar>::MapbagEditorServer( const ros::NodeHandle &nh, const ros::NodeHandle &pnh )
    : nh_( nh ), pnh_( pnh )
{
  resolution_ = pnh_.param( "map_resolution", 0.0 );
  world_frame_ = pnh_.param<string>( "world_frame", "world" );
  ROS_INFO_NAMED( "MapbagEditorServer", "World frame: %s", world_frame_.c_str() );

  if ( resolution_ == 0.0 ) {
    ROS_INFO_NAMED( "MapbagEditorServer",
                    "Resolution will be set automatically when first map is received." );
  } else {
    mapbag_ = make_shared<MapBag<Scalar>>( resolution_ );
    mapbag_->setFrame( world_frame_ );
    initForNewMap();
    ROS_INFO_NAMED( "MapbagEditorServer", "Resolution: %f", resolution_ );
  }

  save_service_ = pnh_.advertiseService( "save_map", &MapbagEditorServer<Scalar>::onSaveMap, this );
  load_service_ = pnh_.advertiseService( "load_map", &MapbagEditorServer<Scalar>::onLoadMap, this );
  polygongridmap_service_ = pnh_.advertiseService( "polygongridmap", &MapbagEditorServer<Scalar>::onPolygonGridMap, this );
  interpolation_service_ = pnh_.advertiseService( "interpolation", &MapbagEditorServer<Scalar>::onInterpolation, this );
  clearmapbag_service_ = pnh_.advertiseService( "clearmapbag", &MapbagEditorServer<Scalar>::onClearMapbag, this );
  undo_service_ = pnh_.advertiseService( "undo", &MapbagEditorServer<Scalar>::onUndo, this );
  redo_service_ = pnh_.advertiseService( "redo", &MapbagEditorServer<Scalar>::onRedo, this );
  save_mode_service_ = pnh_.advertiseService( "modechange", &MapbagEditorServer<Scalar>::onSaveModeChange, this );
  system_settings_service_ = pnh_.advertiseService( "systemsettings", &MapbagEditorServer<Scalar>::onSettingsChange, this );

  pub_resolution_ = pnh_.advertise<std_msgs::Float64>( "mapbag_resolution", 1, true );
  pub_frame_ = pnh_.advertise<std_msgs::String>( "mapbag_frame", 1, true );

  pub_submap_ = pnh_.advertise<grid_map_msgs::GridMap>( "submap", 1, true );
  pub_submap_ref_ = pnh_.advertise<grid_map_msgs::GridMap>( "submap_ref", 1, true );
  // pub_submap_pos_ = pnh_.advertise<geometry_msgs::Pose>( "submap_posi", 1, true );

  changedsubmap_sub_ = nh_.subscribe<grid_map_msgs::GridMap>(
      "changedsubmap", 5, [&]( const grid_map_msgs::GridMap::ConstPtr &map ) { saveSubmapCallback( *map ); });
  verschiebsubmap_sub_ = nh_.subscribe<grid_map_msgs::GridMap>(
      "verschiebsubmap", 5, [&]( const grid_map_msgs::GridMap::ConstPtr &map ) { saveVerschiebenCallback( *map ); });

  last_confirmed_points_.clear(); 
  polygon_points_.clear(); 
  save_mode_ = SAVE_DEAKTIV;
  polygon_mode_ = KONKAVHULL;
  intepolation_mode_ = 0;
  smooth_mode_ = 0;


  //test
  std::vector<std::pair<Eigen::Index, Scalar>> test_input;
  test_input.emplace_back( 1, 0.8 );
  test_input.emplace_back( 2, numeric_limits<Scalar>::quiet_NaN() );
  test_input.emplace_back( 3, numeric_limits<Scalar>::quiet_NaN() );
  test_input.emplace_back( 4, numeric_limits<Scalar>::quiet_NaN() );
  test_input.emplace_back( 5, numeric_limits<Scalar>::quiet_NaN() );
  test_input.emplace_back( 6, 0.2 );
  // test_input.emplace_back( 7, numeric_limits<Scalar>::quiet_NaN() );
  // test_input.emplace_back( 8, 0.4 );
  // test_input.emplace_back( 9, 0.7 );
  // test_input.emplace_back( 10, numeric_limits<Scalar>::quiet_NaN() );
  // test_input.emplace_back( 11, numeric_limits<Scalar>::quiet_NaN() );

  cubic_spline_ = make_shared<Cubic_Spline<Scalar>>( test_input );
  std::vector<std::pair<Eigen::Index, Scalar>> result = cubic_spline_->interpolated_points();
  for ( size_t i = 0; i < result.size(); i++ ) 
  {
    // ROS_INFO_STREAM_NAMED("MapbagEditorServer", 
    //                       "test: " << "index: " << result[i].first << "value: " << result[i].second );
  }
  
}


template<typename Scalar>
void MapbagEditorServer<Scalar>::initForNewMap()
{
  resolution_ = mapbag_->resolution();
  std_msgs::Float64 resolution_msg;
  resolution_msg.data = resolution_;
  pub_resolution_.publish( resolution_msg );

  //muss weiter optimiert werden
  submap_ = mapbag_->getMap(0);
  submap_origin_ = submap_->origin(); 
  submap_row_min_ = 0;
  submap_row_max_ = submap_->map_.rows() - 1;
  submap_col_min_ = 0;
  submap_col_max_ = submap_->map_.cols() - 1;
  submap_size_ << submap_->map_.rows(), submap_->map_.cols();
  string frame_ = mapbag_->frame();
  if ( world_frame_ != frame_ ) {
    ROS_WARN_STREAM_NAMED("MapbagEditorServer", 
                          "world_frame_ and frame_ are different: world_frame_ = " << world_frame_ << ", frame_ = " << frame_);
  }
  std_msgs::String frame_msg;
  frame_msg.data = frame_;
  pub_frame_.publish( frame_msg );

  if( invoker_ == nullptr ) invoker_ = make_shared<Invoker>();
}


/** Response to service "save_map" requests.
 * @param req Request object representing a string type.
 */
template<typename Scalar>
bool MapbagEditorServer<Scalar>::onSaveMap( hector_std_msgs::StringServiceRequest &req,
                                            hector_std_msgs::StringServiceResponse & )
{ return saveMap( req.param ); }


/** Speichern der modifizierten Karte.
 * @param path Path to the map file.
 */
template<typename Scalar>
bool MapbagEditorServer<Scalar>::saveMap( const std::string &path )
{
  if ( mapbag_ == nullptr ) {
    ROS_ERROR( "Map is not initialized yet and can not be saved!" );
    return false;
  }
  ROS_INFO_STREAM_NAMED("MapbagEditorServer", 
                          "Mapsave: " << path );
  if( path == "save" ) {
    hector_world_heightmap::io::writeToFile( *mapbag_, map_url_ );
    ROS_INFO_STREAM_NAMED("MapbagEditorServer", 
                          "Mapsave: " << map_url_ );
    ROS_INFO_STREAM_NAMED("MapbagEditorServer", 
                          "Mapsave: " << path );
  }
  else { hector_world_heightmap::io::writeToFile( *mapbag_, path ); }
  invoker_->reset();
  saveModeChange( int( 0 ));
  return true;
}


/** Response to service "load_map" requests.
 * @param req Request object representing a string type.
 */
template<typename Scalar>
bool MapbagEditorServer<Scalar>::onLoadMap( hector_std_msgs::StringServiceRequest &req,
                                            hector_std_msgs::StringServiceResponse & )
{ return loadMap( req.param ); }


/** Karte öffnen (Mapbag) und Visualisierung in RViz.
 * @param path Path to the map file.
 */
template<typename Scalar>
bool MapbagEditorServer<Scalar>::loadMap( const std::string &path )
{
  map_url_ = path;
  auto new_map = hector_world_heightmap::io::loadFromFile<Scalar>( path );
  if ( new_map == nullptr ) return false;
  mapbag_ = new_map;
  
  initForNewMap();
  ROS_INFO_STREAM_NAMED( "MapbagEditorServer",
                         "Loaded map. Resolution: " << resolution_
                                                    << ". Frame: " << world_frame_ );                                             
  publishMapbagInformation();
  return true;
}


/** Update polygonpoints_indices_ according to multipoints, to convert the set of coordinate points to index space.
 * @param multipoints polygon_points_: The set of incoming polygon points.
 */
template<typename Scalar>
void MapbagEditorServer<Scalar>::updateMultipointsIndices( const std::vector<hector_math::Vector3<Scalar>> &multipoints )
{
  using namespace hector_world_heightmap::math;

  polygonpoints_indices_.resize( 0, 0 );
  polygonpoints_indices_.resize( 2, multipoints.size());
  for ( size_t i = 0; i < multipoints.size(); i++ ) {
    MapBagIndex  index;
    mapbag_->getIndexAt( multipoints[i], index );
    polygonpoints_indices_( 0, i ) = index.index.row;
    polygonpoints_indices_( 1, i ) = index.index.col;
  }
}


/** Saves the input 3D point set to polygon_points_.
 * @param poly_points input 3D point set.
 */
template<typename Scalar>
void MapbagEditorServer<Scalar>::updatePolygonPoints( const std::vector<hector_math::Vector3<Scalar>> &poly_points )
{
  polygon_points_ = poly_points;
}


/** Optimierung der Start- und Zielpunkte von Linien.
 *  ( Da sich der Mittelpunkt bei der Erstellung der Unterkarte von (2, 2) im oberen rechten Punkt des Unterkartenquadrats befindet )
 * @param index_punkt input Index of point.
 */
template<typename Scalar>
void MapbagEditorServer<Scalar>::index_line_op( hector_world_heightmap::MapBagIndex &index_punkt, const hector_math::Vector3<Scalar> &polygon_point ) 
{
  Vector3<Scalar> location_punkt = mapbag_->getLocationForIndex( index_punkt );
  if( location_punkt[0] < polygon_point[0] && location_punkt[1] < polygon_point[1] ) {
      index_punkt.index.row += 1;
      index_punkt.index.col += 1; 
    } else if( location_punkt[0] < polygon_point[0] && location_punkt[1] > polygon_point[1] ) {
      index_punkt.index.row += 1;
    } else if( location_punkt[0] > polygon_point[0] && location_punkt[1] < polygon_point[1] ) {
      index_punkt.index.col += 1;
    }
}


/** Publish Mapbag as ROS topic, publish each sub-map separately.
 */
template<typename Scalar>
void MapbagEditorServer<Scalar>::publishMapbagInformation()
{
  if ( mapbag_ == nullptr ) return;
  
  for ( size_t i = map_publishers_.size(); i < mapbag_->numberOfMaps(); ++i ) {
    map_publishers_.push_back(
        pnh_.advertise<grid_map_msgs::GridMap>( "heightmaps/map" + to_string( i ), 1, true ) );
  }

  for ( size_t i = 0; i < mapbag_->numberOfMaps(); ++i ) {
    grid_map_msgs::GridMap msg = heightmapToMsg<Scalar>( mapbag_->getMap( i ));
    map_publishers_[i].publish( msg );
  }
}


/** Publish the generated polygon submaps.
 * @param points Number of polygon endpoints.
 */
template<typename Scalar>
void MapbagEditorServer<Scalar>::publishSubmapInformation( const size_t &points ) 
{
  if( points <= 0 ) return;
  if( points == 1 ) {
    // muss noch optimiert werden
    typename HeightmapRef<Scalar>::ConstPtr submap = mapbag_->getSubMap( polygon_points_[0], 1, 1 );
    if ( submap == nullptr ) { ROS_WARN_STREAM_NAMED( "MapbagEditorServer", " Submap ist leer. " ); }
    // geometry_msgs::Pose center_pose;
    // center_pose.position.x = polygon_points_[0][0];
    // center_pose.position.y = polygon_points_[0][1];
    // center_pose.position.z = polygon_points_[0][2];
    // pub_submap_pos_.publish( center_pose );
    // pub_submap_.publish( heightmapToMsg<Scalar>( submap ));
  } else {
    if ( whm_ == nullptr ) return;
    int i = 0;
    whm_->getMap( 0 )->map_.setConstant( numeric_limits<Scalar>::quiet_NaN() );
    hector_world_heightmap::integrators::HeightmapIntegrator<Scalar> heightmapIntegrator( whm_ );

    for ( auto it = sub_submaps_.begin(); it != sub_submaps_.end(); ++it )
    {
      const auto& element = *it;
      heightmapIntegrator.integrate( 
                          make_shared<Heightmap<Scalar>>( element, resolution_, submap_locations_[i], "" ), 
                          integrators::IntegratorMode::SourceKnown );
      i++;
    }
    // geometry_msgs::Pose center_pose;
    // center_pose.position.x = whm_->getMap( 0 )->origin()[0];
    // center_pose.position.y = whm_->getMap( 0 )->origin()[1];
    // center_pose.position.z = whm_->getMap( 0 )->origin()[2];
    // pub_submap_pos_.publish( center_pose );
    pub_submap_.publish( heightmapToMsg<Scalar>( whm_->getMap( 0 )));
  }
}


/** Responding to the "Generate Submap" service.
 * @param req The SubmapRequest containing the polygon points.
 * @param res The SubmapResponse to be populated with the generated submap.
 * @return True if the submap generation is successful, false otherwise.
 */
template<typename Scalar>
bool MapbagEditorServer<Scalar>::onPolygonGridMap( mapbag_editor_msgs::SubmapRequest &req,
                                                   mapbag_editor_msgs::SubmapResponse &resp )
{
  //Checks if the 'datas' field in the SubmapRequest is empty
  if ( req.datas.empty() ) 
  {
    ROS_WARN_STREAM_NAMED( "MapbagEditorServer", "Received empty datas in request." );
    return false; 
  }
  poly_submap_confirmed_ = false;

  

  // Process the data from the request and update 'confirmed_points'
  std::vector<hector_math::Vector3<Scalar>> confirmed_points;
  for ( const auto &data : req.datas ) {
    Vector3<Scalar> result = msgToVector<Scalar>( data );
    confirmed_points.emplace_back( result );
  }

  // Check if 'confirmed_points' have changed; update polygon points accordingly
  if( confirmed_points != last_confirmed_points_ ) 
  {
    last_confirmed_points_.clear();
    last_confirmed_points_ = confirmed_points;

    if( confirmed_points.size() == 1 || confirmed_points.size() == 2 || polygon_mode_ == ROOMWALLS ) { updatePolygonPoints( confirmed_points ); } 
    else if( polygon_mode_ == KONKAVHULL ) {
      // if() {
            //mode_test
            // ROS_INFO_STREAM_NAMED( "MapbagEditorServer", req.mode );
            //response_test
            // resp.result = 23;
      //   return polygonGridMap( polygon_points_ );
      // }
      // concav_hull_generator_ = make_shared<Concav_Hull_Generator<Scalar>>( confirmed_points );
      // concav_hull_generator_->concavHull( confirmed_points, 3 );
      // std::vector<hector_math::Vector3<Scalar>> concav_points = concav_hull_generator_->concav_points();
      // for ( size_t i = 0; i < concav_points.size(); i++ ) 
      // {
      //   ROS_INFO_STREAM_NAMED("MapbagEditorServer", 
      //                         "concav: " << "x: " << concav_points[i][0] << ", y: " << concav_points[i][1] << ", z: " << concav_points[i][2] );
      // }
      if( !polygonCheck( confirmed_points ))
      { ROS_INFO_STREAM_NAMED( "MapbagEditorServer", 
                                "Concavhull false, please check the input polygon points!" );
        resp.result = 1;
      } else { resp.result = 0; }
      updatePolygonPoints( confirmed_points );
    } else if( polygon_mode_ == KONVEXHULL ) {
      // For more points, compute the convex hull using Graham Scan
      std::vector<hector_math::Vector3<Scalar>> ch_vertex_points;
      gs_ch_executor_ = make_shared<GrahamScan_CH_Executor<Scalar>>( confirmed_points );
      gs_ch_executor_->grahamScan();
      ch_vertex_points = gs_ch_executor_->ch_points();
      updatePolygonPoints( ch_vertex_points );
    }
  } else {
    ROS_INFO_STREAM_NAMED( "MapbagEditorServer", "Polygonpoint_datas are the same as before." );
  }

  // Generate the submap based on the polygon points
  return polygonGridMap( polygon_points_ );
} 


/** Generate a separate Mapbag（ whm_ ）for map editing requirements..
 */
template<typename Scalar>
bool MapbagEditorServer<Scalar>::polygonGridMap( const std::vector<hector_math::Vector3<Scalar>> &polygon_points )
{
  whm_ = make_shared<MapBag<Scalar>>( resolution_ );
  whm_->setFrame( world_frame_ );
  if ( polygon_points.size() == 0 ) { return false; }
  else { polygonSubmap( polygon_points ); }

  publishSubmapInformation( polygon_points.size() ); 
  return true;
}


/** Generate the corresponding sub-map based on a given set of polygon points.
 * @param polygon_points A vector containing 3D points representing the polygon.
 *                       Each point should have x, y, and z coordinates.
 */
template<typename Scalar>
void MapbagEditorServer<Scalar>::polygonSubmap( const std::vector<hector_math::Vector3<Scalar>> &polygon_points ) 
{  
  // Clear previous submaps and submap locations
  sub_submaps_.clear();
  submap_locations_.clear();

  // Initialize variables for calculating the polygon's bounding box
  MapBagIndex index_center;
  Eigen::Index polygon_maxrow = numeric_limits<Eigen::Index>::min();
  Eigen::Index polygon_minrow = numeric_limits<Eigen::Index>::max();
  Eigen::Index polygon_maxcol = numeric_limits<Eigen::Index>::min();
  Eigen::Index polygon_mincol = numeric_limits<Eigen::Index>::max();

  // Handle cases depending on the number of points( point, line, polygon )
  if( polygon_points.size() == 0 ) return;
  else if( polygon_points.size() == 1 ) {
    // muss noch optimiert werden
    typename HeightmapRef<Scalar>::ConstPtr submap = mapbag_->getSubMap( polygon_points[0], 3, 3 );
    grid_map_msgs::GridMap msg = heightmapToMsg<Scalar>( submap );
    pub_submap_ref_.publish( msg );
    invoker_backup( msgToHeightmap<Scalar>( msg, "elevation" ) );
    return;
  } else if( polygon_points.size() == 2 ) { index_line_generate( polygon_points[0], polygon_points[1], polygonIte_indices_ ); 
  } else if( polygon_mode_ == KONKAVHULL ){
    updateMultipointsIndices( polygon_points );
    polygonIte_indices_.clear();
    hector_math::iteratePolygon( polygonpoints_indices_, submap_row_min_, submap_row_max_, submap_col_min_, submap_col_max_,
                                          [this]( Eigen::Index x, Eigen::Index y ) {
                                              Vector2<Eigen::Index> new_index( x, y );
                                              polygonIte_indices_.emplace_back( new_index );
                                          });     
  } else if( polygon_mode_ == KONVEXHULL ){
    updateMultipointsIndices( polygon_points );
    polygonIte_indices_.clear();
    polygonIterator( polygonpoints_indices_, [this]( Eigen::Index x, Eigen::Index y ) {
                                                Vector2<Eigen::Index> new_index( x, y );
                                                polygonIte_indices_.emplace_back( new_index );
                                            }) ;     
  } else if( polygon_mode_ == ROOMWALLS ) {
    polygonIte_indices_.clear();
    for( size_t i = 1; i < polygon_points.size(); i++ ) {
      vector<Vector2<Eigen::Index>> line_index;
      index_line_generate( polygon_points[ i - 1 ], polygon_points[i], line_index );
      for( const auto &element : line_index ) {
        polygonIte_indices_.push_back( element ); 
      }
    }    
  }

  // Calculate the bounding box of the polygon
  for ( const auto& index : polygonIte_indices_ ) {
    polygon_minrow = min( polygon_minrow, index[0] );
    polygon_maxrow = max( polygon_maxrow, index[0] );
    polygon_mincol = min( polygon_mincol, index[1] );
    polygon_maxcol = max( polygon_maxcol, index[1] );
  } 

  // Calculate the center of the polygon
  Eigen::Index index_center_row, index_center_col;
  if( ( polygon_minrow + polygon_maxrow ) % 2 == 0 ) index_center_row = static_cast<Eigen::Index>(( polygon_minrow + polygon_maxrow + 1 ) / 2 );
  else index_center_row = static_cast<Eigen::Index>(( polygon_minrow + polygon_maxrow ) / 2 );
  if( ( polygon_mincol + polygon_maxcol ) % 2 == 0 ) index_center_col = static_cast<Eigen::Index>(( polygon_mincol + polygon_maxcol + 1 ) / 2 );
  else index_center_col = static_cast<Eigen::Index>(( polygon_mincol + polygon_maxcol ) / 2 );

  index_center = {
    .map_index = 0,
    .index = {
      .row = index_center_row,
      .col = index_center_col
    }
  };
  Vector3<Scalar> location_center = mapbag_->getLocationForIndex( index_center );

  // Get the submap of the polygon
  typename HeightmapRef<Scalar>::ConstPtr polygon_submap = mapbag_->getSubMap( 
                                                        location_center, polygon_maxrow - polygon_minrow + 4, polygon_maxcol - polygon_mincol + 4 );
  GridMap<Scalar> map = polygon_submap->map();

  //Version 1 : Subtractive Submap
  // for( const auto& element : polygonIte_indices_ ) {
  //   ...  
  // }
                                            
  whm_->addMap( make_shared<Map<Scalar>>
                      ( map, Vector3<Scalar>( polygon_submap->origin()[0], polygon_submap->origin()[1], 0 ), resolution_, 0, 0 ));
  grid_map_msgs::GridMap msg = heightmapToMsg<Scalar>( whm_->getMap( 0 ));
  pub_submap_ref_.publish( msg );
  invoker_backup( msgToHeightmap<Scalar>( msg, "elevation" ) );

  //Version 2 : additive Subamp
  MapBagIndex index_c_sub;
  whm_->getIndexAt( location_center, index_c_sub );
  for( const auto& element : polygonIte_indices_ ) {
    MapBagIndex index_temp{ 0, { element[0], element[1] }};
    MapBagIndex index_sub{ 0, { element[0] - index_center.index.row + index_c_sub.index.row, 
                                element[1] - index_center.index.col + index_c_sub.index.col }};

    typename HeightmapRef<Scalar>::ConstPtr sub_submap;
    if( polygon_points.size() == 2 || polygon_mode_ == ROOMWALLS ) {
      sub_submap = mapbag_->getSubMap( mapbag_->getLocationForIndex( index_temp ), 2, 2 );
    } else if( polygon_points.size() > 2 ) {
      sub_submap = mapbag_->getSubMap( mapbag_->getLocationForIndex( index_temp ), 1, 1 );
    }

    // Store the sub-submap and its location
    sub_submaps_.emplace_back( sub_submap->map() );
    Vector3<Scalar> sub_location = whm_->getLocationForIndex( index_sub );
    submap_locations_.emplace_back( sub_location.template topRows<2>() );
  }   

  poly_submap_confirmed_ = true;
}


template<typename Scalar>
void MapbagEditorServer<Scalar>::index_line_generate( const hector_math::Vector3<Scalar> &point_start, const hector_math::Vector3<Scalar> &point_end,
                                                      std::vector<hector_math::Vector2<Eigen::Index>> &indices )
{
  MapBagIndex index_start, index_end;
  mapbag_->getIndexAt( point_start, index_start );
  mapbag_->getIndexAt( point_end, index_end );

  index_line_op( index_start, point_start );
  index_line_op( index_end, point_end );

  bh_line_executor_ = make_shared<Bresenhams_Line_Executor<Eigen::Index>>( index_start, index_end );
  bh_line_executor_->bresenhams_line();
  indices.clear();
  indices = bh_line_executor_->line_indices();
}


template<typename Scalar>
void MapbagEditorServer<Scalar>::invoker_backup( const std::shared_ptr<HeightmapRef<float>>& map ) 
{
  if( map == nullptr ) return;
  else {
    Command* command = new EditMapCommand( this, map, int( 1 ));
    invoker_->check_und_delete();
    invoker_->execute( command );
    invoker_->delete_backup();
  }
}
                                  

template<typename Scalar>
std::pair<MapIndex, bool> MapbagEditorServer<Scalar>::findNearestNonNaN( 
                                  std::vector<Vector2<Eigen::Index>> &indicesMap, const MapIndex& start_index, int row_step, int col_step )
{
  Vector2<Eigen::Index> index = { start_index.row, start_index.col };
  while ( find( indicesMap.begin(), indicesMap.end(), index ) != indicesMap.end() ) {
    if ( !isnan( submap_->getValueAt( MapIndex{ index[0], index[1] } ) ) ) {
        return make_pair( MapIndex{ index[0], index[1] }, true );
    }
    index[0] += row_step;
    index[1] += col_step;
  }
  return make_pair( start_index, false ); 
}


template<typename Scalar>
bool MapbagEditorServer<Scalar>::onInterpolation( hector_std_msgs::StringServiceRequest &,
                                                  hector_std_msgs::StringServiceResponse & )
{
  if( poly_submap_confirmed_ == false ) return false;
  return interpolation();
}


template<typename Scalar>
bool MapbagEditorServer<Scalar>::interpolation() 
{
  if( polygonIte_indices_.empty() ) return false;

  unordered_map<Eigen::Index, vector<Vector2<Eigen::Index>>> rowToIndicesMap, colToIndicesMap;
  for ( const auto& it : polygonIte_indices_ ) {
    rowToIndicesMap[it[0]].push_back( it );
    colToIndicesMap[it[1]].push_back( it );
  }

  for ( const auto& it : polygonIte_indices_ ) {
    MapIndex index = { it[0], it[1] };
    
    if( isnan( submap_->getValueAt( index ) ) ) {
      Scalar nan_bilinear = numeric_limits<Scalar>::quiet_NaN();
      ROS_INFO_STREAM_NAMED( "MapbagEditorServer",
                      "interpolation: " << it[0] << "y: "<< it[1] );

      auto left_result = findNearestNonNaN( rowToIndicesMap[it[0]], { index.row, index.col - 1 }, 0, -1 );
      auto right_result = findNearestNonNaN( rowToIndicesMap[it[0]], { index.row, index.col + 1 }, 0, 1 );
      auto lower_result = findNearestNonNaN( colToIndicesMap[it[1]], { index.row - 1, index.col }, -1, 0 );
      auto upper_result = findNearestNonNaN( colToIndicesMap[it[1]], { index.row + 1, index.col }, 1, 0 );

      bool lower_found = lower_result.second;
      bool upper_found = upper_result.second;
      bool left_found = left_result.second;
      bool right_found = right_result.second;

      Scalar lower_value = submap_->getValueAt( lower_result.first );
      Scalar upper_value = submap_->getValueAt( upper_result.first );
      Scalar left_value = submap_->getValueAt( left_result.first );
      Scalar right_value = submap_->getValueAt( right_result.first );
      
      if ( lower_found && upper_found && left_found && right_found ) { 
        Scalar weight_lower = std::abs( 1 / ( index.row - lower_result.first.row ) );
        Scalar weight_upper = std::abs( 1 / ( index.row - upper_result.first.row ) );
        Scalar weight_left = std::abs( 1 / ( index.col - left_result.first.col ) );
        Scalar weight_right = std::abs( 1 / ( index.col - right_result.first.col ) );
        nan_bilinear = ( weight_lower * lower_value + weight_upper * upper_value + weight_left * left_value + weight_right * right_value ) 
                        / ( weight_lower + weight_upper + weight_left + weight_right );
                        // 0.5 * ( lower_value + ( upper_value - lower_value ) 
                        // * ( index.row - lower_result.first.row ) / ( upper_result.first.row - lower_result.first.row ))
                        // + 0.5 * ( left_value + ( right_value - left_value ) 
                        // * ( index.col - left_result.first.col ) / ( upper_result.first.col - right_result.first.col ));            
      } else if (lower_found && upper_found) {
        nan_bilinear = ( index.row * ( upper_value - lower_value ) + lower_value * upper_result.first.row - upper_value * lower_result.first.row )
                        / ( upper_result.first.row - lower_result.first.row );
                      //  ( lower_value + ( upper_value - lower_value ) 
                      //   * ( index.row - lower_result.first.row ) / ( upper_result.first.row - lower_result.first.row ));
      } else if (left_found && right_found) {
        nan_bilinear = ( left_value + ( right_value - left_value ) 
                        * ( index.col - left_result.first.col ) / ( upper_result.first.col - right_result.first.col ));
      } 
      submap_->map_( index.row, index.col ) = nan_bilinear;
    }
  } 
  submap_->finishUpdate();
  publishMapbagInformation();
  return true;
}


template<typename Scalar>
void MapbagEditorServer<Scalar>::saveSubmapCallback( const grid_map_msgs::GridMap &msg )
{
  typename hector_world_heightmap::Heightmap<Scalar>::Ptr map = msgToHeightmap<Scalar>( msg, "elevation" );
  if( map == nullptr ) return;
  else {
    saveModeChange( int( 1 ));
    Command* command = new EditMapCommand( this, map, int( 1 ));
    invoker_->execute( command );
  }
}


template<typename Scalar>
void MapbagEditorServer<Scalar>::saveVerschiebenCallback( const grid_map_msgs::GridMap &msg )
{
  typename hector_world_heightmap::Heightmap<Scalar>::Ptr map = msgToHeightmap<Scalar>( msg, "elevation" );
  if( map == nullptr ) return;
  else {
    typename hector_world_heightmap::Map<Scalar>::Ptr submap = mapbag_->getMap( 0 );
    hector_world_heightmap::Heightmap<float>::Ptr mapbag_backup = std::make_shared<Heightmap<float>>(
                                                                    submap->map(), submap->resolution(), submap->origin(), 
                                                                    submap->frame(), static_cast<long long>( ros::Time::now().toNSec() ) ); 
    invoker_backup( mapbag_backup );
    saveModeChange( int( 3 ));
    Command* command = new EditMapCommand( this, map, int( 3 ));
    invoker_->execute( command );
  }
}


template<typename Scalar>
void MapbagEditorServer<Scalar>::mapIntegrate( const std::shared_ptr<HeightmapRef<float>>& map, const int& mode )
{
  if( mode == 0 ) return;
  if( mapbag_ == nullptr ) return;

  if( map != nullptr ) 
  {
    hector_world_heightmap::integrators::HeightmapIntegrator<Scalar> heightmapIntegrator( mapbag_ );
    if( mode == 1 || mode == 2 ) {
      heightmapIntegrator.integrate( map, integrators::IntegratorMode::All );
    } else if( mode == 3 ) {
      heightmapIntegrator.integrate( map, integrators::IntegratorMode::SourceKnown );
    }
    publishMapbagInformation();
  }
}


template<typename Scalar>
bool MapbagEditorServer<Scalar>::onClearMapbag( hector_std_msgs::StringServiceRequest &,
                                               hector_std_msgs::StringServiceResponse & )
{ return clearMapbag(); }


template<typename Scalar>
bool MapbagEditorServer<Scalar>::clearMapbag() 
{
  const Scalar NaN = std::numeric_limits<Scalar>::quiet_NaN();

  //muss noch weiter optimiert werden. Hier wird nur der Fall einer einzelnen Karte in Mapbag behandelt 
  if( mapbag_ != nullptr ) {
    size_t index = mapbag_->numberOfMaps();
    if( index != 0 ) {
      mapbag_->removeMap( 0 );
      hector_math::GridMap<float> map( 2, 2 );
      map << NaN, NaN,
            NaN, NaN;
      mapbag_->addMap( std::make_shared<Map<float>>
                          ( map, Vector3<float>::Zero(), mapbag_->resolution(), 0, 0 ));
      
      whm_ = make_shared<MapBag<Scalar>>( resolution_ );
      whm_->setFrame( world_frame_ );
      pub_submap_ref_.publish( heightmapToMsg<Scalar>( mapbag_->getMap( 0 )));
      pub_submap_.publish( heightmapToMsg<Scalar>( mapbag_->getMap( 0 )));

      publishMapbagInformation();
      invoker_->reset();
      saveModeChange( int( 0 ));
    }
  } 
  return true;
}


template<typename Scalar>
bool MapbagEditorServer<Scalar>::onUndo( hector_std_msgs::StringServiceRequest &,
                                         hector_std_msgs::StringServiceResponse & )
{ return undo(); }


template<typename Scalar>
bool MapbagEditorServer<Scalar>::undo() 
{
  invoker_->undo();
  return true;
}


template<typename Scalar>
bool MapbagEditorServer<Scalar>::onRedo( hector_std_msgs::StringServiceRequest &,
                                         hector_std_msgs::StringServiceResponse & )
{ return redo(); }


template<typename Scalar>
bool MapbagEditorServer<Scalar>::redo() 
{
  invoker_->redo();
  return true;
}


template<typename Scalar>
bool MapbagEditorServer<Scalar>::onSaveModeChange( hector_std_msgs::StringServiceRequest &req,
                                                   hector_std_msgs::StringServiceResponse & )
{ 
  try {
    int mode = std::stoi( req.param );
    return saveModeChange( mode );
  } catch ( const std::invalid_argument &e ) {
    ROS_INFO_STREAM_NAMED( "MapbagEditorServer", "Invalid input for function saveModeChange" );
    return false;
  }
}


template<typename Scalar>
bool MapbagEditorServer<Scalar>::saveModeChange( const int &mode ) 
{
  if( mode != 0 && mode != 1 && mode != 2 && mode != 3 ) return false;
  if( mode == 0 ){
    save_mode_ = SAVE_DEAKTIV;
  } else if ( mode == 1 ){
    save_mode_ = SAVE_NORMAL;
  } else if ( mode == 2 ){
    save_mode_ = SAVE_DELETE;
  } else if ( mode == 3 ){
    save_mode_ = SAVE_VERSCHIEBEN;
  } 
  return true;
}


template<typename Scalar>
bool MapbagEditorServer<Scalar>::onSettingsChange( mapbag_editor_msgs::SettingsRequest &req,
                                                   mapbag_editor_msgs::SettingsResponse & )
{ 
  if ( req.polygon_mode != polygon_mode_ ) {
    if ( req.polygon_mode == 0 ) polygon_mode_ = KONKAVHULL;
    else if (req.polygon_mode == 1) polygon_mode_ = KONVEXHULL;
    else if (req.polygon_mode == 2) polygon_mode_ = ROOMWALLS;
    else {
      ROS_WARN_STREAM_NAMED("MapbagEditorServer", 
                          "Wrong Setting Configuration : Polygon Selection");
    }
    last_confirmed_points_.clear();
  }
  if( req.intepolation_mode != intepolation_mode_ ) intepolation_mode_ = req.intepolation_mode;
  if( req.smooth_mode != smooth_mode_ ) smooth_mode_ = req.smooth_mode;
  // ROS_INFO_STREAM_NAMED( "MapbagEditorServer", "polygon_mode_: " << polygon_mode_ << ",intepolation_mode_: " 
  //                                               << intepolation_mode_ << ",smooth_mode_: " << smooth_mode_ );
  return true;
}

} //namespace mapbag_editor_server

#endif //MAPBAG_EDITOR_SERVER_MAPBAG_EDITOR_SERVER_IMPL_HPP