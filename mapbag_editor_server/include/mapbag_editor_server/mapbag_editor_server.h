#ifndef MAPBAG_EDITOR_SERVER_MAPBAG_EDITOR_SERVER_H
#define MAPBAG_EDITOR_SERVER_MAPBAG_EDITOR_SERVER_H

#include "grahamscan_ch_executor.h"
#include "bresenhams_line_executor.h"
#include "cubic_spline.h"
#include "concav_hull_generator.h"

#include <ros/ros.h>
#include <hector_std_msgs/StringService.h>
#include <hector_world_heightmap/world_heightmap.h>
#include <hector_world_heightmap_ros/integrators/tf2_heightmap_integrator.h>
#include <hector_world_heightmap_ros/map_bag_publisher.h>

#include <grid_map_msgs/GridMap.h>
#include <mapbag_editor_msgs/Polygon.h>
#include <mapbag_editor_msgs/Submap.h>
#include <mapbag_editor_msgs/Settings.h>

#include <hector_math/types.h>

namespace mapbag_editor_server
{

class Invoker;


template<typename Scalar>
class MapbagEditorServer 
{
enum SaveMode {
    SAVE_DEAKTIV,
    SAVE_NORMAL,
    SAVE_DELETE,
    SAVE_VERSCHIEBEN,
};

enum PolygonMode {
    KONKAVHULL,
    KONVEXHULL,
    ROOMWALLS,
};

public:
    typedef std::shared_ptr<MapbagEditorServer<Scalar>> Ptr;
    typedef std::shared_ptr<const MapbagEditorServer<Scalar>> ConstPtr;

    explicit MapbagEditorServer( const ros::NodeHandle &nh, const ros::NodeHandle &pnh );

    bool saveMap( const std::string &path );

    bool loadMap( const std::string &path );

    bool polygonGridMap( const std::vector<hector_math::Vector3<Scalar>> &polygon_points );

    void publishMapbagInformation();

    void publishSubmapInformation( const size_t &points );

    std::pair<hector_world_heightmap::MapIndex, bool> findNearestNonNaN( 
                                  std::vector<hector_math::Vector2<Eigen::Index>> &indicesMap, 
                                  const hector_world_heightmap::MapIndex& start_index, int row_step, int col_step );

    bool interpolation();

    void mapIntegrate( const std::shared_ptr<HeightmapRef<float>>& map, const int& mode );

    bool clearMapbag();

    bool undo();

    bool redo();

    bool saveModeChange( const int &mode );

private:
    void initForNewMap();

    void updatePolygonPoints( const std::vector<hector_math::Vector3<Scalar>> &poly_points );
    
    void updateMultipointsIndices( const std::vector<hector_math::Vector3<Scalar>> &multipoints );
    
    void polygonSubmap( const std::vector<hector_math::Vector3<Scalar>> &polygon_points );

    void index_line_op( hector_world_heightmap::MapBagIndex &index_start, const hector_math::Vector3<Scalar> &polygon_point );

    void index_line_generate( const hector_math::Vector3<Scalar> &point_start, const hector_math::Vector3<Scalar> &point_end, 
                              std::vector<hector_math::Vector2<Eigen::Index>> &indices );

    void saveSubmapCallback( const grid_map_msgs::GridMap &msg );

    void saveVerschiebenCallback( const grid_map_msgs::GridMap &msg );

    void deleteSubmapCallback( const grid_map_msgs::GridMap &msg );

    void invoker_backup( const std::shared_ptr<HeightmapRef<float>>& map );

    bool onSaveMap( hector_std_msgs::StringServiceRequest &req,
                    hector_std_msgs::StringServiceResponse &resp );

    bool onLoadMap( hector_std_msgs::StringServiceRequest &req,
                    hector_std_msgs::StringServiceResponse &resp );

    bool onPolygonGridMap( mapbag_editor_msgs::SubmapRequest &req,
                           mapbag_editor_msgs::SubmapResponse &resp );

    bool onInterpolation( hector_std_msgs::StringServiceRequest &req,
                          hector_std_msgs::StringServiceResponse &resp );

    bool onClearMapbag( hector_std_msgs::StringServiceRequest &req,
                        hector_std_msgs::StringServiceResponse &resp );

    bool onUndo( hector_std_msgs::StringServiceRequest &req,
                 hector_std_msgs::StringServiceResponse &resp );
    
    bool onRedo( hector_std_msgs::StringServiceRequest &req,
                 hector_std_msgs::StringServiceResponse &resp );

    bool onSaveModeChange( hector_std_msgs::StringServiceRequest &req,
                           hector_std_msgs::StringServiceResponse &resp );

    bool onSettingsChange( mapbag_editor_msgs::SettingsRequest &req,
                           mapbag_editor_msgs::SettingsResponse &resp );

    //ROS 
    ros::NodeHandle nh_, pnh_;

    ros::ServiceServer save_service_, load_service_;
    ros::ServiceServer polygongridmap_service_;
    ros::ServiceServer interpolation_service_; 
    ros::ServiceServer savesubmap_service_;
    ros::ServiceServer clearmapbag_service_; 
    ros::ServiceServer undo_service_, redo_service_; 
    ros::ServiceServer save_mode_service_;
    ros::ServiceServer system_settings_service_;
    std::vector<ros::Publisher> map_publishers_;
    ros::Publisher pub_resolution_, pub_frame_;
    ros::Publisher pub_submap_, pub_submap_ref_, pub_submap_pos_;
    ros::Subscriber changedsubmap_sub_, verschiebsubmap_sub_;

    //Mapbag
    std::string map_url_;

    std::vector<hector_math::Vector3<Scalar>> last_confirmed_points_;
    std::vector<hector_math::Vector3<Scalar>> polygon_points_;

    typename hector_world_heightmap::MapBag<Scalar>::Ptr mapbag_;
    typename hector_world_heightmap::Map<Scalar>::Ptr submap_;
    typename hector_world_heightmap::MapBag<Scalar>::Ptr whm_;
    
    Scalar resolution_;
    std::string world_frame_;
    Eigen::Index submap_row_min_, submap_row_max_;
    Eigen::Index submap_col_min_, submap_col_max_;
    hector_math::Vector2<Scalar> submap_origin_;
    hector_math::Vector2<Eigen::Index> submap_size_;

    std::vector<hector_math::GridMap<Scalar>> sub_submaps_;
    std::vector<hector_math::Vector2<Scalar>> submap_locations_;
    std::shared_ptr<GrahamScan_CH_Executor<Scalar>> gs_ch_executor_;
    std::shared_ptr<Bresenhams_Line_Executor<Eigen::Index>> bh_line_executor_;
    std::shared_ptr<Concav_Hull_Generator<Scalar>> concav_hull_generator_;
    std::shared_ptr<Cubic_Spline<Scalar>> cubic_spline_;
    std::vector<hector_math::Vector2<Eigen::Index>> line_indices_;
    std::vector<hector_math::Vector2<Eigen::Index>> polygonIte_indices_;
    hector_math::Polygon<Eigen::Index> polygonpoints_indices_;

    //Server Parameters
    std::shared_ptr<Invoker> invoker_;

    bool poly_submap_confirmed_;
    // int invoker_first_save_;
    SaveMode save_mode_;
    PolygonMode polygon_mode_;
    int intepolation_mode_;
    int smooth_mode_; 
};
} //namespace mapbag_editor_server


namespace mapbag_editor_server {
using namespace std;
using namespace hector_world_heightmap;

class Command 
{
public:
    virtual ~Command() {}
    virtual void execute() = 0;
    virtual void undo() = 0;
    virtual void redo() = 0;
};


class EditMapCommand : public Command 
{
public:
    explicit EditMapCommand( MapbagEditorServer<float>* editor, const std::shared_ptr<HeightmapRef<float>>& map, const int& mode ) 
                        : editor_( editor ), map_( map ), save_mode_( mode ) {
        assert( editor_ );
        assert( map_ );
        assert( save_mode_ );
    }

    void execute() override {
        if( editor_ == nullptr || map_ == nullptr ) return;
        editor_->mapIntegrate( map_, save_mode_ );
    }

    void undo() override {
        if( editor_ == nullptr || map_ == nullptr ) return;
        editor_->mapIntegrate( map_, save_mode_ );
    }

    void redo() override {
        if( editor_ == nullptr || map_ == nullptr ) return;
        editor_->mapIntegrate( map_, save_mode_ );
    }

private:
    MapbagEditorServer<float>* editor_;
    const std::shared_ptr<HeightmapRef<float>> map_;
    int save_mode_;
};


/**
 * The Invoker is associated with one or several commands. It sends a request to
 * the command.
 */
class Invoker 
{
public:
    explicit Invoker() {}

    ~Invoker() {
        reset();
    }

    void reset() {
        std::set<Command*> uniquePointers;

        while (!history_.empty()) {
            Command* command = history_.top();
            history_.pop();
            uniquePointers.insert( command );  
        }

        while (!history_backup_.empty()) {
            Command* command = history_backup_.top();
            history_backup_.pop();
            uniquePointers.insert( command );  
        }

        while (!redo_backup_.empty()) {
            Command* command = redo_backup_.top();
            redo_backup_.pop();
            uniquePointers.insert( command );  
        }

        while (!redo_list_.empty()) {
            Command* command = redo_list_.top();
            redo_list_.pop();
            uniquePointers.insert( command );  
        }

        // 现在uniquePointers中包含了所有不重复的指针
        // 清空uniquePointers中的指针，同时确保每个指针只被释放一次
        for ( Command* ptr : uniquePointers ) {
            delete ptr;
        }
    }

    void stack_clear() {
        std::set<Command*> uniquePointers;

        while ( !history_backup_.empty() ) {
            Command* command = history_backup_.top();
            history_backup_.pop();
            uniquePointers.insert( command );
        }
        while (!redo_list_.empty()) {
            Command* command = redo_list_.top();
            redo_list_.pop();
            uniquePointers.insert( command );  
        }
        for ( Command* ptr : uniquePointers ) {
            delete ptr;
        }
    }

    void execute( Command* command ) {
        command->execute();
        history_.push( command );
        redo_backup_.push( command );
        stack_clear();
    }

    void redo() {
        if ( !redo_list_.empty() ) {
            Command* command = redo_list_.top();
            redo_list_.pop();
            command->redo();
            redo_backup_.push( command );
        } 
        if ( !history_backup_.empty() ) {
            Command* command = history_backup_.top();
            history_.push( command );
            history_backup_.pop();
        }
    }

    void undo() {
        if ( history_.size() > 1 ) {
            Command* command = history_.top();
            history_backup_.push( command );
            history_.pop();
            command = history_.top();
            command->undo();
        } 
        if ( !redo_backup_.empty() ) {
            Command* redo_command = redo_backup_.top();
            redo_backup_.pop();
            redo_list_.push( redo_command );
        } 
    }

    void check_und_delete() {
        if( history_.size() > 1 ) { history_.pop(); }
    }

    void delete_backup() {
        if( redo_backup_.size() >= 1 ) { redo_backup_.pop(); }
    }

    int undo_size() { return history_.size(); }

    int redo_size() { return redo_list_.size(); }

private:
    std::stack<Command*> history_;
    std::stack<Command*> history_backup_;
    std::stack<Command*> redo_backup_;
    std::stack<Command*> redo_list_;
};
} //namespace mapbag_editor_server

#include "mapbag_editor_server_impl.hpp"

#endif //MAPBAG_EDITOR_SERVER_MAPBAG_EDITOR_SERVER_H