#ifndef MAPBAG_EDITOR_USER_INTERFACE_POLYGON_TOOL_H
#define MAPBAG_EDITOR_USER_INTERFACE_POLYGON_TOOL_H

#include <ros/ros.h>
#include <rviz/default_plugin/tools/interaction_tool.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Vector3.h>
#include <grid_map_msgs/GridMap.h>

#include <hector_math/types.h>
#include <hector_world_heightmap/map_bag.h>
#include <hector_world_heightmap_ros/integrators/tf2_heightmap_integrator.h>
#include <hector_world_heightmap/integrators/heightmap_integrator.h>

#include <QQuaternion>
#include <QVariantMap>
#include <OgreSharedPtr.h>

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

namespace Ogre
{
    class SceneNode;
    class Vector3;
}


namespace mapbag_editor_user_interface
{

class PolygonTool : public rviz::InteractionTool
{
Q_OBJECT
    Q_PROPERTY( QVariantList polygonpoints READ polygonpoints NOTIFY polygonpointsChanged )
    Q_PROPERTY( QString frame READ frame )
    Q_PROPERTY( QVector3D tempo READ tempo NOTIFY tempoChanged )

enum EditorMode {
    MODE_POLYGON,
    MODE_EDITOR,
    MODE_PRIMITIVE_ELEMENT,
    MODE_VERSCHIEBEN,
};

enum SelectionMode {
    Sequentiell,
    Konvexhull
};

enum AutomatischMode {
    Aktivieren,
    Deaktivieren
};

public:

    PolygonTool();

    ~PolygonTool();

    void activate() override;

    void deactivate() override;

    int processMouseEvent( rviz::ViewportMouseEvent &event ) override;

    int processKeyEvent( QKeyEvent *event, rviz::RenderPanel *panel ) override;

    QVariantList polygonpoints() const;

    QString frame() const;

    QVector3D tempo() const;

    Q_INVOKABLE void clearPolygonpoints();

    Q_INVOKABLE void removePolygonpoint( int index );

    Q_INVOKABLE void addPolygonpoint( float x, float y, float z );

    Q_INVOKABLE void changeEditorMode( int editor_mode );

    Q_INVOKABLE void changeAutomatischMode( int automatisch_mode );

    Q_INVOKABLE void setMarker( int selected_Polygon_nummer );

    Q_INVOKABLE void setHeight( int mode, float height );

    Q_INVOKABLE void dropChanges();

    Q_INVOKABLE void deleteSubmap();

    Q_INVOKABLE void clearmap();

    Q_INVOKABLE void publishToServer();

    Q_INVOKABLE void submapVerschiebenClear();

    Q_INVOKABLE void submapVerschiebenSave();

    Q_INVOKABLE void primitiveSave(); 

    Q_INVOKABLE void primitive( std::vector<int> mode, std::vector<double> center, std::vector<int> size );

    Q_INVOKABLE void primitiveClear(); 

    void publishToolmapInformation( const typename hector_world_heightmap::HeightmapRef<float>::ConstPtr &map );

signals:

    void polygonpointsChanged();

    void polygonpointRemoved( int index, QVariantMap polygonpoint );

    void polygonpointAdded( QVariantMap polygonpoint );

    void tempoChanged();

protected:

    void onInitialize() override;

    Ogre::SceneNode *createPolygonpointNode();

    void destroyPolygonpointNode( Ogre::SceneNode *node );

    Ogre::SceneNode *createPolygonpointNode( const Ogre::Vector3 &position );

    void mouseEventhelper( rviz::ViewportMouseEvent &event, typename hector_world_heightmap::Heightmap<float>::Ptr &map );

    void resolutionCallback( const std_msgs::Float64 &resulotion );

    void frameCallback( const std_msgs::String &frame );

    void submapSubCallback( const grid_map_msgs::GridMap &msg );

    void submapRefSubCallback( const grid_map_msgs::GridMap &ref_msg );

    void newCenterSubCallback( const geometry_msgs::Point &center );

    void publishEmpty();

    Ogre::MeshPtr preview_mesh_;
    Ogre::MeshPtr marker_mesh_;

    Ogre::SceneNode *polygonpoint_preview_node_ = nullptr;
    Ogre::SceneNode *polygonpoint_active_node_ = nullptr;
    
    std::vector<Ogre::SceneNode *> polygonpoint_nodes_;
    
    QVariantList polygonpoints_;
    QVector3D tempo_;
    QVector3D tempo_ref_;
    // The polygonpoint that is currently placed. Nullptr if not currently in placing state.
    rviz::BoolProperty *polygonpoint_3d_proporty_ = nullptr;

    EditorMode editor_mode_;
    AutomatischMode automatisch_mode_;

private: 
    void submapSubscribe( const grid_map_msgs::GridMap &msg, hector_world_heightmap::Heightmap<float>::Ptr &map );

    ros::NodeHandle nh_;
    ros::Publisher mapToServer_pub_, mapInTool_pub_, mapVerschieben_pub_;
    ros::Subscriber resolution_sub_, frame_sub_;
    ros::Subscriber submap_sub_, submap_ref_sub_, newcenter_sub_;

    
    float resolution_;
    float skalenfaktor_;
    bool verschieben_func_called_;
    std::string world_frame_;
    hector_math::Vector3<float> last_center_;

    Ogre::Real depth_;

    Eigen::ArrayXXf map_original_;
    hector_math::GridMap<float> submap_in_tool_;
    hector_math::GridMap<float> submap_ref_in_tool_;
    typename hector_world_heightmap::Heightmap<float>::Ptr map_;
    typename hector_world_heightmap::Heightmap<float>::Ptr map_ref_;
    typename hector_world_heightmap::Heightmap<float>::Ptr mapref_copy_;
    typename hector_world_heightmap::MapBag<float>::Ptr whm_;
};

} //namespace mapbag_editor_user_interface

#endif //MAPBAG_EDITOR_USER_INTERFACE_POLYGON_TOOL_H