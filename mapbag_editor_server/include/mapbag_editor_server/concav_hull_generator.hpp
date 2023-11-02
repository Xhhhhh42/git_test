#ifndef MAPBAG_EDITOR_SERVER_CONCAV_HULL_GENERATOR_HPP
#define MAPBAG_EDITOR_SERVER_CONCAV_HULL_GENERATOR_HPP

#include "concav_hull_generator.h"
#include "doLinesIntersect.h"
#include "server_math.h"
#include <ros/ros.h>

namespace mapbag_editor_server
{
using namespace hector_math;

template<typename Scalar>
Concav_Hull_Generator<Scalar>::Concav_Hull_Generator( std::vector<hector_math::Vector3<Scalar>> &polygon_points, int k )
    : original_points_( polygon_points ), k_( k )
{
    concavhull_found_ = false;
    kdtree_ = new KdTree<Scalar>( original_points_, 2 );
    kdtree_->createTree();

    if ( k < 3 ) { throw std::runtime_error( "k must be greater or equal to 3." ); }
}


template <typename Scalar>
Concav_Hull_Generator<Scalar>::~Concav_Hull_Generator() { delete kdtree_; }


template <typename Scalar>
std::vector<hector_math::Vector3<Scalar>> Concav_Hull_Generator<Scalar>::GetNearestNeighbors( const hector_math::Vector3<Scalar> &search_point, int &k )
{
    ROS_INFO_STREAM_NAMED("MapbagEditorServer", 
                              "GetNearestNeighbors" );
    std::vector<hector_math::Vector3<Scalar>> kneighbors = kdtree_->knearest_points( search_point, k );
    return kneighbors;
}


template <typename Scalar>
void Concav_Hull_Generator<Scalar>::removePoint( std::vector<hector_math::Vector3<Scalar>> &modified_points, 
                                                 const hector_math::Vector3<Scalar> &removed_point ) 
{
    // Variante 1 : Use std::remove_if to copy only the points that do not match the specified point
    std::copy_if( original_points_.begin(), original_points_.end(), std::back_inserter( modified_points ), 
        [&removed_point]( const hector_math::Vector3<Scalar> &p ) { return p[0] != removed_point[0] || p[1] != removed_point[1]; });

    // Variante 2
    // original_points_.erase( std::remove( original_points_.begin(), original_points_.end(), removed_point ), original_points_.end() );
}


template<typename Scalar>
bool Concav_Hull_Generator<Scalar>::concavHull( const std::vector<hector_math::Vector3<Scalar>> &original_points, const int &k )
{
    ROS_INFO_STREAM_NAMED("MapbagEditorServer", 
                              "concavHullstart: " );
    std::vector<hector_math::Vector3<Scalar>> points = original_points;
    ROS_INFO_STREAM_NAMED("MapbagEditorServer", 
                              "pointssize: " << points.size() );
    // 找到最低y值的点作为起始点
    auto min_y_point = *std::min_element( points.begin(), points.end(), []( const Vector3<Scalar>& p1, const Vector3<Scalar>& p2 ) {
                                                                            return p1[1] < p2[1]; });
    concav_points_.clear();
    concav_points_.push_back( min_y_point );
    removePoint( points, min_y_point );
    ROS_INFO_STREAM_NAMED("MapbagEditorServer", 
                              "pointssize: " << points.size() );
    Vector3<Scalar> current_point = min_y_point;

    int step = 2;
    int num_points = points.size();
    int lastPoint = 0;
    ROS_INFO_STREAM_NAMED("MapbagEditorServer", 
                              "concavHull1: " );

    while (( current_point != min_y_point || step == 2 ) && num_points > 0 ) 
    {
        // if ( step == 5 ) { points.push_back( min_y_point ); }

        // 找到当前点的k个最近邻点
        std::vector<Vector3<Scalar>> k_nearest_points = GetNearestNeighbors( current_point, k_ );
        // k_nearest_points.reserve( k_ );
        // for ( const Vector3<Scalar>& point : points ) {
        //     Scalar dist = distance( current_point, point );
        //     if ( k_nearest_points.size() < k_ ) {
        //         k_nearest_points.push_back( point );
        //     } else {
        //         auto max_dist_point = std::max_element( k_nearest_points.begin(), k_nearest_points.end(), 
        //                                                 [current_point](const Vector3<Scalar>& p1, const Vector3<Scalar>& p2 ) {
        //             return distance( current_point, p1 ) < distance( current_point, p2 );
        //         });
        //         if ( dist < distance(current_point, *max_dist_point )) {
        //             *max_dist_point = point;
        //         }
        //     }
        // }
        ROS_INFO_STREAM_NAMED("MapbagEditorServer", 
                              "concavHull2: " );

        // 按照角度排序k个最近邻点
        std::sort( k_nearest_points.begin(), k_nearest_points.end(), 
                   [current_point]( const Vector3<Scalar> &p1, const Vector3<Scalar> &p2 ) {
                        Scalar angle1 = std::atan2( p1[1] - current_point[1], p1[0] - current_point[0] );             
                        Scalar angle2 = std::atan2( p2[1] - current_point[1], p2[0] - current_point[0] ); 
                        return angle1 < angle2; });
        for ( size_t i = 0; i < k_nearest_points.size(); i++ ) 
        {
            ROS_INFO_STREAM_NAMED("MapbagEditorServer", 
                                "k_nearest_points: " << "x: " << k_nearest_points[i][0] << ", y: " << k_nearest_points[i][1] << ", z: " << k_nearest_points[i][2] );
        }

        // 选择第一个不与多边形边相交的点
        // avoid intersections: select first candidate that does not intersect any polygon edge
        bool intersection = true;
        int i = 0;
        while ( intersection && i < k_nearest_points.size() ) {
            // if ( k_nearest_points[i] == min_y_point ) { lastPoint = 1; }
            // else { lastPoint = 0; }
            int lastPoint = ( k_nearest_points[i] == min_y_point ) ? 1 : 0;

            intersection = false;
            int j = 2;
            while ( !intersection && j < concav_points_.size() - lastPoint ) {
                intersection = doLinesIntersect( concav_points_[step - 2], k_nearest_points[i], concav_points_[step - 1 - j], concav_points_[step - j - 2] );
                j++;
            }
            
            i++;
        }

        if ( intersection ) {
            return concavHull( original_points_, k_ + 1 );
        }

        current_point = k_nearest_points[i - 1];
        concav_points_.push_back( current_point );
        removePoint( points, current_point );
        
        // auto point_iter = std::find( original_points_.begin(), original_points_.end(), current_point );
        // if ( point_iter != original_points_.end() ) {
        //     original_points_.erase( point_iter );
        // }

        step++;
        num_points--;
    }

    // // 检查所有的点是否在凹多边形壳内
    // std::vector<bool> points_contained;
    // points_contained.reserve( original_points_.size() );
    // for ( const Vector3<Scalar> &point : original_points_) {
    //     points_contained.push_back( isPointInPolygon( concav_points_, point ));
    // }

    // if ( !std::all_of( points_contained.begin(), points_contained.end(), [](bool contained ) { return contained; })) {
    //     return concaveHull( original_points_, k_ + 1 );
    // }

    // // Check if all points are inside the hull
    // if ( !allPointsInsideHull( original_points_, concav_points_ )) {
    //     return concaveHull( original_points_, k_ + 1 );
    // }

    concavhull_found_ = true;
    return true;
}


template<typename Scalar>
bool Concav_Hull_Generator<Scalar>::allPointsInsideHull( const std::vector<hector_math::Vector3<Scalar>> &dataset, 
                                                         const std::vector<hector_math::Vector3<Scalar>>  &hull ) 
{
    // // Create a Path object from the hull points
    // Path p;
    // for (const Point& point : hull) {
    //     p.push_back(point);
    // }

    // // Initialize a boolean vector to store containment information for each point
    // std::vector<bool> pContained;

    // // Set a small radius for the containment check
    // double radius = 0.0000000001;

    // // Check if each point in the dataset is contained within the hull
    // for ( const auto& point : dataset ) {
    //     pContained.push_back( p.contains( point, radius ));
    // }

    // // Check if all points are contained inside the hull
    // return std::all_of( pContained.begin(), pContained.end(), [](bool contained) { return contained; });
    return true;
}

} //namespace mapbag_editor_server

#endif // MAPBAG_EDITOR_SERVER_CONCAV_HULL_GENERATOR_HPP