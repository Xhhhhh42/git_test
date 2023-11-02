/**
 * Created by Yuchen Xia on 01.11.23.
 * Check for intersections within the generated polygons ( 2D )
 */

#ifndef MAPBAG_EDITOR_SERVER_POLYGONCHECK_H
#define MAPBAG_EDITOR_SERVER_POLYGONCHECK_H

#include "intersect_segments.h"

#include <ros/ros.h>
#include <hector_math/types.h>
// #include <unordered_set>
#include <vector>
#include <queue>


namespace mapbag_editor_server
{
/**
 * Check for intersections within the generated polygons ( 2D )
 * @param polygon_points A vector containing 3D point indices representing the polygon.
 * @return true if there is no intersection in the polygon, otherwise false      
 */
template<typename Scalar>
inline bool polygonCheck( const std::vector<hector_math::Vector3<Scalar>> &polygon_points ) 
{
    struct Line_Segment {
        Line_Segment() = default;

        Line_Segment( const hector_math::Vector3<Scalar> &a, const hector_math::Vector3<Scalar> &b )
        {
            start_point = ( a[1] < b[1] ) ? b : a;
            end_point = ( a[1] < b[1] ) ? a : b;
        }

        hector_math::Vector3<Scalar> start_point;
        hector_math::Vector3<Scalar> end_point;
    };

    // std::unordered_set<hector_math::Vector3<Scalar>> points_set;
    std::priority_queue<Scalar> maxHeap_y_value;
    std::vector<Line_Segment> lines;
    std::vector<Line_Segment> active_lines;
    lines.reserve( polygon_points.size() );
    for ( const hector_math::Vector3<Scalar>& point : polygon_points) {
        // points_set.insert( point );
        maxHeap_y_value.push( point[1] );
    }

    // Build lines from points and obtain max y for the stopping criterion during the iteration loop
    for ( size_t i = 0; i < polygon_points.size() - 1; i++ ) {
        lines.emplace_back( polygon_points[i], polygon_points[i + 1] );
    }
    lines.emplace_back( polygon_points[polygon_points.size() - 1], polygon_points[0] );
    std::sort( lines.begin(), lines.end(),
               []( const Line_Segment &a, const Line_Segment &b ) { return a.start_point[1] > b.start_point[1]; } );

    while( !maxHeap_y_value.empty() ) {
        Scalar current_y = maxHeap_y_value.top();
        maxHeap_y_value.pop();
        for ( size_t active_line_index = 0; active_line_index < lines.size(); ++active_line_index ) {
            if ( lines[active_line_index].start_point[1] == current_y ) 
                { active_lines.push_back( lines[active_line_index] ); }
        }

        for ( size_t i = 0; i < active_lines.size(); ++i ) 
        {
            for ( size_t j = i + 1; j < active_lines.size(); ++j ) 
            {
                if ( active_lines[i].start_point == active_lines[j].start_point || active_lines[i].start_point == active_lines[j].end_point ||
                     active_lines[i].end_point == active_lines[j].end_point || active_lines[i].end_point == active_lines[j].start_point ) continue;
                else {
                    hector_math::Vector3<Scalar> i_null, i_eins;
                    if ( intersect2D_Segments( active_lines[i].start_point, active_lines[i].end_point, active_lines[j].start_point,
                                               active_lines[j].end_point, i_null, i_eins ) != 0 ) {
                        bool founded = false; 
                        for( const auto &element : polygon_points ) {
                            if( i_null == element ) founded = true;
                        }
                        if( !founded ) {
                            ROS_INFO_STREAM_NAMED( "MapbagEditorServer", 
                                "Concavhull false" << i_null[0] << "y:" << i_null[1] << "z:" << i_null[2] );
                            return false;
                        } 
                    }
                }
            }
        }

        for ( size_t active_line_index = 0; active_line_index < active_lines.size(); ++active_line_index ) {
            if ( active_lines[active_line_index].end_point[1] == current_y ) 
                { active_lines.erase( active_lines.begin() + active_line_index ); }
        }
    }
    return true;
}

} // namespace mapbag_editor_server

#endif // MAPBAG_EDITOR_SERVER_POLYGONCHECK_H



