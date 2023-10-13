#ifndef MAPBAG_EDITOR_SERVER_GRAHAMSCAN_CH_EXECUTOR_IMPL_HPP
#define MAPBAG_EDITOR_SERVER_GRAHAMSCAN_CH_EXECUTOR_IMPL_HPP

#include "grahamscan_ch_executor.h"

namespace mapbag_editor_server
{
using namespace hector_math;

template<typename Scalar>
GrahamScan_CH_Executor<Scalar>::GrahamScan_CH_Executor( std::vector<hector_math::Vector3<Scalar>> &polygon_points )
    : ordered_polygon_points_( polygon_points )
{
    convexhull_found = false;
    ch_vertex_number = 0;
}


template <typename Scalar>
GrahamScan_CH_Executor<Scalar>::~GrahamScan_CH_Executor() {}


template<typename Scalar>
bool GrahamScan_CH_Executor<Scalar>::comparepoints ( hector_math::Vector3<Scalar> p1, hector_math::Vector3<Scalar> p2 ) {
    if (p2[1] != p1[1])
        return p2[1] < p1[1];
    return p2[0] > p1[0];
}


// returns -1 if p2 is on the left side of line p0p1,
// +1 for right side of line p0p1
template<typename Scalar>
int GrahamScan_CH_Executor<Scalar>::orien_comp( hector_math::Vector3<Scalar> p0, hector_math::Vector3<Scalar> p1, hector_math::Vector3<Scalar> p2 ) {
    Scalar val = (p2[0] - p0[0]) * (p1[1] - p0[1]) 
              - (p2[1] - p0[1]) * (p1[0] - p0[0]);
    if (val == 0) return 0;
    return (val > 0) ? 1 : -1; 
}


template<typename Scalar>
Scalar square( hector_math::Vector3<Scalar> p1, hector_math::Vector3<Scalar> p2 )
{
    Scalar dx = p2[0] - p1[0];
    Scalar dy = p2[1] - p1[1];
    return dx * dx + dy * dy;
}


// true: p1在p2之前
// false: p1在p2之后
template<typename Scalar>
bool GrahamScan_CH_Executor<Scalar>::polar_order( hector_math::Vector3<Scalar> p1, hector_math::Vector3<Scalar> p2 )    
{
    int order = orien_comp(p0, p1, p2);
    if (order == 0)
        return square(p0, p1) < square(p0, p2);
    return (order == -1);
}


template<typename Scalar>
bool GrahamScan_CH_Executor<Scalar>::findP0() 
{
    int min_index = 0;
    p0 = ordered_polygon_points_[min_index];
    int i = 1;
    for( it_ = ordered_polygon_points_.begin() + 1; it_ != ordered_polygon_points_.end(); it_++,i++ )    {
        Vector3<Scalar> element = *it_;
        if( comparepoints( p0, element )) {
            p0 = element;
            min_index = i;
        }
    }
    std::swap( ordered_polygon_points_[0], ordered_polygon_points_[min_index] );
    ch_vertex_number = 1;
    return true;    
}


template<typename Scalar>
bool GrahamScan_CH_Executor<Scalar>::grahamScan()
{
    findP0();
    std::sort( ordered_polygon_points_.begin() + 1, ordered_polygon_points_.end(),  [this] (const auto &p1, const auto &p2) { return polar_order(p1, p2); } );
    // ROS_INFO_STREAM_NAMED( "MapbagEditorServer",
    //                        "ordered_polygon_points_:" << " 0 : " << ordered_polygon_points_[0][0] << " 1 : " << ordered_polygon_points_[1][0] << " 2 : " << ordered_polygon_points_[2][0]
    //                        << " 3 : " << ordered_polygon_points_[3][0] << " 4 : " << ordered_polygon_points_[4][0]); 

    ch_points_ = ordered_polygon_points_;
    
    for ( long unsigned int i = 1; i < ch_points_.size(); i++ ) {
        while ( i < ch_points_.size() - 1 && orien_comp(p0, ch_points_[i], ch_points_[i + 1] ) == 0) {
            i++;
        }
        ch_points_[ch_vertex_number] = ch_points_[i];
        ch_vertex_number++;
    }

    if ( ch_vertex_number < 3 ) {
        return true;
    }

    convexHull_.push(ch_points_[0]);
    convexHull_.push(ch_points_[1]);
    convexHull_.push(ch_points_[2]);
    
    for ( int i = 3; i < ch_vertex_number; i++ ) {
        Vector3<Scalar> top = convexHull_.top();
        convexHull_.pop();
        while (convexHull_.size() > 1 && orien_comp(convexHull_.top(), top, ch_points_[i] ) != -1) {
            top = convexHull_.top();
            convexHull_.pop();
        }
        convexHull_.push( top );
        convexHull_.push( ch_points_[i] );
    }

    ch_points_.clear();
    while (!convexHull_.empty()) {
        ch_points_.push_back(convexHull_.top());
        convexHull_.pop();
    }
    // std::reverse(ch_points_.begin(), ch_points_.end()); 
    return true;
}

} //namespace mapbag_editor_server

#endif // MAPBAG_EDITOR_SERVER_GRAHAMSCAN_CH_EXECUTOR_IMPL_HPP