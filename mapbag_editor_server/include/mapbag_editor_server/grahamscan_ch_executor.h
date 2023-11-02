#ifndef MAPBAG_EDITOR_SERVER_GRAHAMSCAN_CH_EXECUTOR_H
#define MAPBAG_EDITOR_SERVER_GRAHAMSCAN_CH_EXECUTOR_H

#include <ros/ros.h>
#include <stack>
#include <hector_math/types.h>


namespace mapbag_editor_server
{

template<typename Scalar>
class GrahamScan_CH_Executor
{

public:
    typedef std::shared_ptr<GrahamScan_CH_Executor<Scalar>> Ptr;
    typedef std::shared_ptr<const GrahamScan_CH_Executor<Scalar>> ConstPtr;

    GrahamScan_CH_Executor( std::vector<hector_math::Vector3<Scalar>> &polygon_points );

    ~GrahamScan_CH_Executor();

    bool grahamScan();

    std::vector<hector_math::Vector3<Scalar>> ch_points() { return ch_points_; }

private:
    bool comparepoints( const hector_math::Vector3<Scalar>& p1, const hector_math::Vector3<Scalar>& p2 );

    int orien_comp( const hector_math::Vector3<Scalar>& p0, const hector_math::Vector3<Scalar>& p1, const hector_math::Vector3<Scalar>& p2 );

    bool polar_order( const hector_math::Vector3<Scalar>& p1, const hector_math::Vector3<Scalar>& p2 );

    bool findP0( );

    std::vector<hector_math::Vector3<Scalar>> ordered_polygon_points_;
    std::vector<hector_math::Vector3<Scalar>> ch_points_;
    std::stack<hector_math::Vector3<Scalar>> convexHull_;
    hector_math::Vector3<Scalar> p0;
    typename std::vector<hector_math::Vector3<Scalar>>::iterator it_;
    bool convexhull_found;
    int ch_vertex_number;

};

} //namespace mapbag_editor_server

#include "grahamscan_ch_executor_impl.hpp"

#endif // MAPBAG_EDITOR_SERVER_GRAHAMSCAN_CH_EXECUTOR_H
