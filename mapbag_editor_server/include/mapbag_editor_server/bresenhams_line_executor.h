#ifndef MAPBAG_EDITOR_SERVER_BRESENHAMS_LINE_EXECUTOR_H
#define MAPBAG_EDITOR_SERVER_BRESENHAMS_LINE_EXECUTOR_H

#include <ros/ros.h>
#include <Eigen/Core>
#include <hector_world_heightmap/math/index.h>

namespace mapbag_editor_server
{
using namespace hector_world_heightmap;

template<typename Scalar>
class Bresenhams_Line_Executor
{

public:
    typedef std::shared_ptr<Bresenhams_Line_Executor<Scalar>> Ptr;
    typedef std::shared_ptr<const Bresenhams_Line_Executor<Scalar>> ConstPtr;

    // Bresenhams_Line_Executor( hector_math::Vector2<Scalar> &start_point, hector_math::Vector2<Scalar> &end_point );
    Bresenhams_Line_Executor( MapBagIndex &index_start, MapBagIndex &index_end );

    ~Bresenhams_Line_Executor();

    bool bresenhams_line();

    std::vector<hector_math::Vector2<Scalar>> line_indices() { return line_indices_; }

    Scalar line_points_size() { return line_points_size_; }

private:
    void if_Swap_needed( Scalar &dx, Scalar &dy );

    hector_math::Vector2<Eigen::Index> start_point_;
    hector_math::Vector2<Eigen::Index> end_point_;
    std::vector<hector_math::Vector2<Scalar>> line_indices_;
    Scalar x_;
    Scalar y_;
    Scalar dx_;
    Scalar dy_;
    Scalar error_;
    Scalar sx_;
    Scalar sy_;
    bool isSwaped_;
    bool line_found_;
    Scalar line_points_size_;
    // int ch_vertex_number;

};

} //namespace mapbag_editor_server

#include "bresenhams_line_executor_impl.hpp"

#endif // MAPBAG_EDITOR_SERVER_BRESENHAMS_LINE_EXECUTOR_H
