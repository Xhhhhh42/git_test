#ifndef MAPBAG_EDITOR_SERVER_CUBIC_SPLINE_H
#define MAPBAG_EDITOR_SERVER_CUBIC_SPLINE_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>
#include <hector_math/types.h>
#include <ros/ros.h>

namespace mapbag_editor_server
{

using namespace std;

template<typename Scalar>
class Cubic_Spline
{
public:
    typedef std::shared_ptr<Cubic_Spline<Scalar>> Ptr;
    typedef std::shared_ptr<const Cubic_Spline<Scalar>> ConstPtr;

    Cubic_Spline( const std::vector<std::pair<Eigen::Index, Scalar>> &points_input );

    void cubicSpline_init( const std::vector<std::pair<Eigen::Index, Scalar>> &filtered_points );

    void cubicSpline_Insert();

    std::vector<std::pair<Eigen::Index, Scalar>> interpolated_points() const { return interpolated_points_; }

private:
    std::vector<std::pair<Eigen::Index, Scalar>> points_input_;
    std::vector<std::pair<Eigen::Index, Scalar>> filtered_points_;
    std::vector<std::pair<Eigen::Index, Scalar>> interpolate_points_;
    std::vector<std::pair<Eigen::Index, Scalar>> interpolated_points_;
	std::vector<std::vector<Scalar>> cubicspline_parameters_;
	std::vector<Scalar> x_differences_;
};

} //namespace qml_class

#include"cubic_spline_impl.hpp"

#endif // MAPBAG_EDITOR_SERVER_CUBIC_SPLINE_H