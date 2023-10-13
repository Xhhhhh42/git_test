#ifndef MAPBAG_EDITOR_SERVER_CUBIC_SPLINE_IMPL_HPP
#define MAPBAG_EDITOR_SERVER_CUBIC_SPLINE_IMPL_HPP

#include "cubic_spline.h"

#include <Eigen/Core>
#include <Eigen/Dense>

namespace mapbag_editor_server
{

/*！
 * 三次样条函数 Si(x)
 * 三次方程可以构造成如下形式：y = ai + bi * x + ci * x^2 + di * x^3
 * 自然边界 ( Natural Spline )：指定端点二阶导数为0
 * ai = yi
 * hi = xi+1 - xi 表示步长
 * bi+1 = bi + 2 * hi * ci + 3 * hi^2 * di
 * ci+1 = ci + 3 * hi * di
 * 设mi = Si''(xi) = 2 * ci, 可得: di = ( mi+1 - mi ) / ( 6 * hi )
 * bi = ...
 * ci = mi / 2
 * 在每个子区间[ xi, xi+1 ]中，创建方程: gi( x ) = ai + bi * ( x - xi ) + ci * ( x - xi )^2 + di * ( x - xi )^3 
*/

using namespace std;

template<typename Scalar>
Cubic_Spline<Scalar>::Cubic_Spline( const std::vector<std::pair<Eigen::Index, Scalar>> &points_input )
    : points_input_( points_input )
{
    x_differences_.clear();
    cubicspline_parameters_.clear();
    for (const auto& point : points_input) {
        if ( !std::isnan( point.second ) ) { filtered_points_.push_back( point ); }
        else interpolate_points_.push_back( point );
    }
    sort( filtered_points_.begin(), filtered_points_.end(),
              []( const std::pair<Eigen::Index, Scalar> &a, const std::pair<Eigen::Index, Scalar> &b ) {
                  return a.first < b.first; } );
    sort( interpolate_points_.begin(), interpolate_points_.end(),
              []( const std::pair<Eigen::Index, Scalar> &a, const std::pair<Eigen::Index, Scalar> &b ) {
                  return a.first < b.first; } );
    // ROS_INFO_STREAM_NAMED("MapbagEditorServer", 
    //                       "test: " <<  filtered_points_.size() );
    // ROS_INFO_STREAM_NAMED("MapbagEditorServer", 
    //                       "test: " <<  interpolate_points_.size() );
    cubicSpline_init( filtered_points_ );
    cubicSpline_Insert();
}


template<typename Scalar>
void Cubic_Spline<Scalar>::cubicSpline_init( const std::vector<std::pair<Eigen::Index, Scalar>> &filtered_points ) 
{
    using Matrix_dy = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;
    size_t p_size = filtered_points.size();
    if( p_size <= 1 ) return;

    x_differences_.reserve( p_size - 1 );
    Matrix_dy A_coeff ( p_size, p_size );
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> B_coeff ( p_size );
    Matrix_dy M;
    vector<Scalar> mV( p_size );
    vector<Scalar> h_coeff( p_size );

    for ( size_t i = 0; i < p_size - 1; i++ )
    {
        Eigen::Index x_d = abs( filtered_points[ i + 1 ].first - filtered_points[i].first );
        x_differences_.emplace_back(x_d);
    }

    //compute h coefficient
    for ( size_t j = 0; j < h_coeff.size(); j++ ) 
    {
        if ( j == 0 || j == h_coeff.size() - 1 ) { h_coeff[j] = 0; }
        else {
            h_coeff[j] = 6 * (( filtered_points[ j + 1 ].second - filtered_points[j].second ) / x_differences_[j] 
                        - ( filtered_points[j].second - filtered_points[ j - 1 ].second ) / x_differences_[ j - 1 ] );
        }
    }

    //init A, B
    B_coeff = Eigen::Map<Eigen::Matrix<Scalar, Eigen::Dynamic, 1>>( h_coeff.data(), p_size );
    A_coeff.row( 0 ).setZero();
    A_coeff.row( p_size - 1 ).setZero();
    A_coeff( 0, 0 ) = 1;
    A_coeff( p_size - 1, p_size - 1 ) = 1;

    for ( size_t i = 1; i < p_size - 1; i++ ) 
    {
        for ( size_t j = 0; j < p_size; j++ ) 
        {
            if ( j == i ) { A_coeff( i, j ) = 2 * ( h_coeff[ i - 1 ] + h_coeff[i] ); }
            else if ( j == i - 1 ) { A_coeff( i, j ) = h_coeff[ i - 1 ]; }
            else if ( j == i + 1 ) { A_coeff( i, j ) = h_coeff[i]; }
            else { A_coeff( i, j ) = 0; }
        }
    }

    M = A_coeff.llt().solve( B_coeff );
    mV.assign( M.col(0).data(), M.col(0).data() + M.rows() );
    for (size_t i = 0; i < mV.size(); i++ ) {
        // ROS_INFO_STREAM_NAMED("MapbagEditorServer", 
        //                   "cubic_spline: " << mV[i] );
    }

    for ( size_t i = 0; i < mV.size() - 1; i++ ) 
    {
        Scalar a = filtered_points[i].second;
        Scalar b = ( filtered_points[ i + 1 ].second - filtered_points[i].second ) / x_differences_[i] 
                    - x_differences_[i] / 2 * mV[i] - x_differences_[i] / 6 * ( mV[ i + 1 ] - mV[i] );
        Scalar c = mV[i] / 2;
        Scalar d = ( mV[ i + 1 ] - mV[i] ) / ( 6 * x_differences_[i] );
        vector<Scalar> cubicspline_parameter = { a, b, c, d };
        cubicspline_parameters_.push_back( cubicspline_parameter );

        // ROS_INFO_STREAM_NAMED("MapbagEditorServer", 
        //                   "parameters: a:" << a << "b: " << b << "c: " << c << "d: " << d );
    }
}


template<typename Scalar>
void Cubic_Spline<Scalar>::cubicSpline_Insert() 
{
    if( interpolate_points_.empty() ) return;
    size_t j = 0;
    for( size_t i = 0; i < filtered_points_.size() - 1; i++ ) 
    {
        std::vector<std::pair<Eigen::Index, Scalar>> temp;
        if( j >= interpolate_points_.size() ) break;

        for( ; j < interpolate_points_.size(); j++ ) 
        {
            if( interpolate_points_[j].first < filtered_points_[i].first || interpolate_points_[j].first > filtered_points_[ i + 1 ].first ) break;
            temp.push_back( interpolate_points_[j] );
        }

        if( temp.empty() ) continue;
        Scalar index_start = filtered_points_[i].first;
        Scalar a = cubicspline_parameters_[i][0];
        Scalar b = cubicspline_parameters_[i][1];
        Scalar c = cubicspline_parameters_[i][2];
        Scalar d = cubicspline_parameters_[i][3];

        for( size_t k = 0; k < temp.size(); k++ )
        {
            Scalar index = temp[k].first;
            temp[k].second = a + b * ( index - index_start )
                + c * std::pow( (index - index_start), 2 ) + d * std::pow( ( index - index_start ), 3 );
            // ROS_INFO_STREAM_NAMED("MapbagEditorServer", 
            //               "PROCESS: " <<  index << "VALUE: " << temp[k].second );    
        }
        // interpolated_points_ = std::move( temp );
        interpolated_points_.insert(interpolated_points_.end(), temp.begin(), temp.end());
        // ROS_INFO_STREAM_NAMED("MapbagEditorServer", 
        //                   "SIZE: " <<  interpolated_points_.size() );
    }
}

} //namespace mapbag_editor_server

#endif // MAPBAG_EDITOR_SERVER_CUBIC_SPLINE_IMPL_HPP