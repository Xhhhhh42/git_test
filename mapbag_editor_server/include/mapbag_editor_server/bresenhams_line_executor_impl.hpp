#ifndef MAPBAG_EDITOR_SERVER_BRESENHAMS_LINE_EXECUTOR_IMPL_HPP
#define MAPBAG_EDITOR_SERVER_BRESENHAMS_LINE_EXECUTOR_IMPL_HPP

#include "bresenhams_line_executor.h"

namespace mapbag_editor_server
{
using namespace hector_math;

template<typename Scalar>
Bresenhams_Line_Executor<Scalar>::Bresenhams_Line_Executor( MapBagIndex &index_start, MapBagIndex &index_end )
{
    start_point_[0] = index_start.index.row;
    start_point_[1] = index_start.index.col;
    end_point_[0] = index_end.index.row;
    end_point_[1] = index_end.index.col;

    line_found_ = false;
    isSwaped_ = false;
    line_points_size_ = 0;
}


template <typename Scalar>
Bresenhams_Line_Executor<Scalar>::~Bresenhams_Line_Executor() {}


template<typename Scalar>
bool Bresenhams_Line_Executor<Scalar>::bresenhams_line()
{
    dx_ = end_point_[0] - start_point_[0];
    dy_ = end_point_[1] - start_point_[1];
    sx_ = ( dx_ >= 0 ) ? 1 : (-1);
    sy_ = ( dy_ >= 0 ) ? 1 : (-1);
    x_ = start_point_[0];
    y_ = start_point_[1];
    if_Swap_needed( dx_, dy_);
    error_ = 2 * ( std::abs( dy_ )) - std::abs( dx_ );
    line_indices_.push_back( start_point_ );

    for( Eigen::Index i = 0; i < std::abs( dx_ ); i++ ) {
        if( error_ < 0 ) {
            if( isSwaped_ != true ){
                x_ = x_ + sx_;
                line_indices_.push_back( { x_, y_ } );
            } else {
                y_ = y_ + sy_;
                line_indices_.push_back( { x_, y_ } );
            }
            error_ = error_ + 2 * std::abs( dy_ );
        } else {
            x_ = x_ + sx_;
            y_ = y_ + sy_;
            line_indices_.push_back( { x_, y_ } );
            error_ = error_ + 2 * std::abs( dy_ ) - 2 * std::abs( dx_ );
        }
    }
    line_points_size_ = line_indices_.size();
    return true;
}


template<typename Scalar>
void Bresenhams_Line_Executor<Scalar>::if_Swap_needed( Scalar &dx, Scalar &dy )
{
    if( std::abs(dy) > std::abs(dx) )  {
        Scalar temp = dx;
        dx = dy;
        dy = temp;
        isSwaped_ = true;
    }
}

} //namespace mapbag_editor_server

#endif // MAPBAG_EDITOR_SERVER_BRESENHAMS_LINE_EXECUTOR_IMPL_HPP