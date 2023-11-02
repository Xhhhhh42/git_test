/**
 * Created by Yuchen Xia on 01.11.23.
 * 
 */

#ifndef MAPBAG_EDITOR_SERVER_SERVER_MATH_H
#define MAPBAG_EDITOR_SERVER_SERVER_MATH_H

#include <hector_math/types.h>

namespace mapbag_editor_server
{

/**
 * 计算两个点的叉积
 * @param a 第一个点
 * @param b 第二个点
 * @return 叉积的值
 */
template<typename Scalar>
inline Scalar crossProduct( const hector_math::Vector3<Scalar> &a, const hector_math::Vector3<Scalar> &b ) {
    return a[0] * b[1] - b[0] * a[1];
}


/**
 * 计算两个点的点积
 * @param a 第一个点
 * @param b 第二个点
 * @return 叉积的值
 */
template<typename Scalar>
inline Scalar dotProduct( const hector_math::Vector3<Scalar> &a, const hector_math::Vector3<Scalar> &b ) {
    return a[0] * b[0] + a[1] * b[1];
}


template <typename Scalar>
inline Scalar distance( const hector_math::Vector3<Scalar>& p1, const hector_math::Vector3<Scalar>& p2 ) {
    Scalar dx = p1[0] - p2[0];
    Scalar dy = p1[1] - p2[1];
    return std::sqrt( dx * dx + dy * dy );
}

} // namespace mapbag_editor_server

#endif // MAPBAG_EDITOR_SERVER_SERVER_MATH_H



