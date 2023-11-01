/**
 * Created by Yuchen Xia on 27.10.23.
 * determine if two line segments intersect ( 2D )
 * see: martin-thoma.com/how-to-check-if-two-line-segments-intersect/
 */

#ifndef MAPBAG_EDITOR_SERVER_DOLINESINTERSECT_H
#define MAPBAG_EDITOR_SERVER_DOLINESINTERSECT_H

#include <hector_math/types.h>

namespace mapbag_editor_server
{
static constexpr double EPSILON = 0.000001;

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
 * Check if bounding boxes do intersect. If one bounding box
 * touches the other, they do intersect.
 * @param a first bounding box
 * @param b second bounding box
 * @return true if they intersect,
 *         false otherwise.
 */
template<typename Scalar>
inline bool doBoundingBoxesIntersect( const hector_math::Vector3<Scalar> &a_lt, const hector_math::Vector3<Scalar> &a_rb,
                                      const hector_math::Vector3<Scalar> &b_lt, const hector_math::Vector3<Scalar> &b_rb ) 
{
    return a_lt[0] <= b_rb[0] && a_rb[0] >= b_lt[0] &&
           a_lt[1] <= b_rb[1] && a_rb[1] >= b_lt[1];
}


/**
 * Checks if a Point is on a line
 * @param a line (interpreted as line, although given as line segment)           
 * @param b point
 * @return true if point is on line, otherwise false      
 */
template<typename Scalar>
inline bool isPointOnLine( const hector_math::Vector3<Scalar> &a_lt, const hector_math::Vector3<Scalar> &a_rb, const hector_math::Vector3<Scalar> &b ) 
{
    // Move the line segment so that a.first is on (0, 0)
    hector_math::Vector3<Scalar> line_endpoint( a_rb[0] - a_lt[0], a_rb[1] - a_lt[1], 0  );
    hector_math::Vector3<Scalar> bTmp( b[0] - a_lt[0], b[1] - a_lt[1], 0  );
    double r = crossProduct( line_endpoint, bTmp );
    return std::abs( r ) < EPSILON;
}


/**
 * Checks if a point is right of a line. If the point is on the
 * line, it is not right of the line.
 * @param a line segment interpreted as a line
 * @param b the point
 * @return true if the point is right of the line,
 *         false otherwise
 */
template<typename Scalar>
inline bool isPointRightOfLine( const hector_math::Vector3<Scalar> &a_lt, const hector_math::Vector3<Scalar> &a_rb, const hector_math::Vector3<Scalar> &b ) 
{
    // 移动线段 a，使得 a_lt 在 (0,0) 处
    hector_math::Vector3<Scalar> line_endpoint( a_rb[0] - a_lt[0], a_rb[1] - a_lt[1], 0 );
    hector_math::Vector3<Scalar> bTmp( b[0] - a_lt[0], b[1] - a_lt[1], 0 );
    return crossProduct( line_endpoint, bTmp ) < 0;
}


/**
 * Check if line segment first touches or crosses the line that is
 * defined by line segment second.
 *
 * @param a_lt, a_rb line segment interpreted as line
 * @param b_lt, b_rb line segment
 * @return true if line segment first touches or crosses line second, false otherwise.        
 */
template<typename Scalar>
inline bool lineSegmentTouchesOrCrossesLine( const hector_math::Vector3<Scalar> &a_lt, const hector_math::Vector3<Scalar> &a_rb,
                                             const hector_math::Vector3<Scalar> &b_lt, const hector_math::Vector3<Scalar> &b_rb ) 
{
    return isPointOnLine( a_lt, a_rb, b_lt ) || isPointOnLine( a_lt, a_rb, b_rb )
           || ( isPointRightOfLine( a_lt, a_rb, b_lt ) != isPointRightOfLine( a_lt, a_rb, b_rb ));
}


/**
 * Check if line segments intersect
 * @param a first line segment
 * @param b second line segment
 * @return true if lines do intersect, false otherwise     
 */
template<typename Scalar>
inline bool doLinesIntersect( const hector_math::Vector3<Scalar> &a_lt, const hector_math::Vector3<Scalar> &a_rb,
                              const hector_math::Vector3<Scalar> &b_lt, const hector_math::Vector3<Scalar> &b_rb ) 
{
    return doBoundingBoxesIntersect( a_lt, a_rb, b_lt, b_rb ) && lineSegmentTouchesOrCrossesLine( a_lt, a_rb, b_lt, b_rb )
           && lineSegmentTouchesOrCrossesLine( a_lt, a_rb, b_lt, b_rb );
}


} // namespace mapbag_editor_server

#endif // MAPBAG_EDITOR_SERVER_DOLINESINTERSECT_H



