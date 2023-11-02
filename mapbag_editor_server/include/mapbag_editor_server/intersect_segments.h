/**
 * Created by Yuchen Xia on 01.11.23.
 * determine if two line segments intersect ( 2D ) and return point if the intersection point exists --- Version 1
 */

#ifndef MAPBAG_EDITOR_SERVER_INTERSECTSEGMENTS_H
#define MAPBAG_EDITOR_SERVER_INTERSECTSEGMENTS_H

#include <hector_math/types.h>

namespace mapbag_editor_server
{

static constexpr double SMALL_NUM = 0.00000001;


/**
 * 验证点是否在线段内
 * @param a point
 * @param b a line segment
 * @return true if point is on line, otherwise false 
 */
template<typename Scalar>
inline bool inSegment( const hector_math::Vector3<Scalar> &a,
                       const hector_math::Vector3<Scalar> &b_lt, const hector_math::Vector3<Scalar> &b_rb ) 
{
    if ( b_lt[0] != b_rb[0] ) {    // S is not vertical
        if (( b_lt[0] <= a[0] && a[0] <= b_rb[0] ) || ( b_lt[0] >= a[0] && a[0] >= b_rb[0] )) {
            return true;
        }
    } else {    // S is vertical, so test y coordinate
        if (( b_lt[1] <= a[1] && a[1] <= b_rb[1] ) || ( b_lt[1] >= a[1] && a[1] >= b_rb[1] )) {
            return true;
        }
    }
    return false;
}


/**
 * intersect2D_2Segments():2D有限线段相交
 * @param a a line segment           
 * @param b a line segment
 * @param i_null point intersection ( if exist )
 * @param i_eins Interval endpoint of intersecting line segments [i_null,i_eins] ( if exist )
 * @return 0 ： disjoint (no intersect)
 *         1 ： intersect in unique point i_null
 *         2 ： overlap in segment from i_null to i_eins  
 */
template<typename Scalar>
inline int intersect2D_Segments( const hector_math::Vector3<Scalar> &a_lt, const hector_math::Vector3<Scalar> &a_rb,
                                 const hector_math::Vector3<Scalar> &b_lt, const hector_math::Vector3<Scalar> &b_rb,
                                 hector_math::Vector3<Scalar> &i_null, hector_math::Vector3<Scalar> &i_eins ) 
{
    hector_math::Vector3<Scalar> vector_u( a_rb[0] - a_lt[0], a_rb[1] - a_lt[1], 0 );
    hector_math::Vector3<Scalar> vector_v( b_rb[0] - b_lt[0], b_rb[1] - b_lt[1], 0 );
    hector_math::Vector3<Scalar> vector_w( a_lt[0] - b_lt[0], a_lt[1] - b_lt[1], 0 );
    Scalar D = crossProduct( vector_u, vector_v );

    // 检查两个线段是否平行，在线段平行的情况下，检查线段是否共线或者是否包含一个点
    if ( std::fabs( D ) < SMALL_NUM ) {
        if ( crossProduct( vector_u, vector_w ) != 0 || crossProduct( vector_v, vector_w ) != 0 ) {
            return 0;
        }
        Scalar du = dotProduct( vector_u, vector_u );
        Scalar dv = dotProduct( vector_v, vector_v );

        if ( du == 0 && dv == 0 ) {
            if ( a_lt[0] != b_lt[0] || a_lt[1] != b_lt[1] ) { return 0; }
            i_null = a_lt;
            return 1;
        }

        if ( du == 0 ) {
            if ( inSegment( a_lt, b_lt, b_rb ) == 0 ) { return 0; }
            i_null = a_lt;
            return 1;
        }

        if ( dv == 0 ) {
            if ( inSegment( b_lt, a_lt, a_rb ) == 0 ) { return 0; }
            i_null = b_lt;
            return 1;
        }

        Scalar t0, t1;
        hector_math::Vector3<Scalar> vector_w2( a_rb[0] - b_lt[0], a_rb[1] - b_lt[1], 0 );

        if ( vector_v[0] != 0 ) {
            t0 = vector_w[0] / vector_v[0];
            t1 = vector_w2[0] / vector_v[0];
        } else {
            t0 = vector_w[1] / vector_v[1];
            t1 = vector_w2[1] / vector_v[1];
        }

        if ( t0 > t1 ) {
            // double temp = t0;
            // t0 = t1;
            // t1 = temp;
            std::swap( t0, t1 );
        }

        if ( t0 > 1 || t1 < 0 ) { return 0; }

        t0 = t0 < 0 ? 0 : t0;
        t1 = t1 > 1 ? 1 : t1;

        if ( t0 == t1 ) {
            i_null << b_lt[0] + t0 * vector_v[0], b_lt[1] + t0 * vector_v[1], 0;
            return 1;
        }

        i_null << b_lt[0] + t0 * vector_v[0], b_lt[1] + t0 * vector_v[1], 0;
        i_eins << b_lt[0] + t1 * vector_v[0], b_lt[1] + t1 * vector_v[1], 0;
        return 2;
    }

    Scalar sI = crossProduct( vector_v, vector_w ) / D;
    if ( sI < 0 || sI > 1 ) { return 0; }

    Scalar tI = crossProduct( vector_u, vector_w ) / D;
    if ( tI < 0 || tI > 1 ) { return 0; }

    i_null[0] = a_lt[0] + sI * vector_u[0];
    i_null[1] = a_lt[1] + sI * vector_u[1];
    return 1;
}

} // namespace mapbag_editor_server

#endif // MAPBAG_EDITOR_SERVER_INTERSECTSEGMENTS_H



