#ifndef MAPBAG_EDITOR_SERVER_POLYGON_INDICES_ITERATOR_H
#define MAPBAG_EDITOR_SERVER_POLYGON_INDICES_ITERATOR_H

#include <hector_math/types.h>
#include <vector>

namespace mapbag_editor_server
{
using namespace hector_math;

namespace detail
{
template<typename Scalar, typename Functor>
void polygonIterator( const Polygon<Scalar> &polygon, Functor functor );
}


template<typename Scalar, typename Functor>
void polygonIterator( const Polygon<Scalar> &polygon, Functor functor )
{
    if ( polygon.cols() < 3 ) {
      ROS_WARN_STREAM_NAMED("MapbagEditorServer", "Wrong Inputdata for polygonIterator.");
      return;
    }
    else {
        detail::polygonIterator<Scalar, Functor>( polygon, functor );
    }
}


namespace detail
{
template<typename Scalar, typename Functor>
void polygonIterator( const Polygon<Scalar> &polygon, Functor functor )
{
  // Build iteration lines from the polygon points that allow us to get the x value for each
  // discrete y index in the map
  struct Line {
    Line() = default;

    Line( const Point<Scalar> &a, const Point<Scalar> &b )
    {
      Point<Scalar> start = a;
      Point<Scalar> end = b;
      if ( a.y() > b.y() )
        std::swap( start, end );
      Scalar diff_y = end.y() - start.y();
      if ( std::abs( diff_y ) < 1E-4 )
        x_increment = 0;
      else
        x_increment =  static_cast<float>( end.x() - start.x() ) / ( end.y() - start.y() );

      x = start.x();
      start_y = start.y();
      end_y = end.y();
    }

    Scalar start_y;
    Scalar end_y;
    float x;
    float x_increment;
  };

  using LineContainer = typename std::vector<Line>;
  using RegionContainer = typename std::vector<Scalar>;

  LineContainer lines;
  lines.reserve( polygon.cols() );
  Eigen::Index max_y = std::round( polygon.col( 0 ).y() );

  // Build lines from points and obtain max y for the stopping criterion during the iteration loop
  for ( Eigen::Index i = 0; i < polygon.cols() - 1; i++ ) {
    lines.emplace_back( polygon.col(i), polygon.col( i + 1 ) );
    Eigen::Index y_end = std::round( lines[i].end_y );
    if ( y_end > max_y )
      max_y = y_end;
  }
  lines.emplace_back( polygon.col( polygon.cols() - 1 ), polygon.col( 0 ) );

  // Sort lines by their y start, to quickly find new active lines as we iterate over y
  std::sort( lines.begin(), lines.end(),
             []( const Line &a, const Line &b ) { return a.start_y < b.start_y; } );
  size_t active_line_index = 0;
  LineContainer active_lines;
  RegionContainer x_region_segments;

  Eigen::Index y = std::round( lines[active_line_index].start_y );
  for ( ; y <= max_y; y++ ) {
    // Determine lines that ended
    for ( int i = active_lines.size() - 1; i >= 0; i-- ) {
      active_lines[i].x += active_lines[i].x_increment;
      if ( active_lines[i].end_y >= y )
        continue;
      active_lines.erase( active_lines.begin() + i );
    }
    
    // Determine new lines that started
    for ( ; active_line_index < lines.size(); ++active_line_index ) {
      if ( lines[active_line_index].start_y > y ) break;
      // if ( lines[active_line_index].end_y < y ) continue;
      if ( lines[active_line_index].start_y < y )
        lines[active_line_index].x += lines[active_line_index].x_increment;
       // Ignore lines that start and end before current column
      
      active_lines.push_back( lines[active_line_index] );
    }

    // We obtain from each line the x for the current y and use that information to iterate between
    // each pair of x(k) -> x(k+1) where k = 2 * i and i is a natural integer
    x_region_segments.clear();
    for ( size_t i = 0; i < active_lines.size(); i++ ) {
      Eigen::Index x = std::round( active_lines[i].x );
      x_region_segments.push_back( x );
    }
    std::sort( x_region_segments.begin(), x_region_segments.end() );

    Eigen::Index x_start = x_region_segments[0];
    Eigen::Index x_end = x_region_segments[0];

    for( const auto& it : x_region_segments ) {
      if( it < x_start ) x_start = it;
      if( it > x_end ) x_end = it;
    }

    for ( Eigen::Index x = x_start; x <= x_end; x++ ) { functor( x, y ); }

    // for ( size_t i = 0; i < x_region_segments.size() - 1; i += 2 ) {
    //   Eigen::Index x_start = std::round( x_region_segments[i] );
    //   Eigen::Index x_end = std::round( x_region_segments[i + 1] );
    //   for ( Eigen::Index x = x_start; x <= x_end; x++ ) { functor( x, y ); }
    // }
  }
}
} // namespace detail

} //namespace mapbag_editor_server

#endif // MAPBAG_EDITOR_SERVER_POLYGON_INDICES_ITERATOR_H