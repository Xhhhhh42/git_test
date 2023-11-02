#ifndef MAPBAG_EDITOR_SERVER_CONCAV_HULL_GENERATOR_H
#define MAPBAG_EDITOR_SERVER_CONCAV_HULL_GENERATOR_H

#include "kdtree.h"
#include <hector_math/types.h>

namespace mapbag_editor_server
{

template<typename Scalar>
class Concav_Hull_Generator
{
public:
    typedef std::shared_ptr<Concav_Hull_Generator<Scalar>> Ptr;
    typedef std::shared_ptr<const Concav_Hull_Generator<Scalar>> ConstPtr;

    Concav_Hull_Generator( std::vector<hector_math::Vector3<Scalar>> &polygon_points, int k = 3 );

    ~Concav_Hull_Generator();

    bool concavHull( const std::vector<hector_math::Vector3<Scalar>> &original_points, const int &k );

    std::vector<hector_math::Vector3<Scalar>> concav_points() { return concav_points_; }

    std::vector<hector_math::Vector3<Scalar>> GetNearestNeighbors( const hector_math::Vector3<Scalar> &search_point, int &k );

    void removePoint( std::vector<hector_math::Vector3<Scalar>> &modified_points, 
                      const hector_math::Vector3<Scalar> &removed_point ); 

    bool allPointsInsideHull( const std::vector<hector_math::Vector3<Scalar>> &dataset, 
                              const std::vector<hector_math::Vector3<Scalar>>  &hull );

private:
    std::vector<hector_math::Vector3<Scalar>> original_points_;
    std::vector<hector_math::Vector3<Scalar>> concav_points_;
    mapbag_editor_server::KdTree<Scalar>* kdtree_; 
    bool concavhull_found_;
    int k_;    
};

} //namespace mapbag_editor_server

#include "concav_hull_generator.hpp"

#endif // MAPBAG_EDITOR_SERVER_CONCAV_HULL_GENERATOR_H
