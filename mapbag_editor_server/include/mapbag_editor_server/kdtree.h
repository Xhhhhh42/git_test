#ifndef MAPBAG_EDITOR_SERVER_KDTREE_H
#define MAPBAG_EDITOR_SERVER_KDTREE_H

#include <hector_math/types.h>
#include <vector>
#include <queue>

namespace mapbag_editor_server
{

template<typename Scalar>
struct KdNode
{
    KdNode* parent;
    KdNode* left;
    KdNode* right;
    hector_math::Vector3<Scalar> point;    
    int axis;       
    KdNode( const hector_math::Vector3<Scalar> &data, const int &ax )
    {
        point = data;
        axis = ax;
        parent = nullptr;
        left = nullptr;
        right = nullptr;
    }
};

// template<typename Scalar>
// struct CompareKdNode {
//     double distance_;
//     KdNode<Scalar>* node;

//     CompareKdNode( double dist, KdNode<Scalar>* n ) : distance_( dist ), node(n) {}

//     bool operator>( const CompareKdNode& other ) const {
//         return distance_ > other.distance_;
//     }
// };

template<typename Scalar>
class KdTree
{
public:
    typedef std::shared_ptr<KdTree<Scalar>> Ptr;
    typedef std::shared_ptr<const KdTree<Scalar>> ConstPtr;

    KdTree( const std::vector<hector_math::Vector3<Scalar>> &points, const int &dim );

    ~KdTree() {}

    void createTree();

    KdNode<Scalar>* createTreeNode( const int &left, const int &right, const int &dim );

    KdNode<Scalar>* searchKdTree( const hector_math::Vector3<Scalar> &searchpoint );

    void searchKdTreeNode( const hector_math::Vector3<Scalar> &searchpoint, Scalar &minDis, KdNode<Scalar>* &nearNode, KdNode<Scalar>* tmp );

    std::vector<KdNode<Scalar>*> kNearestNeighbors( const hector_math::Vector3<Scalar> &searchpoint, int &k );

    // void kNearestNeighborsHelper( KdNode<Scalar>* node, const hector_math::Vector3<Scalar> &searchpoint, 
    //                               std::priority_queue<std::pair<double, KdNode<Scalar>*>> &nearestNeighbors, int &k );
    
    void kNearestNeighborsHelper( KdNode<Scalar>* node, const hector_math::Vector3<Scalar> &searchpoint,
                                  std::priority_queue<std::pair<double, KdNode<Scalar>*>> &nearestNeighbors, int &k );

    std::vector<hector_math::Vector3<Scalar>> knearest_points( const hector_math::Vector3<Scalar> &searchpoint, int &k );                              

private:
    std::vector<hector_math::Vector3<Scalar>> points_;
    mapbag_editor_server::KdNode<Scalar>* root_;
    int dimension_;
};

} //namespace mapbag_editor_server

#include "kdtree.hpp"

#endif // MAPBAG_EDITOR_SERVER_KDTREE_H
