#ifndef MAPBAG_EDITOR_SERVER_KDTREE_HPP
#define MAPBAG_EDITOR_SERVER_KDTREE_HPP

#include "kdtree.h"
#include <cmath>
#include <ros/ros.h>

namespace mapbag_editor_server
{
using namespace hector_math;
using namespace std;

template <typename Scalar>
Scalar dist_sqrt( const hector_math::Vector3<Scalar> &p1, const hector_math::Vector3<Scalar> &p2 ) {
    Scalar dx = p1[0] - p2[0];
    Scalar dy = p1[1] - p2[1];
    return std::sqrt( dx * dx + dy * dy );
}


template<typename Scalar>
KdTree<Scalar>::KdTree( const std::vector<hector_math::Vector3<Scalar>> &points, const int &dim ) 
    : points_( points ), dimension_( dim ) {}


// template <typename Scalar>
// KdTree<Scalar>::~KdTree() {}


template <typename Scalar>
KdNode<Scalar>* KdTree<Scalar>::createTreeNode( const int &left, const int &right, const int &dim ) 
{
    if ( right < left ) return nullptr;
    // 按照k维进行排序
    sort( points_.begin() + left, points_.begin() + right + 1, [dim] ( const auto &p1, const auto &p2 ) { return p1[dim] < p2[dim]; });
    int mid = ( left + right + 1 ) / 2;
    // KdNode<Scalar>* root( points_[mid], dim );
    KdNode<Scalar>* root = new KdNode<Scalar>( points_[mid], dim );


    root->left = createTreeNode( left, mid - 1, ( dim + 1 ) % dimension_);
    if ( root->left != nullptr ) root->left->parent = root;

    root->right = createTreeNode( mid + 1, right, (dim + 1) % dimension_);
    if ( root->right != nullptr ) root->right->parent = root;
    return root;
}


template <typename Scalar>
void KdTree<Scalar>::createTree() 
{ root_ = createTreeNode( 0, points_.size() - 1, 0 );
ROS_INFO_STREAM_NAMED("MapbagEditorServer", 
                              "createTree" ); 
}


template <typename Scalar>
KdNode<Scalar>* KdTree<Scalar>::searchKdTree( const hector_math::Vector3<Scalar> &searchpoint ) 
{
    int dim = 0;
    Scalar minDis = numeric_limits<Scalar>::max();
    KdNode<Scalar>* root = root_;
    KdNode<Scalar>* tmp;

    while ( root != nullptr )
    {
        tmp = root;
        if ( searchpoint[dim] < root->point[dim] ) { root = root->left; }  
        else { root = root->right; }
        dim = ( dim + 1 ) % dimension_;
    }

    // 找到属于的那个节点
    minDis = min( dist_sqrt( searchpoint, tmp->point ), minDis );
    KdNode<Scalar>* nearNode = tmp;
    // 退回到根节点
    while ( tmp->parent != nullptr )
    {
        tmp = tmp->parent;
        // 判断父节点是否更近，如果近，更新最近节点
        if ( dist_sqrt( tmp->point, searchpoint ) < minDis )
        {
            nearNode = tmp;
            minDis = dist_sqrt( tmp->point, searchpoint );
        }

        KdNode<Scalar>* son;
        // 判断当前轴与点的距离，如果小于minDis，则进行到另一半进行查找
        if ( abs( tmp->point[tmp->axis] - searchpoint[tmp->axis]) < minDis )
        {
            if( tmp->point[tmp->axis] > searchpoint[tmp->axis] ) { son = tmp->right; }
            else { son = tmp->left; }
            searchKdTreeNode( searchpoint, minDis, nearNode, son );
        }
    }   
    // 对根节点的另外半边子树进行搜索
    /*if (abs(tmp->val[tmp->axis] - d[tmp->axis]) < minDis)
    {
        if (tmp->val[tmp->axis] > d[tmp->axis])
            tmp = tmp->rightChild;
        else
            tmp = tmp->leftChild;
        searchKdTreeNode(d, minDis, nearNode, tmp);
    }*/
    return nearNode;
}


template <typename Scalar>
void KdTree<Scalar>::searchKdTreeNode( const hector_math::Vector3<Scalar> &searchpoint, Scalar &minDis, 
                                       KdNode<Scalar>* &nearNode, KdNode<Scalar>* tmp )
{
    if ( tmp == nullptr ) return;
    if ( dist_sqrt( tmp->point, searchpoint ) < minDis ) {
        minDis = dist_sqrt( tmp->point, searchpoint );
        nearNode = tmp;
    }
    // 如果轴与节点的距离小于minDis，则两个半边都需要搜索，否则只需要搜索一个半边
    if ( abs( tmp->point[tmp->axis] - searchpoint[tmp->axis] ) < minDis ) {
        searchKdTreeNode( searchpoint, minDis, nearNode, tmp->left );
        searchKdTreeNode( searchpoint, minDis, nearNode, tmp->right );
    } else {
        // 选择搜索的一个半边
        if ( tmp->point[tmp->axis] > searchpoint[tmp->axis]) searchKdTreeNode( searchpoint, minDis, nearNode, tmp->left );
        else searchKdTreeNode( searchpoint, minDis, nearNode, tmp->right );
    }
}


// k-Nearest Neighbors search function
template <typename Scalar>
std::vector<KdNode<Scalar>*> KdTree<Scalar>::kNearestNeighbors( const hector_math::Vector3<Scalar> &searchpoint, int &k ) {
    ROS_INFO_STREAM_NAMED("MapbagEditorServer", 
                              "kNearestNeighbors" );
    // Create a priority queue to store k-nearest neighbors
    std::priority_queue<pair<double, KdNode<Scalar>*>> nearestNeighbors;
    // std::priority_queue<CompareKdNode*> nearestNeighbors;

    kNearestNeighborsHelper( root_, searchpoint, nearestNeighbors, k );

    // Extract the k-nearest neighbors from the priority queue
    std::vector<KdNode<Scalar>*> result;
    while ( !nearestNeighbors.empty() ) {
        result.push_back( nearestNeighbors.top().second );
        nearestNeighbors.pop();
    }
    return result;
}


template <typename Scalar>
void KdTree<Scalar>::kNearestNeighborsHelper( KdNode<Scalar>* node, const hector_math::Vector3<Scalar> &searchpoint,
                                              std::priority_queue<std::pair<double, KdNode<Scalar>*>> &nearestNeighbors, int &k  ) 
{
    ROS_INFO_STREAM_NAMED("MapbagEditorServer", 
                              "kNearestNeighborsHelper" );
    if ( node == nullptr ) return;

    double dist_sqrtToNode = dist_sqrt( searchpoint, node->point );

    if ( nearestNeighbors.size() < k + 1 ) {
        nearestNeighbors.push( {dist_sqrtToNode, node} );
    } else if ( dist_sqrtToNode < nearestNeighbors.top().first ) {
        nearestNeighbors.pop();
        nearestNeighbors.push( {dist_sqrtToNode, node} );
    }

    int axis = node->axis;
    double axisDifference = searchpoint[axis] - node->point[axis];

    if ( axisDifference <= 0 ) {
        kNearestNeighborsHelper( node->left, searchpoint, nearestNeighbors, k );
        if( axisDifference * axisDifference < nearestNeighbors.top().first ) {
            kNearestNeighborsHelper( node->right, searchpoint, nearestNeighbors, k );
        }
    } else {
        kNearestNeighborsHelper( node->right, searchpoint, nearestNeighbors, k );
        if( axisDifference * axisDifference < nearestNeighbors.top().first ) {
            kNearestNeighborsHelper(node->left, searchpoint, nearestNeighbors, k );
        }
    }
}

template <typename Scalar>
std::vector<hector_math::Vector3<Scalar>> KdTree<Scalar>::knearest_points( const hector_math::Vector3<Scalar> &searchpoint, int &k ) 
{
    ROS_INFO_STREAM_NAMED("MapbagEditorServer", 
                              "knearest_points" );
    std::vector<hector_math::Vector3<Scalar>> knearest_points;
    std::vector<KdNode<Scalar>*> neighbors = kNearestNeighbors( searchpoint, k );
    for( const auto &element : neighbors ) {
        knearest_points.push_back( element->point );
    }
    ROS_INFO_STREAM_NAMED("MapbagEditorServer", 
                              "knearest_points.size: " << knearest_points.size() );
    if (!knearest_points.empty()) {
        knearest_points.erase( knearest_points.end() );
    }
    ROS_INFO_STREAM_NAMED("MapbagEditorServer", 
                              "knearest_points.size: " << knearest_points.size() );
    return knearest_points;
}

} //namespace mapbag_editor_server
#endif // MAPBAG_EDITOR_SERVER_KDTREE_HPP
