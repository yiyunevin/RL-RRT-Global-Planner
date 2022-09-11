/**
* 2022.07.28 Last Edited by Evin Hsu
*/

#ifndef RRT_NODE_H_
#define RRT_NODE_H_

#include <cmath>

static inline double euclideanDistance2D(double x1, double y1, double x2, double y2)
{
    return hypot( (x1-x2), (y1-y2) );
}

struct Node {
    double x{0.0};
    double y{0.0}; 
    int node_id{0}; 
    int parent_id{-1};
    double cost{0.0}; 
    
    Node() {}
    
    Node(double px, double py, int node_index, int parent_index) : 
        x(px),
        y(py),
        node_id(node_index),
        parent_id(parent_index) {}
    
    bool operator ==(const Node& node)
    { 
        if (node_id == node.node_id)
            return true;
        
        if ( euclideanDistance2D(this->x, this->y, node.x, node.y) <= 0.001 )
            return true;
        
        return false;
    }
    
    bool operator !=(const Node& node) 
    { 
        if (node_id == node.node_id)
            return false;
        
        if ( euclideanDistance2D(this->x, this->y, node.x, node.y) <= 0.001 )
            return false;
        
        return true;
    }
};

#endif
