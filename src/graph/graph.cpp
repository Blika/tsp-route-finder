#include "graph.hpp"
#include "../node/node.hpp"

Graph::Graph(const float& r)
    : radius{r}{
}

Graph::~Graph(){
    for(auto&[k,v]: nodes){
        delete nodes[k];
    }
}

float Graph::getRadius(){
    return radius;
}

void Graph::addNode(Node* node){
    if(hasNode(node->getIndex())) return;
    nodes[node->getIndex()] = node;
}

bool Graph::hasNode(const uint32_t& index){
    return nodes.contains(index);
}

Node* Graph::getNode(const uint32_t& index){
    if(!hasNode(index)) return nullptr;
    return nodes[index];
}