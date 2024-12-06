#include "node.hpp"
#include <cmath>

Node::Node(const uint32_t& i, const float& xx, const float& yy)
    : index{i}, x{xx}, y{yy}{
}

Node::~Node(){
}

uint32_t Node::getIndex(){
    return index;
}

float Node::getX(){
    return x;
}

float Node::getY(){
    return y;
}

float Node::distance(Node* node){
    return pow(pow(x - node->getX(), 2) + pow(y - node->getY(), 2), 0.5f);
}

void Node::addConnection(Node* node){
    if(node == nullptr) return;
    connections[node->getIndex()] = true;
}

bool Node::hasConnection(Node* node){
    if(node == nullptr) return false;
    return connections.contains(node->getIndex());
}

uint32_t Node::getTotalConnections(){
    return connections.size();
}