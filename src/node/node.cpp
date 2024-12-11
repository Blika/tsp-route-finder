#include "node.hpp"
#include "../graph/graph.hpp"
#include "../graph/chunk.hpp"
#include "../path/path_finder.hpp"
#include <cmath>

Node::Node(const int16_t& i, const float& xx, const float& yy)
    : index{i}, x{xx}, y{yy}{
}

Node::~Node(){
}

int16_t Node::getIndex(){
    return index;
}

Chunk* Node::getChunk(){
    int chunkX = (int)std::floor(x) >> 4;
    int chunkY = (int)std::floor(y) >> 4;
    int64_t chunkIndex = ((chunkX << 16) | ((chunkY) & 0xffff));
    return PathFinder::getInstance()->getGraph()->getChunk(chunkIndex);
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

std::unordered_map<int16_t, bool>& Node::getConnections(){
    return connections;
}

int16_t Node::getTotalConnections(){
    return connections.size();
}