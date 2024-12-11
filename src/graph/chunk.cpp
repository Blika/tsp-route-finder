#include "chunk.hpp"
#include "../node/node.hpp"

Chunk::Chunk(const int& xx, const int& yy)
    :x{xx},y{yy}{
}

Chunk::~Chunk(){

}

int Chunk::getIndex(){
    return ((x << 16) | ((y) & 0xffff));
}

int Chunk::getX(){
    return x;
}

int Chunk::getY(){
    return y;
}

void Chunk::addNode(Node* node){
    nodes[node->getIndex()] = node;
}

std::unordered_map<int16_t, Node*>& Chunk::getNodes(){
    return nodes;
}

int16_t Chunk::getNodeCount(){
    return nodes.size();
}