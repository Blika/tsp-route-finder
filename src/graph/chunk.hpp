#pragma once

#include <stdint.h>
#include <unordered_map>
#include <vector>

class Node;
class Chunk{
    public:
        std::unordered_map<int64_t, std::vector<std::pair<int16_t, int16_t>>> connections;
        Chunk(const int& x, const int& y);
        ~Chunk();

        Chunk(const Chunk&) = delete;
        Chunk &operator=(const Chunk&) = delete;
        Chunk(Chunk&&) = delete;
        Chunk &operator=(Chunk&&) = delete;

        int getX();
        int getY();
        int getIndex();
        void addNode(Node* node);
        std::unordered_map<int16_t, Node*>& getNodes();
        int16_t getNodeCount();

    protected:
        int x,y;
        std::unordered_map<int16_t, Node*> nodes;
};