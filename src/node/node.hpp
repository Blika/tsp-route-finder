#pragma once

#include <stdint.h>
#include <unordered_map>

class Chunk;
class Node{
    public:
        Node(const int16_t& index, const float& x, const float& y);
        ~Node();

        Node(const Node&) = delete;
        Node &operator=(const Node&) = delete;
        Node(Node&&) = delete;
        Node &operator=(Node&&) = delete;

        int16_t getIndex();
        Chunk* getChunk();
        float getX();
        float getY();
        float distance(Node* node);
        void addConnection(Node* node);
        bool hasConnection(Node* node);
        std::unordered_map<int16_t, bool>& getConnections();
        int16_t getTotalConnections();

    private:
        int16_t index;
        float x,y;
        std::unordered_map<int16_t, bool> connections;
};