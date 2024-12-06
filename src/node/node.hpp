#pragma once

#include <stdint.h>
#include <unordered_map>

class Node{
    public:
        Node(const uint32_t& index, const float& x, const float& y);
        ~Node();

        Node(const Node&) = delete;
        Node &operator=(const Node&) = delete;
        Node(Node&&) = delete;
        Node &operator=(Node&&) = delete;

        uint32_t getIndex();
        float getX();
        float getY();
        float distance(Node* node);
        void addConnection(Node* node);
        bool hasConnection(Node* node);
        uint32_t getTotalConnections();

    private:
        uint32_t index;
        float x,y;
        std::unordered_map<uint32_t, bool> connections;
};