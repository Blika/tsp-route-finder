#pragma once

#include <stdint.h>
#include <unordered_map>

class Node;
class Graph{
    public:
        Graph(const float& radius);
        ~Graph();

        Graph(const Graph&) = delete;
        Graph &operator=(const Graph&) = delete;
        Graph(Graph&&) = delete;
        Graph &operator=(Graph&&) = delete;

        float getRadius();
        void addNode(Node* node);
        bool hasNode(const uint32_t& index);
        Node* getNode(const uint32_t& index);

    private:
        float radius;
        std::unordered_map<uint32_t, Node*> nodes;
};