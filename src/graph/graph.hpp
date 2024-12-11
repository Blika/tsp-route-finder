#pragma once

#include <stdint.h>
#include <mutex>
#include <unordered_map>
#include <string>
#include <vector>

class Chunk;
class Node;
class Graph{
    public:
        std::vector<std::vector<float>> distanceMatrix;
        std::unordered_map<int64_t, std::unordered_map<int16_t, std::unordered_map<int16_t, std::pair<float,std::vector<int16_t>>>>> cachedPaths;
        Graph(const float& radius);
        ~Graph();

        Graph(const Graph&) = delete;
        Graph &operator=(const Graph&) = delete;
        Graph(Graph&&) = delete;
        Graph &operator=(Graph&&) = delete;

        void plot(const int16_t& node_count, const float& min_neighbors, const float& max_neighbors);
        void cachePaths(const int64_t& xy, const int16_t& from, const int16_t& to);
        void drawCircle();
        size_t getChunkCount();
        Chunk* getChunk(const int64_t& xy);
        std::unordered_map<int64_t, Chunk*>& getChunks();
        float getTotalDistance();
        float getRadius();
        void addNode(Node* node);
        bool hasNode(const int16_t& index);
        Node* getNode(const int16_t& index);

    private:
        float radius, totalDistance;
        std::unordered_map<int64_t, Chunk*> chunks;
        std::unordered_map<int16_t, Node*> nodes;
        std::vector<double> xv,yv;
        std::vector<std::string> txt;
        std::mutex cachedPathsMutex;
};