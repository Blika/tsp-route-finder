#pragma once

#include <stdint.h>
#include <mutex>
#include <queue>
#include <vector>
#include <unordered_map>

class Graph;
class ThreadPool;
class PathFinder{
    public:
        const int16_t node_count = 100;
        PathFinder();
        ~PathFinder();

        PathFinder(const PathFinder&) = delete;
        PathFinder &operator=(const PathFinder&) = delete;

        static PathFinder* getInstance();
        ThreadPool* getThreadPool();
        Graph* getGraph();
        float rnd(const float& min, const float& max);
        std::pair<float, std::vector<int16_t>> dijkstraPath(const int16_t& start, const int16_t& dest);
        std::pair<int16_t, std::vector<int64_t>> findChunkPath(int64_t currentChunk, const int64_t& endPoint, const int64_t& prevChunk, int16_t distance, std::unordered_map<int64_t, int64_t> visitedChunks, std::vector<int64_t> path, int16_t minDistance);
        void visitAll(int16_t currentNode, const int16_t& endPoint, float distance, std::unordered_map<int64_t, bool> unvisitedChunks, std::vector<int16_t> path);
        std::pair<float,std::vector<int16_t>> findPathResursive(int16_t currentNode, const int16_t& endPoint, const int16_t& prevNode, float distance, std::unordered_map<int16_t, int16_t> visitedNodes, std::vector<int16_t> path, const int16_t& max, float minDistance);
        void drawPath(std::vector<int16_t>& path);
        void run();

    private:
        const float min_r = 30.f;
        const float max_r = 60.f;
        const float min_neighbors = 2.f;
        const float max_neighbors = 6.f;
        const float usd_per_unit = 10.f;
        static PathFinder* instance;
        ThreadPool* threadpool;
        Graph* graph;
        std::mutex foundPathMutex;
        std::pair<float,std::vector<int16_t>> foundPath{0.f,{}};
};