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
        void visitAll(int16_t currentNode, const int16_t& endPoint, float distance, std::unordered_map<int64_t, bool> visitedChunks, int16_t chunksCount, std::vector<int16_t> path, bool breakAlg, const int16_t& start, const int16_t& end);
        std::pair<float,std::vector<int16_t>> findPathResursive(int16_t currentNode, const int16_t& endPoint, const int16_t& prevNode, float distance, std::unordered_map<int16_t, int16_t> visitedNodes, std::vector<int16_t> path, const int16_t& max, float minDistance);
        void drawPath(std::vector<int16_t>& path);
        int64_t getTimestamp();
        void handleTimeout(int64_t max);

        void run();

    private:
        const float min_r = 40.f;
        const float max_r = 50.f;
        const float min_neighbors = 2.f;
        const float max_neighbors = 6.f;
        const float usd_per_unit = 10.f;
        bool stopSearching = false;
        int64_t lastFoundPath = 0;
        static PathFinder* instance;
        ThreadPool* threadpool;
        Graph* graph;
        std::mutex foundPathMutex;
        std::pair<float,std::vector<int16_t>> foundPath{0.f,{}};
};