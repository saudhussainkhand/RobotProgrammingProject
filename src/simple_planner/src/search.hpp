#ifndef SEARCH_HPP
#define SEARCH_HPP

#include <vector>
#include <opencv2/opencv.hpp>
#include <queue>

namespace planner {

    class Node {
        public:
            int row;
            int col;
            int steps;
            int vector_index;
            int closest_object_distance;
            int heuristic_cost;  // A* heuristic cost
            Node* parent;

            Node();
            Node(int row, int col, int steps, int vector_index, int closest_object_distance, int heuristic_cost, Node* parent = nullptr);

            bool equals(const Node& other) const;
            // Modify total cost to include the penalty for obstacle proximity
            int totalCost() const { return steps + heuristic_cost + 200 / (closest_object_distance + 1); }  
    };

    struct NodeComparator {
        bool operator()(Node* node_1, Node* node_2);
    };

    Node search(Node& root, Node& goal, cv::Mat distance_map);

}

#endif  // SEARCH_HPP
