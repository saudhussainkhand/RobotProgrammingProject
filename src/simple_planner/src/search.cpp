#include "search.hpp"
#include "utils.hpp"
#include <cmath>
#include <vector>
#include <queue>

namespace planner {

    Node::Node() : row(0), col(0), steps(0), vector_index(0), closest_object_distance(0), heuristic_cost(0), parent(nullptr) {}

    Node::Node(int row, int col, int steps, int vector_index, int closest_object_distance, int heuristic_cost, Node* parent)
        : row(row), col(col), steps(steps), vector_index(vector_index), closest_object_distance(closest_object_distance), heuristic_cost(heuristic_cost), parent(parent) {}

    bool Node::equals(const Node& other) const {
        return this->row == other.row && this->col == other.col;
    }

    bool NodeComparator::operator()(Node* node_1, Node* node_2) {
        return node_1->totalCost() > node_2->totalCost();  // A* total cost comparison
    }

    // Heuristic function: Manhattan distance (can be replaced with Euclidean distance)
    int calculate_heuristic(int row, int col, int goal_row, int goal_col) {
        return abs(row - goal_row) + abs(col - goal_col);  // Manhattan distance
    }

    Node search(Node& root, Node& goal, cv::Mat distance_map) {
        std::vector<std::pair<int, int>> movements = { {-1, 0}, {1, 0},  {0, -1}, {0, 1} };  // 4-way movement
        std::priority_queue<Node*, std::vector<Node*>, NodeComparator> frontier;
        std::vector<std::vector<bool>> visited(distance_map.rows, std::vector<bool>(distance_map.cols, false));

        root.heuristic_cost = calculate_heuristic(root.row, root.col, goal.row, goal.col);
        frontier.push(&root);
        visited[root.row][root.col] = true;

        while (!frontier.empty()) {
            Node* current = frontier.top();
            frontier.pop();

            if (current->equals(goal)) {
                save_explored_nodes(distance_map, visited, root, *current);
                return *current;
            }

            for (const auto& move : movements) {
                int n_row = current->row + move.first;
                int n_col = current->col + move.second;
                if (in_bounds(distance_map, n_row, n_col) && distance_map.at<int>(n_row, n_col) != 0) {
                    if (!visited[n_row][n_col]) {
                        int heuristic = calculate_heuristic(n_row, n_col, goal.row, goal.col);
                        int closest_object_distance = distance_map.at<int>(n_row, n_col);
                        Node* child = new Node(n_row, n_col, current->steps + 1, (distance_map.rows - n_row - 1) * distance_map.cols + n_col, closest_object_distance, heuristic, current);
                        frontier.push(child);
                        visited[n_row][n_col] = true;
                    }
                }
            }
        }

        save_explored_nodes(distance_map, visited, root, goal);
        return root;  // Path not found
    }
}
