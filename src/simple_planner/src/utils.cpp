#include "utils.hpp"
#include "search.hpp"  // Include the full Node definition here

namespace planner {

    bool in_bounds(cv::Mat map, int row, int col) {
        return row >= 0 && row < map.rows && col >= 0 && col < map.cols;
    }

    // Implementation of save_explored_nodes
    void save_explored_nodes(cv::Mat map, std::vector<std::vector<bool>> visited, Node root, Node goal) { 
        cv::Mat map_image(map.rows, map.cols, CV_8UC3, cv::Scalar(255, 255, 255)); // BGR
        
        for(unsigned int col = 0; col < uint(map.cols); col++) {
            for(unsigned int row = 0; row < uint(map.rows); row++) {
                if(map.at<int>(row, col) == 0) {
                    map_image.at<cv::Vec3b>(row, col) = cv::Vec3b(0, 0, 0);
                } else if(visited[row][col]) {
                    map_image.at<cv::Vec3b>(row, col) = cv::Vec3b(0, 255, 0);
                }
            }
        }
        map_image.at<cv::Vec3b>(goal.row, goal.col) = cv::Vec3b(255, 0, 0);
        Node* current = goal.parent;
        while (current != nullptr) {
            map_image.at<cv::Vec3b>(current->row, current->col) = cv::Vec3b(0, 195, 255);
            current = current->parent;
        }
        map_image.at<cv::Vec3b>(root.row, root.col) = cv::Vec3b(0, 0, 255);

        cv::imwrite("explored_area.png", map_image);
    }

}
