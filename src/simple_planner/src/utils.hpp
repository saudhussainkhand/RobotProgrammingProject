#ifndef UTILS_HPP
#define UTILS_HPP

#include <opencv2/opencv.hpp>
#include <vector>
#include "search.hpp"  // Including search.hpp to use the Node class

namespace planner {

    bool in_bounds(cv::Mat map, int row, int col);

    // Declare save_explored_nodes function
    void save_explored_nodes(cv::Mat map, std::vector<std::vector<bool>> visited, Node root, Node goal);

}

#endif  // UTILS_HPP
