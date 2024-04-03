#include <ros/ros.h>
#include "basis_stuff/grid_map.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include <iostream>
#include <vector>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h> 

// Function to convert vector of pairs to a nav_msgs Path message
nav_msgs::Path convertVectorToPath(const std::vector<std::pair<int, int>>& vector_of_pairs) {
    nav_msgs::Path path_message;
    path_message.header.stamp = ros::Time::now(); 
    path_message.header.frame_id = "map"; 

    for (const auto& pair : vector_of_pairs) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose.position.x = pair.first; 
        pose_stamped.pose.position.y = pair.second;
        pose_stamped.pose.position.z = 0.0; 
        pose_stamped.pose.orientation.x = 0.0; 
        pose_stamped.pose.orientation.y = 0.0;
        pose_stamped.pose.orientation.z = 0.0;
        pose_stamped.pose.orientation.w = 1.0;
        pose_stamped.header.stamp = ros::Time::now(); 
        pose_stamped.header.frame_id = "map";
        path_message.poses.push_back(pose_stamped);
    }

    return path_message;
}

// Function to convert image to an OccupancyGrid message
nav_msgs::OccupancyGrid imageToOccupancyGrid(const cv::Mat& image) {
    nav_msgs::OccupancyGrid grid;

    // Set grid dimensions
    grid.info.width = image.cols;
    grid.info.height = image.rows;

    // Set grid resolution (e.g., in meters/cell)
    grid.info.resolution = 1.0;

    // Set grid origin
    grid.info.origin.position.x = 0.0;
    grid.info.origin.position.y = 0.0;
    grid.info.origin.orientation.w = 1.0;

    // Fill grid with image data
    for (int i = 0; i < image.rows; ++i) {
        for (int j = 0; j < image.cols; ++j) {
            // Assuming white pixels are free space and black pixels are obstacles
            int occupancy = (image.at<uchar>(i, j) == 255) ? 0 : 100;
            grid.data.push_back(occupancy);
        }
    }
    return grid;
}

int main(int argc, char** argv) {
   
    GridMap gridMap;
    gridMap.loadImage("/home/lattinone/catkin_ws/your_workspace/src/ros_controller/src/img_folder");

    std::pair<int, int> start = {10,10};  
    std::pair<int, int> goal = {1657,1657};
    gridMap.setStartGoal(start, goal);

    // Initialize the ROS node
    ros::init(argc, argv, "node_gridmap");
    ros::NodeHandle nh;

    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 10);
    std::vector<std::pair<int, int>> path_points;

    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", 10);

    cv::Mat image = cv::imread("/home/lattinone/catkin_ws/your_workspace/src/ros_controller/src/img_folder/labirinto1.jpg", cv::IMREAD_GRAYSCALE);
    if(image.empty()){
        ROS_ERROR("Failed to load image");
        return -1;
    } else {
        std::cout << "Image loaded successfully" << std::endl;
    }

    nav_msgs::OccupancyGrid grid = imageToOccupancyGrid(image);
    gridMap.setOccupancy(grid);
    gridMap.computeDistanceMap();

    path_points = gridMap.findPath(start, goal);

    nav_msgs::Path path_message = convertVectorToPath(path_points);

    // Set the publishing rate
    ros::Rate rate(1); // 1 Hz

    while (ros::ok()) {
        map_pub.publish(grid);
    
        nav_msgs::Path path_msg;
        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = "map";
        for (const auto& point : path_message.poses) {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = "map";
            pose.pose.position.y = point.pose.position.y;
            pose.pose.position.x = point.pose.position.x;
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }

        // Publish the Path message
        path_pub.publish(path_msg);
        rate.sleep();
    }

    return 0;
}
