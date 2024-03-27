#include <ros/ros.h>
#include "basis_stuff/grid_map.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include <iostream>
#include <vector>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h> 


nav_msgs::Path convertiVectorToPath(const std::vector<std::pair<int, int>>& vettore) {
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now(); // Imposta il timestamp
    path_msg.header.frame_id = "map"; // Sostituisci "nome_frame" con il frame ID desiderato

    for (const auto& coppia : vettore) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose.position.x = coppia.first; // Assegna la prima parte della coppia all'asse x
        pose_stamped.pose.position.y = coppia.second; // Assegna la seconda parte della coppia all'asse y
        pose_stamped.pose.position.z = 0.0; // Imposta z a zero o al valore desiderato
        pose_stamped.pose.orientation.x = 0.0; // Imposta l'orientamento se necessario
        pose_stamped.pose.orientation.y = 0.0;
        pose_stamped.pose.orientation.z = 0.0;
        pose_stamped.pose.orientation.w = 1.0;
        pose_stamped.header.stamp = ros::Time::now(); // Imposta il timestamp per questa posa
        pose_stamped.header.frame_id = "map"; // Imposta il frame ID per questa posa

        path_msg.poses.push_back(pose_stamped); // Aggiunge la posa alla sequenza
    }

    return path_msg;
}

nav_msgs::OccupancyGrid imageToOccupancyGrid(const cv::Mat& img) {
    nav_msgs::OccupancyGrid grid;

    // Imposta le dimensioni della griglia
    grid.info.width = img.cols;
    grid.info.height = img.rows;

    // Imposta la risoluzione della griglia (ad esempio, in metri/cella)
    grid.info.resolution = 1.0;

    // Imposta l'origine della griglia
    grid.info.origin.position.x = 0.0;
    grid.info.origin.position.y = 0.0;
    grid.info.origin.orientation.w = 1.0;
    // Riempie la griglia con i dati dell'immagine
    for (int i = 0; i < img.rows; ++i) {
        for (int j = 0; j < img.cols; ++j) {
            // Supponendo che i pixel bianchi siano spazi liberi e i pixel neri siano ostacoli
            int occupancy = (img.at<uchar>(i, j) == 255) ? 0 : 100;
            grid.data.push_back(occupancy);
        }
    }
    return grid;
}

int main(int argc, char** argv) {
   
    GridMap gridMap;
    gridMap.loadImage("/home/lattinone/catkin_ws/your_workspace/src/ros_controller/src/img_folder");

    pair<int, int> start = {320,290};
    pair<int, int> goal = {1600,290};
    gridMap.setStartGoal(start,goal);


    vector<int> rowsAndCols(2);
    rowsAndCols[0]= gridMap.rows;
    rowsAndCols[1]= gridMap.cols;

    // Initialize the ROS node
    ros::init(argc, argv, "node_gridmap");
    ros::NodeHandle nh;


    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map",20);
    vector<pair<int, int>> path_;

    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", 20);

    // Define the topic name
   // std::string topic_name = "gridmap";

    // Create a publisher for UInt8MultiArray messages
    //ros::Publisher pub = nh.advertise<std_msgs::UInt8MultiArray>(topic_name, 10);
    //ros::Publisher pub2 = nh.advertise<std_msgs::Int32MultiArray>("rowscols", 10);


    ////////////////////////////////////////////
    cv::Mat img = cv::imread("/home/lattinone/catkin_ws/your_workspace/src/ros_controller/src/img_folder/labirinto.jpg", cv::IMREAD_GRAYSCALE);
    if(img.empty()){
        ROS_ERROR("Failed to load img");
        return -1;
    }else cout << "ok" << endl;

    
    nav_msgs::OccupancyGrid grid = imageToOccupancyGrid(img);
    gridMap.setOccupancy(grid);
    gridMap.computeDistanceMap(grid);


    path_ = gridMap.findPath(start,goal);

    nav_msgs::Path path = convertiVectorToPath(path_);


    // Set the publishing rate
    ros::Rate rate(80); // 1 Hz

    

    while (ros::ok())
    {   
        map_pub.publish(grid);
        // Create the message object
        std_msgs::UInt8MultiArray matrix_msg;
        matrix_msg.data = gridMap.gridMapArray;

        // Create the message object
        std_msgs::Int32MultiArray rowsAndColsMsg;
        rowsAndColsMsg.data = rowsAndCols;
        


        nav_msgs::OccupancyGrid grid_msg;
        grid_msg.header.stamp = ros::Time::now();
        grid_msg.header.frame_id = "map";  // or any other frame in which the path is defined

        // Set the grid size
        grid_msg.info.width = rowsAndColsMsg.data[0];
        grid_msg.info.height = rowsAndColsMsg.data[1];

        // Set the grid resolution (e.g., in meters/cell)
        grid_msg.info.resolution = 1.0;

        // Set the grid origin
        grid_msg.info.origin.position.x = 0.0;
        grid_msg.info.origin.position.y = 0.0;
        grid_msg.info.origin.orientation.w = 1.0;
        grid_msg.data.resize(grid_msg.info.width * grid_msg.info.height,0);

        // Fill the grid with the path data
        for (const auto& point : path_) {
            // Assuming that 'point' is a pair (x, y)

            int index = point.first + point.second * rowsAndColsMsg.data[0];

            
            // Set the cell at the calculated index to occupied
            grid_msg.data[index] = 100;
            
        }

        // Publish the OccupancyGrid message
        map_pub.publish(grid_msg);


        nav_msgs::Path path_msg;
            path_msg.header.stamp = ros::Time::now();
            path_msg.header.frame_id = "map";
            for (const auto& point : path_) {
                geometry_msgs::PoseStamped pose;
                pose.header.stamp = ros::Time::now();
                pose.header.frame_id = "map";
                pose.pose.position.x = point.second;
                pose.pose.position.y = rowsAndCols[0] - point.first;
                pose.pose.orientation.w = 1.0;
                path_msg.poses.push_back(pose);
            }

            // Pubblica il messaggio Path
            path_pub.publish(path_msg);


        //ROS_INFO("completed");
        
        

        // Publish the message
        //pub.publish(matrix_msg);
        //pub2.publish(rowsAndColsMsg);
        rate.sleep();
    }

    return 0;
}