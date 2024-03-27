#pragma once

#include <Eigen/Core>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <queue>
#include <nav_msgs/OccupancyGrid.h>
#include "geometry_msgs/PoseStamped.h"


namespace fs = boost::filesystem;
using namespace Eigen;
using namespace std;

struct GridMap{
    int rows;
    int cols;
    vector<uint8_t> gridMapArray;
    vector<vector<uint8_t>> gridMapMatrix;
    cv::Mat gridMapImage;
    Matrix<float, Dynamic, Dynamic> distanceMap;
    double minDist, maxDist;
    pair<int, int> start;
    pair<int, int> goal;
    nav_msgs::OccupancyGrid gridmapocc;

    GridMap();

    void cv2vec(const cv::Mat& src, std::vector<uint8_t>& dst);
    void vec2cv(const std::vector<uint8_t>& src, cv::Mat& dst);
    bool isImageFile(const fs::path& filePath);
    void loadFromVec(vector<uint8_t>,int rows,int cols);
    void loadImage(const fs::path& folderName);
    fs::path findImage(const fs::path& folder);
    void processImage(const cv::Mat& img);
    void computeDistanceMap(nav_msgs::OccupancyGrid gridmapocc);
    void convertDistanceMap(const cv::Mat& distanceTransform);
    void displayDistanceMap();
    bool checkValidStartAndGoal(pair<int, int> start, pair<int, int> goal); 
    vector<pair<int, int>> findPath(pair<int, int> Start, pair<int, int> Goal);
    int actionCost(int x,int y);
    bool isValid(int x,int y);
    int heuristic(int x,int y);
    void displayPath(vector<pair<int, int>> path);
    void setStartGoal(pair<int, int> start, pair<int, int> goal);
    void setOccupancy(nav_msgs::OccupancyGrid grid);
  

};