#include "grid_map.h"
#include <boost/filesystem.hpp>
#include "geometry_msgs/PoseStamped.h"

using namespace std;
namespace fs = boost::filesystem;
using namespace Eigen;

GridMap::GridMap(){
    std::cout << "Initializing the GridMap..." << endl;
};

void GridMap::loadImage(const fs::path& folder){
    fs::path imagePath = findImage(folder);
    if(!imagePath.empty()){
        std::cout << "Image found: " << imagePath <<endl;
        cv::Mat loadedImage = cv::imread(imagePath.string(), cv::IMREAD_GRAYSCALE);
        if(loadedImage.empty()){
            throw runtime_error("Error loading the image");
        }else{
            std::cout << "Image loaded successfully" << endl;
            processImage(loadedImage);
        }
    } else {
        cerr << "No image found in the specified folder" << endl;
    }
}


fs::path GridMap::findImage(const fs::path& folder){
    for (const auto& entry : fs::directory_iterator(folder)) {
        if (isImageFile(entry.path()) && fs::is_regular_file(entry.path())) {
            return entry.path();
        }
    }
    return fs::path();
}

bool GridMap::isImageFile(const fs::path& filePath) {
    std::string extension = filePath.extension().string();
    return (extension == ".png" || extension == ".jpeg" || extension == ".jpg" || extension == ".bmp" || extension == ".gif");
};

void GridMap::processImage(const cv::Mat& image) {
    rows = image.rows;
    cols = image.cols;
    cv2vec(image, gridMapArray);
}

void GridMap::cv2vec(const cv::Mat& src, std::vector<uint8_t>& dst) {
    dst.resize(cols * rows);

    // Convert each pixel from cv::Mat to std::vector<uint8_t>
    for (int i = 0; i < src.rows; ++i) {
        for (int j = 0; j < src.cols; ++j) {
            // Set 0 or 255 to gridMapArray[i,j] if there is an obstacle or not in cell i j
            if (src.at<uchar>(i, j) > 230) {
                dst[i * src.cols + j] = 255;
            } else {
                dst[i * src.cols + j] = 0;
            }
        }
    }
}

void GridMap::setStartGoal(pair<int, int> Start, pair<int, int> Goal){
    start = Start;
    goal  = Goal;
    std::cout << start.first << endl;
    cout << goal.first << endl;
}

void GridMap::setOccupancy(nav_msgs::OccupancyGrid grid){
    gridmapocc = grid;
}

void GridMap::computeDistanceMap() {
    cv::Mat gridMapCV(gridmapocc.info.height, gridmapocc.info.width, CV_8UC1);
    
    for (int i = 0; i < gridMapCV.rows; ++i) {
        for (int j = 0; j < gridMapCV.cols; ++j) {
            gridMapCV.at<uchar>(i, j) = gridmapocc.data[i * gridMapCV.cols + j];
        }
    }

    cv::Mat distance;
    cv::distanceTransform(gridMapCV, distance, cv::DIST_L2, cv::DIST_MASK_PRECISE);
    cv::minMaxLoc(distance, &minDist, &maxDist);

    convertDistanceMap(distance);
}

void GridMap::convertDistanceMap(const cv::Mat& distanceTransform) {
    distanceMap.resize(distanceTransform.rows, distanceTransform.cols);
    for (int i = 0; i < distanceTransform.rows; ++i) {
        for (int j = 0; j < distanceTransform.cols; ++j) {
            distanceMap(i, j) = distanceTransform.at<float>(i, j);
        }
    }
}

int GridMap::actionCost(int x, int y){  //with (x,y) the coords to reach
    int distanceCost = exp((maxDist - distanceMap(x, y))); //exp to highlight the importance of small values of distance cost
    return distanceCost + 1;
}

bool GridMap::isValid(int x, int y) {
    return x >= 0 && x < cols && y >= 0 && y < rows && gridmapocc.data[x + gridmapocc.info.width * y] == 0;
}

int GridMap::heuristic(int x, int y){
    return abs(x - goal.first) + abs(y - goal.second);
}

bool GridMap::checkValidStartAndGoal(pair<int, int> start, pair<int, int> goal) {
    if (start.first < 0 || start.second < 0 || gridmapocc.data[start.first + cols * start.second] == 100 || start.first >= gridmapocc.info.width || start.second >= gridmapocc.info.height) {
        cerr << "Start position is not traversable or it is out of bounds. ";
        return false;
    } else {
        std::cout << "Start position is valid" << endl;
    }
    
    if (goal.first < 0 || goal.second < 0 || gridmapocc.data[goal.second * gridmapocc.info.width + goal.first] == 100 || goal.first >= gridmapocc.info.width || goal.second >= gridmapocc.info.height) {
        cerr << "Goal position is not traversable or it is out of bounds. ";
        return false;
    } else {
        std::cout << "Goal position is valid" << endl;
    }
    
    return true;
}

vector<pair<int, int>> GridMap::findPath(pair<int, int> Start, pair<int, int> Goal) {
    rows = gridmapocc.info.height;
    cols = gridmapocc.info.width;
    
    std::cout << "Starting findPath" << endl;
    vector<pair<int, int>> path;
    if(checkValidStartAndGoal(start, goal)){
        vector<vector<bool>> visited(rows, vector<bool>(cols, false));
        vector<vector<pair<int, int>>> parent(rows, vector<pair<int, int>>(cols, {-1, -1}));
        vector<vector<int>> gScore(rows, vector<int>(cols, INT_MAX));
        vector<vector<int>> fScore(rows, vector<int>(cols, INT_MAX));

        gScore[start.first][start.second] = 0;
        fScore[start.first][start.second] = heuristic(start.first, start.second);
        
        std::set<pair<int, int>> openSet;
        openSet.insert(start);

        vector<pair<int, int>> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

        int min_fScore, tentative_gScore;
        pair<int, int> min_fScoreNode;
        pair<int, int> current, neighbour;
        
        int current_fScore;

        while (!openSet.empty()) {
            min_fScore = INT_MAX;
            for (const auto& node : openSet) {
                current_fScore = fScore[node.first][node.second]; 
                
                if (current_fScore < min_fScore) {
                    min_fScore = current_fScore;
                    min_fScoreNode = node;
                }
            }
            
            current = min_fScoreNode;

            if (current == goal){
                std::cout <<"Path found"<<endl;
                while (current != start) {
                    path.push_back(current);
                    current = parent[current.first][current.second];
                }
                path.push_back(start);
                std::reverse(path.begin(), path.end());
                return path;
            }
            openSet.erase(current);
            visited[current.first][current.second] = true;
            
            for (auto direction : directions ){
                neighbour = {current.first + direction.first, current.second + direction.second};
                if (isValid(neighbour.first, neighbour.second)){
                    tentative_gScore = gScore[current.first][current.second] + actionCost(neighbour.first, neighbour.second);
                    if (tentative_gScore < gScore[neighbour.first][neighbour.second]){
                        parent[neighbour.first][neighbour.second] = current;
                        gScore[neighbour.first][neighbour.second] = tentative_gScore;
                        fScore[neighbour.first][neighbour.second] = gScore[neighbour.first][neighbour.second] + heuristic(neighbour.first, neighbour.second);
                        
                        openSet.insert(neighbour);
                    }
                }
            }
        }
        cerr << "Path not found";
        return path; 
    } else {
        return path;
    }
}
