#include "grid_map.h"
#include <boost/filesystem.hpp>
#include "geometry_msgs/PoseStamped.h"

using namespace std;
namespace fs = boost::filesystem;
using namespace Eigen;

GridMap::GridMap(){
    std::cout << "Initializing the GridMap..." << endl;
};

void GridMap::cv2vec(const cv::Mat& src, std::vector<uint8_t>& dst) {
        // Assuming src is a grayscale image (single channel)

        dst.resize(cols*rows);

        // Convert each pixel from cv::Mat to std::vector<uint8_t>
        for (int i = 0; i < src.rows; ++i) {
            for (int j = 0; j < src.cols; ++j) {
                // Setting to 0 or 255 the gridmap[i,j] if there is an obstacle or not in cell i j
                if (src.at<uchar>(i, j) > 230) {
                    dst[i * src.cols + j] = 255;
                } else {
                    dst[i * src.cols + j] = 0;
                }
            }
        }
}

void GridMap::vec2cv(const std::vector<uint8_t>& src, cv::Mat& dst){
    // Assuming dst is a grayscale image (single channel)
    dst.create(rows, cols, CV_8UC1);

    // Convert each pixel from std::vector<uint8_t> to cv::Mat
    for (int i = 0; i < dst.rows; ++i) {
        for (int j = 0; j < dst.cols; ++j) {
            // Copying the pixel value from src to the corresponding location in dst
            dst.at<uchar>(i, j) = src[i * dst.cols + j];
        }
    }
}

bool GridMap::isImageFile(const fs::path& filePath) {

    std::string extension = filePath.extension().string();
    return (extension == ".png" || extension == ".jpeg" || extension == ".jpg" || extension == ".bmp" || extension == ".gif");
};


void GridMap::loadImage(const fs::path& folder){
    fs::path imgPath = findImage(folder);
    if(!imgPath.empty()){
        std::cout << "Image found: " << imgPath <<endl;
        cv::Mat loadedImg = cv::imread(imgPath.string(), cv::IMREAD_GRAYSCALE);
        if(loadedImg.empty()){
            throw runtime_error("Error loading the image");
        }else{
            std::cout << "Image loaded successfully" << endl;
            processImage(loadedImg);
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
 
void GridMap::processImage(const cv::Mat& img) {
    rows = img.rows;
    cols = img.cols;
    cv2vec(img, gridMapArray);
}

void GridMap::loadFromVec(std::vector<uint8_t> src, int Rows,int Cols){
    gridMapArray = src;
    rows = Rows;
    cols = Cols;
    vec2cv(gridMapArray,gridMapImage);
}

/*
void GridMap::computeDistanceMap() {
    // Converti la griglia della mappa in una matrice OpenCV
    cv::Mat gridMapCV(rows, cols, CV_8U);
    std::copy(gridMapArray.begin(), gridMapArray.end(), gridMapCV.data);

    // Calcola la trasformata di distanza
    cv::Mat distanceTransform;
    cv::distanceTransform(gridMapCV, distanceTransform, cv::DIST_L2, cv::DIST_MASK_PRECISE);
    cv::minMaxLoc(distanceTransform, &minDist, &maxDist);

    // Converti la trasformata di distanza nel formato Eigen
    convertDistanceMap(distanceTransform);
}
 
void GridMap::convertDistanceMap(const cv::Mat& distanceTransform) {
    distanceMap.resize(distanceTransform.rows, distanceTransform.cols);
    for (int i = 0; i < distanceTransform.rows; ++i) {
        for (int j = 0; j < distanceTransform.cols; ++j) {
            distanceMap(i, j) = distanceTransform.at<float>(i, j);
        }
    }
}*/

    


void GridMap::computeDistanceMap(const nav_msgs::OccupancyGrid gridmapoc) {
    // Converti l'OccupancyGrid in una matrice OpenCV
    cv::Mat gridMapCV(gridmapocc.info.height, gridmapocc.info.width, CV_8UC1);
    
    for (int i = 0; i < gridMapCV.rows; ++i) {
        for (int j = 0; j < gridMapCV.cols; ++j) {
            gridMapCV.at<uchar>(i, j) = gridmapocc.data[i * gridMapCV.cols + j];
        }
    }

    // Calcola la trasformata di distanza
    cv::Mat distance;
    cv::distanceTransform(gridMapCV, distance, cv::DIST_L2, cv::DIST_MASK_PRECISE);
    cv::minMaxLoc(distance, &minDist, &maxDist);

    distanceMap.resize(distance.rows, distance.cols);
    for (int i = 0; i < distance.rows; ++i) {
        for (int j = 0; j < distance.cols; ++j) {
            distanceMap(i, j) = distance.at<float>(i, j);
        }
    }

    // Converti la trasformata di distanza nel formato Eigen
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

void GridMap::setStartGoal(pair<int, int> Start, pair<int, int> Goal){
    start = Start;
    goal  = Goal;
    std::cout << start.first << endl;
    cout << goal.first << endl;

}

void GridMap::setOccupancy(nav_msgs::OccupancyGrid grid){
    gridmapocc = grid;
}


int GridMap::actionCost(int x,int y){  //with (x,y) the coords to reach
    int distanceCost = exp(0.5*(maxDist - distanceMap(x, y))); //exp to highlight the importance of small values of distance cost
    return distanceCost + 1;
}

bool GridMap::isValid(int x, int y) {
    return x >= 0 && x < cols && y >= 0 && y < rows && gridmapocc.data[y * gridmapocc.info.width + x] == 0;
}


int GridMap::heuristic(int x,int y){
    return abs(x - goal.first) + abs(y - goal.second);
}

bool GridMap::checkValidStartAndGoal(pair<int, int> start, pair<int, int> goal) {
    //cerr << "start.first: " << goal.first << ", start.second: " << goal.second << ", cols: " << cols << endl;
    //cerr << "Data size: " << gridmapocc.data.size() << endl; // Assicurati che ci siano dati nell'array

    if (gridmapocc.data[start.first + cols * start.second] >= 50) {
        cerr << "Start position is not traversable. Value: ";
        cerr << gridmapocc.data[0];
        return false;
    }else std::cout << "Start position is valid" << endl;
    if (gridmapocc.data[goal.first + gridmapocc.info.width * goal.second] >= 50) {
        cerr << "Goal position is not traversable. Value: ";
        cerr << to_string(gridMapArray[goal.first * cols + goal.second]);
        return false;
    }else std::cout << "Goal position is valid" << endl;
    //std::cout << "Valids" << endl;
    return true;
}

vector<pair<int, int>> GridMap::findPath(pair<int, int> Start, pair<int, int> Goal) {
    rows = gridmapocc.info.height;
    cols = gridmapocc.info.width;
    //cout << gridmapocc.info.width << endl;
    //set start and goal
    std::cout << "Starting findPath" << endl;
    vector<pair<int, int>> path;
    if(checkValidStartAndGoal(start,goal)){
        // Define vectors for keeping track of visited cells and parent cells
        vector<vector<bool>> visited(rows, vector<bool>(cols,false));
        
        vector<vector<pair<int, int>>> parent(rows, vector<pair<int, int>>(cols, {-1, -1}));

        //gscore = the cost from getting from start node to that node
        vector<vector<int>> gScore(rows, vector<int>(cols, INT_MAX));
        
        
        //fscore = the cost from getting from the start node to the goal passing trhough that node (uses heuristic so is estimated)
        vector<vector<int>> fScore(rows, vector<int>(cols, INT_MAX));
        

        //initializing gScore and fScore
        gScore[start.first][start.second] = 0;

        fScore[start.first][start.second] = heuristic(start.first,start.second);
        //std::cout << "heuristic  found" << endl;
        
        // Create a set of pairs
        std::set<pair<int, int>> openSet;

        // Add the starting cell to the open set
        openSet.insert(start);

        // Define vectors for possible movements
        vector<pair<int, int>> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

        int min_fScore,tentative_gScore;
        pair<int, int> min_fScoreNode;
        pair<int, int> current,neighbour;
        
        int current_fScore;
        
        while (!openSet.empty()) {
            //std::::cout << "Size of the set: " << openSet.size() << std::endl;
            // finding the cell with the lowest fscore
            min_fScore = INT_MAX;
            for (const auto& node : openSet) {
                current_fScore = fScore[node.first][node.second]; 
                
                if (current_fScore < min_fScore) {
                    min_fScore = current_fScore;
                    min_fScoreNode = node; // Update min_fScoreNode
                }
            }
            
            current = min_fScoreNode;

            if (current == goal){
                // Reconstruct path
                std::cout <<"path found"<<endl;
                while (current != start) {
                    path.push_back(current);
                    current = parent[current.first][current.second];
                }
                path.push_back(start);
                std::reverse(path.begin(), path.end());
                return path;
            }
            openSet.erase(current);
            visited[current.first][current.second]=true;
            
            for (auto direction : directions ){
                neighbour = {current.first+direction.first,current.second+direction.second};
                if (isValid(neighbour.first,neighbour.second)){
                    tentative_gScore = gScore[current.first][current.second]+actionCost(neighbour.first,neighbour.second);
                    //cerr<<"g score:"<<tentative_gScore<<endl;
                    //cerr<<gScore[0][0];
                    if (tentative_gScore < gScore[neighbour.first][neighbour.second]){
                        // This path to the neighbor is better than any previous one
                        // Record it!
                        parent[neighbour.first][neighbour.second] = current;
                        gScore[neighbour.first][neighbour.second] = tentative_gScore;
                        fScore[neighbour.first][neighbour.second] = gScore[neighbour.first][neighbour.second] + heuristic(neighbour.first,neighbour.second);
                        
                        openSet.insert(neighbour);

                    }

                }

            }
        }
        cerr<<"path not found";
        return path; //failure empty path
    } else return path;
}


void GridMap::displayPath(vector<pair<int, int>> path){
    //convert to rgb
    cv::Mat gridMapImageWithPath(rows, cols, CV_8UC3);
    
    // Convert grayscale to RGB
    cv::cvtColor(gridMapImage, gridMapImageWithPath, cv::COLOR_GRAY2RGB);

    for (auto cell:path){
        cv::Vec3b& pixel = gridMapImageWithPath.at<cv::Vec3b>(cell.first, cell.second);

        // Modify the pixel values (BGR order)
        pixel[0] = 0;  // Blue
        pixel[1] = 0;    // Green
        pixel[2] = 255;    // Red
    }
    // Display the distance map
    cv::namedWindow("Path", cv::WINDOW_AUTOSIZE);
    cv::imshow("Path", gridMapImageWithPath);
    cv::waitKey(0); // Wait for a key press
    cv::destroyWindow("Path");
}


