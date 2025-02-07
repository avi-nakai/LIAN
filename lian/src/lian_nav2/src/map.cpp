#include"map.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/OccupancyGrid.hpp"

Map::Map() : height(-1), width(-1), Grid(nullptr) {
    this->declare_parameter("traversable_threshold", 50.0);
    this->get_parameter("traversable_threshold", traversable_threshold);
}
Map::~Map()
{	
    if(Grid) {
        for(int i = 0; i < height; i++) {
            delete[] Grid[i];
        }
        delete[] Grid;
    }
}

int * Map::operator [] (int i) {
    return Grid[i];
}

const int * Map::operator [] (int i) const {
    return Grid[i];
}

bool Map::CellIsTraversable(int curr_i, int curr_j) const {
    return (Grid[curr_i][curr_j] != CN_OBSTL);
}

bool Map::CellIsObstacle(int curr_i, int curr_j) const {
    return (Grid[curr_i][curr_j] == CN_OBSTL);
}

bool Map::CellOnGrid(int curr_i, int curr_j) const {
    return (curr_i < height && curr_i >= 0 && curr_j < width && curr_j >= 0);
}

int Map::getHeight() const {
    return height;
}

int Map::getWidth() const {
    return width;
}

double Map::getCellSize() const {
    return CellSize;
}

int cellThreshold(int cell_data){
    
    if(cell_data < traversable_threshold){
        cell_data = 0;
    }
    else{
        cell_data = 1;
    }    
}

bool Map::getMap(OccupancyGrid camelCase) {

    height = camelCase.info.height;
    if (height <= 0) {
        std::cout << "Error! Wrong 'height' value." << std::endl;
        return false;
    }
    
    width = camelCase.info.width;
    if (width <= 0) {
        std::cout << "Error! Wrong 'width' value." << std::endl;
        return false;
    }
    
    CellSize = camelCase.info.resolution;
    if (CellSize <= 0) {
        std::cout << "Warning! Wrong 'CellSize' value. Set to default value: 1." << std::endl;
        CellSize = 1;
    }

    Grid = new int * [height];
    for (int i = 0; i < height; i++) {
        Grid[i] = new int[width];
    }

    hasGrid = true;
    
    for(int i=0; i < height; i++) {
        for(int j=0; j< width; j++){
            Grid[i][j] = cellThreshold(camelCase.data[width * i + j]);
        }
    }
    
} 
