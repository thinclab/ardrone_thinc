#ifndef DRONE_HPP_
#define DRONE_HPP_

#include "ros/ros.h"
#include <string>
#include <cmath>
using namespace std; 

class drone {
    public: 
        drone(int, int, int); 
        int id; 
        int grid_pos[2]; 

    // ADD: Check for orientation to adjust which way we move
};

#endif
