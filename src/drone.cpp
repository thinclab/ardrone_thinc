/*
 * Represents a drone- qualities include position, topics, etc.
 */

#include "drone.hpp"

/*
 * Constructor for a drone. The drone has id n 
 * and has initial grid position (x,y).
 */
drone::drone(int n, int x, int y) {
    id = n; 
    grid_pos[0] = x; 
    grid_pos[1] = y;

    //we can query absolute position
    //from Gazebo if needed 
}

