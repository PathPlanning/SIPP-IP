#pragma once
#include <iostream>
using namespace std;

// Horizon timestep. This equals to INF/F (seconds). 
// Please be attention that this variable affects the stored memory in generate_obstacles file.
#define INF 40000 
#define MXO 4 // number of different orientation. 0 is the East, then rotate with counterclockwise
#define MXV 2 // number of different velocities.
#define F 10 // number of timesteps in one second i.e. 1/T

// Exactly one map (and its specs) from below should be uncommented.

/** map room-64-64-16 ready specifications. Uncomment the three lines below to use this map. **/
// #define MXH 64 // maximum height of the environment
// #define MXW 64 // maximum width of the environment
// string map = "room-64-64-16";

/** map empty_64_64 ready specifications. Uncomment the three lines below to use this map. **/
// #define MXH 64 // maximum height of the environment
// #define MXW 64 // maximum width of the environment
// string map = "empty_64_64";

/** map warehouse-10-20-10-2-2 ready specifications. Uncomment the three lines below to use this map. **/
// #define MXH 84 // maximum height of the environment
// #define MXW 170 // maximum width of the environment
// string map = "warehouse-10-20-10-2-2";

/** map Sydney_2_256 ready specifications. Uncomment the three lines below to use this map. **/
// #define MXH 256 // maximum height of the environment
// #define MXW 256 // maximum width of the environment
// string map = "Sydney_2_256";

/** map random128 ready specifications. Uncomment the three lines below to use this map. **/
#define MXH 128 // maximum height of the environment
#define MXW 128 // maximum width of the environment
string map = "random128";


const int NumOfTests = 1;
vector<int> factors = {25, 20, 15, 10, 5, 4, 3};// number of obstacles = number of free cells / factor.
const int MAX_NUM_NODES = 100000000; // one hundred million nodes (OPEN+CLOSED sets' nodes)