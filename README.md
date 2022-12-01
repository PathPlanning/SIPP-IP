# Safe Interval Path Planning with Interval Projection (SIPP-IP)
Safe Interval Path Planning with Interval Projection (SIPP-IP) - a SIPP-based planner capable of handling non-instantaneous accelerations/decelerations of an agent (kinodynamic constraints).

## Code:
### General structure:
The project consists of the following folders:
1. `src` and `include` folders where the codes and headers of algorithms A*, SIPP1, SIPP2, SIPP-IP are located in addition to a code to generate the tests.
2. In `maps` folder, the maps which were used in the tests are added.
3. `tests` folder contains the generated instances grouped by the name of the map.
4. The results of the algorithms are written to text files inside the folder `results`.

All codes are self-contained. That is, no dependencies other than C++ compiler are needed, and the needed headers are added directly to the codes. Just **compile the needed file and run it**.

### Generating the instances
The first step in testing is to generate the instances of the tests. However, several sample instances for each map are already generated and added.

To generate the tests, use the code `generate_obstacles` in `src` folder. This will generate `NumberOfTests` instances of each map and store them in corresponding folder in `tests` folder. 
* The first paramter which need to be specified is `NumOfTests` in `include/constants.h` file. This defines the number of tests to be generated.
* The second paramter is vector `factors` in `include/constants.h` which defines the number of dynamic obstacles to be generated in each instance. That is, for each one of `NumOfTests`, several instances will be generated each one specified by *factor* from `factors` where the number of dynamic obstacles equal to *(number of free cells in the map / factor)*. See folder `tests/empty_64_64` for an example of one group of tests.
* The procedure of generating the dynamic obstacles is described in the paper. It is also easy to track the procedure inside the code and modify it if needed.

### Running the algorithms
We added our implementation of algorithms A*, SIPP1, SIPP2 and SIPP-IP (all were mentioned in the paper). To run a code, the following paramters in file `include/constants.h` should be specified before compiling:
* Parameters `MXH`, `MXW` and `map`: the height, the width and the name of the map on which the tests will be conducted.
* Paramter `NumOfTests` to specify how many tests to be conducted on the chosen map.
* The number of dynamic obstacles are also defined by vector `factors`.

The used motion primitves are hard-coded inside the code by function `fillActions()`, however they can be modified if needed.

#### output:
The output of algorithms are stored in corresponding folder in folder `results`. There will a one output file for each (map+number of dynamic obstacles). This file containts `NumberOfTests` lines. Each line outputs the following: 
1. Whether the algorithm finds a solution or not.
2. The cost of found solutions.
3. The runtime.

See `results/results SIPP-IP/res-empty_64_64-SIPP-IP-obs163.txt` for an example.

We also added function `printSolutionStates` inside the codes of the algorithms to print the solution states. It can be used to print out the solutions.
