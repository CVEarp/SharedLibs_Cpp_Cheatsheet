# Creating shared libraries in C++.

By Carlos Viescas Huerta. 
Robot Systems MSc student at University of Southern Denmark.

## 1. Introduction 

Simple and very useful for any kind of project which requires some functions. Three types of files are required:
  - Header file: `file.h`
  - Source file: `file.cpp`
  - Main file: `main.cpp`

For this cheatsheet, I am using a short example (simplified version, not the complete one) from another [repository](https://github.com/CVEarp/ROVI2_Object_Avoidance/tree/master/ROS_pkgs/planner) of mine. 

## 2. Project's tree structure. 

 ```
 - Project_Directory
            - build/
            - include/
                    - AnytimePlanning.h
            - src/
                    - AnytimePlanning.cpp
                    - main.cpp
           - CMakeLists.txt
 ```

## 3. Header files.

Header files are design to contain all class, functions and global variable declarations in a short and efficient way. They provide easy and quick view of the content of the program. 

Mainly, in the header file we will declare all the new classes we will create and its public and private functions and variables. If we will create functions based on other libraries previously built and compiled in our system (as in the example), we will include the header reference to such libraries aswell. 

Here is an example of header file:

```cpp
// Anytime Dynamic Path Planning Shared Library

// Implemented fr Robotics & Computer Vision 2 course 2018, SDU Robotics
// Carlos Viescas
// Uses RobWork & CAROS

// AnytimePlanning.h

#include <iostream>
#include <fstream> 
#include <vector>
#include <math.h> 
#include <numeric>
#include <algorithm>
#include <iterator>
#include <boost/foreach.hpp>
#include <string>
#include <vector>
// RobWork headers
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/trajectory/CubicSplineFactory.hpp>
// Caros headers
/*#include <caros/universal_robots.h>
#include <caros/common.h>
#include <caros/common_robwork.h>*/

using namespace std;
using namespace rw::common;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;


class AnytimePlanning{
  public: 
	
	void Load_WorkCell(const string wc_name, const string dev_name);
	bool checkCollisions(const State &state, const CollisionDetector &detector, const Q &q);
	QPath get_path(double epsilon, State state, rw::math::Q from, rw::math::Q to);
	QPath get_trajectory(QPath path, rw::math::Q dq_start, rw::math::Q dq_end);
	QPath return_path(const string filename);
	//vector<caros::caros_common_msgs::Q> convert_trajectory(QPath path);

  private: 
	/*const string wc_name;
	const string dev_name;*/
	rw::models::WorkCell::Ptr wc;
	Device::Ptr device;
	bool dev_found;
	bool wc_found;
	rw::pathplanning::QToQPlanner::Ptr planner;



}; // AnytimePlanning

```

## 4. Source files.

Source files must be named the same as their header file. For the previous example, where we created `AnytimePlanning.h`, we should create the source file `AnytimePlanning.cpp`. Here is where we will code our functions. The first step is to include the corresponding header file. Then, we can set the namespaces used (if wanted). If not, remember to use the class namespace to the newly created class `AnytimePlanning{}` and  the rest of the external libraries used (like `std`, `rw::math::Q{}`, ... )

Here is the .cpp source file corresponding to the example:

```cpp
// Anytime Dynamic Path Planning Shared Library

// Implemented fr Robotics & Computer Vision 2 course 2018
// SDU Robotics
// Carlos Viescas

#include <AnytimePlanning.h>

#define MAXTIME 100.

using namespace std;
using namespace rw::common;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;


/* NOTE */

// Remember to initialize rw::kinematics::State state variable in the main().


void AnytimePlanning::Load_WorkCell(const string wc_name, const string dev_name)
{

	cout << "	>> WorkCell: " << wc_name << endl; 

	// if found, loading workcell
	wc = WorkCellFactory::load(wc_name);
	if (wc == NULL) 
	{
		cerr << "WorkkCell: " << wc_name << " not found!" << endl;
		wc_found = false;
	} // if	

	// and device
	device = wc->findDevice(dev_name);
	if (device == NULL) 
	{
		cerr << "Device: " << dev_name << " not found!" << endl;
		dev_found = false;
	} // if

	cout << "	>> Found device: " << dev_name << endl;

}// Load_WorkCell()


/* 
	In the main, after calling Load_Workcell(), use the following breaking condition:

	if(wc_found == false || dev_found == false)
		{return 0;}
*/


// Function that looks for collisions at a given state (Q).
bool AnytimePlanning::checkCollisions(const State &state, const CollisionDetector &detector, const rw::math::Q &q) 
{
	State testState;
	CollisionDetector::QueryResult data;
	bool colFrom;

	testState = state;
	device->setQ(q,testState);
	colFrom = detector.inCollision(testState,&data);
	if (colFrom) 
	{
		cerr << "Configuration in collision: " << q << endl;
		cerr << "Colliding frames: " << endl;
		FramePairSet fps = data.collidingFrames;
		for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) 
		{
			cerr << (*it).first->getName() << " " << (*it).second->getName() << endl;
		} //for
		return false;
	} // if
	return true;

} // checkCollisions()



// Constraint and Collision strategies; Then Generates path
QPath AnytimePlanning::get_path(double epsilon, State state, rw::math::Q from, rw::math::Q to)
{
	// Set collision detection strategy.
	CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy()); 

	// Set the planner constraint to build RRT-connect.
	PlannerConstraint constraint = PlannerConstraint::make(&detector, device, state); 
	QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(device),constraint.getQConstraintPtr());
	QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();

	// Check for collisions at initial configuration.
	if (!AnytimePlanning::checkCollisions(state, detector, from))
		{return 0;}
	// Check for collisions at final configuration.
	if (!AnytimePlanning::checkCollisions(state, detector, to))
		{return 0;}

	/*if(ext1 == true || ext2 == true)
		{return 0;}*/

	cout << endl;
	cout << "Initial and Final configurations checked. <=========> Collision Free" << endl;
	cout << endl;

	planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, epsilon, RRTPlanner::RRTConnect);

	QPath path;
	Timer t;
	t.resetAndResume();
	planner->query(from, to, path, MAXTIME);
	t.pause();
	cout << endl;
	cout << "	>> Path's length: " << path.size() << endl;
	cout << "	>> Computation time: " << t.getTime() << " seconds." << endl;
	cout << endl;

	// Saving original plan
	int j = 0;
	ofstream pf;
	pf.open("/home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/RWStudio/genfiles/path_original.txt");
	for (QPath::iterator it = path.begin(); it < path.end(); it++) 
	{
		pf << j << ":  " << *it << endl;
		j++;	
	} // for
	cout << "Saved to /home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/RWStudio/genfiles/path_original.txt" << endl;
	cout << endl;
	
	pf.close();

	return path;

} // get_path()




// Interpolate path nodes to get a trayectory
QPath AnytimePlanning::get_trajectory(QPath path, rw::math::Q dq_start, rw::math::Q dq_end)
{

	int imax = path.size();
	QPath interpolated_path;
	vector<double> times;
	double tt = 0.1;
	for( int i = 0; i< imax; i++)
	{
		times.push_back(tt);
		tt = tt+0.1;
	} // for
	InterpolatorTrajectory<rw::math::Q>::Ptr traj = CubicSplineFactory::makeClampedSpline(path, times, dq_start, dq_end);

	int j = 0;
	ofstream tf;
	tf.open("/home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/RWStudio/genfiles/path_interpolated.txt");	

	double t0 = traj->startTime();
  	double tn = traj->endTime();

	cout << "	>> Trajectory duration: " << tn << " seconds." << endl;
	
	for (double t = t0; t <= tn; t += 0.01) 
	{
		// Conversion to save to .lua file
		rw::math::Q q_i(6, traj->x(t)[0], traj->x(t)[1], traj->x(t)[2], traj->x(t)[3], traj->x(t)[4], traj->x(t)[5] );  
		tf << j << ":  t = " << t << "  |  " << q_i << endl;
		interpolated_path.push_back(q_i);

	} // for
	cout << "	>> Trajectory length: " << interpolated_path.size() << " nodes." << endl;
	cout << "	>> Saved with time steps of 0.01 seconds." << endl;
	cout << endl;
	cout << "Saved to /home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/RWStudio/genfiles/path_interpolated.txt" << endl;
	cout << endl;
	
	tf.close();
	
	return interpolated_path;

} // get_trajectory()



// This function reads the path to go back to q_start from the file where it is stored. This path is always constant in the program.
// No obstacle are added when returning.
QPath return_path(const string filename)
{

	ifstream rf;
	rf.open(filename);

	string lines;	
	int nl = 0;

	QPath return_path;

	// Getting number of lines in .csv file (number of steps in the path).
	while( getline(rf, lines) )
	{
		nl++;
	
	} // while 
	
	rf.close();


	// Transfering joint values.
	ifstream rff(filename);
	float val;
	vector<float> values;
	
	while( rff >> val )
    	{
	
		values.push_back(val);
    	
	} // for line
		
	rff.close();

	int values_lgth = values.size();

	if (nl == values_lgth)
	{
		// Reading data in file into rw::math::Q vector to be stored in QPath.
		cout << "	>> Reading path:" << endl;
		int ind = 0;	
		for (int i = 0; i<nl; i++)
		{
			int joint0 = ind;
			int joint1 = ind+1;
			int joint2 = ind+2;
			int joint3 = ind+3;
			int joint4 = ind+4;
			int joint5 = ind+5;
	
			float q0 = values[joint0];
			float q1 = values[joint1];
			float q2 = values[joint2];
			float q3 = values[joint3];
			float q4 = values[joint4];
			float q5 = values[joint5];

			rw::math::Q q_new(6, q0, q1, q2, q3, q4, q5);
		
			return_path.push_back(q_new);
		
			ind = ind + 6;

		} // for i

	} // if
	else
	{
		cout << "An error occurred while reading the file" << endl;
		return 0;
	} // else

	return return_path;

} // return_path()

```

## 5. Main.cpp

In the main program, it is just required to include the header to the newly created class. Then, we can start using the functions that we coded before. Here it is an exampe of how to use it:

```cpp
#include <AnytimePlanning.h>

#include <iostream>
#include <fstream> 
#include <vector>
#include <math.h> 
#include <numeric>
#include <algorithm>
#include <iterator>
#include <boost/foreach.hpp>
#include <string>
#include <vector>
// RobWork headers
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/trajectory/CubicSplineFactory.hpp>

using namespace std;

#define MAXTIME 1000.

//---------------------------------------------------------------------------------------------------------


int main(int argc, char** argv) 
{
	//rw::math::Math::seed();
	
	// LOADING WORKCELL 
	
	cout << endl;
	cout << "C++ SHARED LIBRARIES EXAMPLE" << endl; 
	cout << endl;

	const string wcFile = "/home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/WorkCell_scenes/WorkStation_2/WC2_Scene.wc.xml"; 
	const string deviceName = "UR1";

	AnytimePlanning plan;
	plan.Load_WorkCell(wcFile, deviceName);
	
	cout << endl;
	cout << "Finished." << endl;
	cout << "Exiting." << endl;
	cout << endl;

	return 0;

} // main
```

## 6. CMakeLists.txt

Up to this point we have a project somewhat like the following one:
```
        - planner/
                - build/
                - include/
                        - AnytimePlanning.h
                - src/ 
                        - AnytimePlanning.cpp
                        - main.cpp
                - CMakeLists.txt
```

The final step is to fil the CMakeLists.txt file. This is the one that will link the shared library to the project. The CMakeLists of the project is shown below. Pay special attention to the lines denoted by `#--------------------   CREATE SHARED LIBRARIES IN C++ !!!   --------------------`.

```cmake
# Test CMake version
CMAKE_MINIMUM_REQUIRED(VERSION 2.6.0)

# The name of the project
PROJECT(planner)
MESSAGE(${PROJECT_NAME} ":")

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# Use c++11
SET(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


# Set the RobWork root (edit in .bashrc if necessary)
SET(RW_ROOT $ENV{RW_ROOT})

# Verbose mode prints more info when doing make
set(CMAKE_VERBOSE_MAKEFILE false)

# Set build type to release
SET(CMAKE_BUILD_TYPE Release)
MESSAGE("-- Build type: " ${CMAKE_BUILD_TYPE})

# Use RobWork
SET(RobWork_DIR ${RW_ROOT}/cmake) 
FIND_PACKAGE(RobWork REQUIRED)
INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} include)
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

#--------------------   CREATE SHARED LIBRARIES IN C++ !!!   --------------------

# Include header files
#include_directories(include)

# Create shared library
#set(LIBRARY_SRC src/AnytimePlanning.cpp) ${LIBRARY_SRC}
add_library(AnytimePlanning SHARED src/AnytimePlanning)

# The shared library to build:
ADD_EXECUTABLE(planner src/main.cpp)
TARGET_LINK_LIBRARIES(planner AnytimePlanning ${ROBWORK_LIBRARIES})

#--------------------   CREATE SHARED LIBRARIES IN C++ !!!   --------------------

MESSAGE(${PROJECT_NAME} " done!")
```
## 7. Building and compiling in Linux.

Finally, to compile the program run the following commands in the terminal:
```sh
$ cd [path_to_project]/planner/build
$ cmake ../
$ make 
```

## 8. References
- [CodeBase](https://support.codebasehq.com/articles/tips-tricks/syntax-highlighting-in-markdown) (for witting markdown files)
- [LearnCpp](http://www.learncpp.com/cpp-tutorial/19-header-files/)
- [RobWork](http://www.robwork.dk/apidoc/nightly/rw/pageRobworkPrimer.html)
- [CMake](https://cmake.org/pipermail/cmake/2016-March/062927.html)

## 9. In addition...

The examples shown in the work perfectly, although they require some changes (like paths to fstream files) to make them work in your machine. They will also require RobWork software package to be installed.

- [RobWork](http://www.robwork.dk/apidoc/nightly/rw/page_rw_installation_ubuntu.html). Full installation, including RobWorkStudio, RobWorkSimulation and RobWorkHardware.
 
