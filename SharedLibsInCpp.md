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
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <vector>
#include <math.h> 
#include <numeric>
#include <algorithm>
#include <iterator>
#include <boost/foreach.hpp>
#include <string>
#include <vector>
#include <rw/trajectory/CubicSplineFactory.hpp>

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

	cout << "WorkCell " << wcFile; //<< " and device " << deviceName << endl;

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


}// Load_WorkCell()


/* 
	In the main, after calling Load_Workcell(), use the following breaking condition:
	if(wc_found == false || dev_found == false)
		{return 0;}
*/


// Function that looks for collisions at a given state (Q).
bool AnytimePlanning::checkCollisions(const State &state, const CollisionDetector &detector, const Q &q) 
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
	if (!checkCollisions(device, state, detector, from))
		{return 0;}
	// Check for collisions at final configuration.
	if (!checkCollisions(device, state, detector, to))
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

	return interpolated_path;

} // get_trajectory()
```

## 5. Main.cpp

In the main program, it is just required to include the header to the newly created class. Then, we can start using the functions that we coded before. Here it is an exampe of how to use it:

```cpp
#include <iostream>
#include <fstream> 
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <vector>
#include <math.h> 
#include <numeric>
#include <algorithm>
#include <iterator>
#include <boost/foreach.hpp>
#include <string>
#include <vector>
#include <rw/trajectory/CubicSplineFactory.hpp>
#include <AnytimePlanning.h>

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


#define MAXTIME 100.
int main(int argc, char** argv) 
{
	rw::math::Math::seed();
	
	// LOAD WORKCELL
	const string wc_name = "/home/charlie/catkin_ws/src/ROVI2_Object_Avoidance/WorkCell_scenes/WorkStation_2/WC2_Scene.wc.xml";
	const string dev_name = "UR1";
	AnytimePlanning::Load_WorkCell(wc_name, dev_name);
	if(wc_found == false || dev_found == false)
		{return 0;}


	// INITIAL SETTINGS

	// Initial configuration defined		
	Q from_deg(6, -59.86, -102.00, -128.35, -44.79, 92.54, 2.04); // deg
	//Q from_deg(6, -50.85, -108.53, -112.61, -44.85, 84.21, 28.59); // deg
	Q from = from_deg*(3.14159265359/180); // rad
	// Goal configuration defined.
	Q to_deg(6,151.34, -96.45, -107.53, -120.87, 272.71, 2.02); // deg
	//Q to_deg(6, -99.47, -164.99, -12.12, -104.93, 84.24, 28.59); // deg
	Q to = to_deg*(3.14159265359/180); // rad

	cout << endl;
	cout << "	>> | Q_initial | = " << from << endl;
	cout << "	>> | Q_goal | = " << to << endl;
	cout << endl;	

	// Initiate state variable by default.
	State state = wc->getDefaultState(); 
	// Get Home position
	Q Q_home = device->getQ(state);
	// Set new state to Q_init position.
	device->setQ(from,state);

	//---------------------------------------------------------------

	// PLANNING

    double epsilon = 0.5; // RRT's extend parameter.
	rw::math::Q dq_start = rw::math::Q(6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	rw::math::Q dq_end = rw::math::Q(6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

	// Calculate path
	QPath rawPath = AnytimePlanning::get_path(wc, device, epsilon, state, from, to);
	// Interpolate path
	QPath smoothPath = AnytimePlanning::get_trajectory(rawPath, dq_start, dq_end);

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
INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

#--------------------   CREATE SHARED LIBRARIES IN C++ !!!   --------------------

# Include header files
include_directories(include)

# Create shared library
set(LIBRARY_SRC src/AnytimePlanning.cpp)
add_library(AnytimePlanning SHARED ${LIBRARY_SRC})

# The shared library to build:
ADD_EXECUTABLE(planner src/main.cpp ${SrcFiles})
TARGET_LINK_LIBRARIES(planner ${ROBWORK_LIBRARIES} AnytimePlanning)

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

