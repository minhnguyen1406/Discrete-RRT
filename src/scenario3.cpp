#include <iostream>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include "collision_checking.h"
#include <math.h> 

#define PI 3.14159265

void plan(int planner) {
    const double robotRadius = 2;
    const double timeout = 60;

    int robotCount = 4;

    // create state space and bounds
    // state is represented by {x1, y1, x2, y2, ... x{robotCount}, y{robotCount}}
    auto stateSpace = std::make_shared<ob::RealVectorStateSpace>(robotCount*2);
    ob::RealVectorBounds bounds(robotCount*2);
    bounds.setLow(0);
    bounds.setHigh(20);
    stateSpace->setBounds(bounds);

    // create simple setup object
    og::SimpleSetup setup(stateSpace);

    // set start and goal states
    ob::ScopedState<> startState(stateSpace), goalState(stateSpace);

    std::vector<double> start;
    std::vector<double> goal;

    double cx = 10, cy = 10, r = 9;
    
    for (int i = 0; i < robotCount; i++){
        double angle = (2 * i * PI) / float(robotCount);
        start.push_back(cx + (r*cos(angle)));
        start.push_back(cy + (r*sin(angle)));

        goal.push_back(cx + (r*cos(PI + angle)));
        goal.push_back(cy + (r*sin(PI + angle)));
    }
    
    startState = start;
    goalState = goal;
    setup.setStartAndGoalStates(startState, goalState, 0.1);

    std::vector<Rectangle> obstacles = {};

    // set state validity checker
    auto svc = [&robotRadius, &obstacles, &robotCount](const ob::State *state) {
        auto st = state->as<ob::RealVectorStateSpace::StateType>()->values;
        for (int i = 0; i < robotCount*2; i += 2) {
            auto robotX = st[i], robotY = st[i + 1];
            // check for collisions with obstacles
            if (distanceToNearestObstacle(obstacles, robotX, robotY) < robotRadius)
                return false;
            // check for collisions with other robots
            for (int j = i + 2; j < robotCount*2; j += 2) {
                auto otherX = st[j], otherY = st[j + 1];
                if (sqrt(pow(robotX - otherX, 2) + pow(robotY - otherY, 2)) < 2 * robotRadius)
                    return false;
            }
        }
        return true;
    };
    setup.setStateValidityChecker(svc);

    switch (planner) {
        case 1:
            setup.setPlanner(std::make_shared<og::RRT>(setup.getSpaceInformation()));
            break;
    }

    auto status = setup.solve(timeout);
    std::cout << status.asString() << std::endl;

    if (status) {
        setup.simplifySolution();
        setup.getSolutionPath().printAsMatrix(std::cout);
    }
}

int main() {
    int planner;
    do {
        std::cout << "Choose a planner: " << std::endl;
        std::cout << " (1) RRT" << std::endl;
        std::cout << " (2) dRRT" << std::endl;
        std::cin >> planner;
    } while (planner < 1 || planner > 2);
    plan(planner);
}