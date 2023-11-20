#include <iostream>

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include "collision_checking.h"


void plan(int planner) {
    const double robotRadius = 2;
    const double timeout = 60;

    // create state space and bounds
    // state is represented by {x1, y1, x2, y2, ... x8, y8}
    auto stateSpace = std::make_shared<ompl::base::RealVectorStateSpace>(8);
    ompl::base::RealVectorBounds bounds(8);
    bounds.setLow(0);
    bounds.setHigh(20);
    stateSpace->setBounds(bounds);

    // create simple setup object
    ompl::geometric::SimpleSetup setup(stateSpace);

    // set start and goal states
    ompl::base::ScopedState<> startState(stateSpace), goalState(stateSpace);
    std::vector<double> start{2.5, 2.5, 17.5, 2.5, 2.5, 17.5, 17.5, 17.5};
    std::vector<double> goal{17.5, 17.5, 2.5, 17.5, 17.5, 2.5, 2.5, 2.5};
    startState = start;
    goalState = goal;
    setup.setStartAndGoalStates(startState, goalState, 0.1);

    std::vector<Rectangle> obstacles = {
            {0,    -1, 20,  1},
            {-1,   0,  1,   20},
            {0,    20, 20,  1},
            {20,   0,  1,   20},
            {0,    5,  7.5, 10},
            {12.5, 5,  7.5, 10}
    };

    // set state validity checker
    auto svc = [&robotRadius, &obstacles](const ompl::base::State *state) {
        auto st = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        for (int i = 0; i < 8; i += 2) {
            auto robotX = st[i], robotY = st[i + 1];
            // check for collisions with obstacles
            if (distanceToNearestObstacle(obstacles, robotX, robotY) < robotRadius)
                return false;
            // check for collisions with other robots
            for (int j = i + 2; j < 8; j += 2) {
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
            setup.setPlanner(std::make_shared<ompl::geometric::RRT>(setup.getSpaceInformation()));
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