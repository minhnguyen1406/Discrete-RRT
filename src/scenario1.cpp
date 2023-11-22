
#include <iostream>
#include <cmath>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>
#include <boost/tuple/tuple.hpp>
#include <ompl/tools/benchmark/Benchmark.h>
#include "CollisionChecking.h"
#include <ompl/geometric/planners/rrt/RRT.h>
#include "DRRT.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;


const double radius = 1;

void createObstacle(vector<Rectangle> &obstacles, double x, double y, double width, double height)
{
    Rectangle rect;
    rect.x = x;
    rect.y = y;
    rect.width = width;
    rect.height = height;
    obstacles.push_back(rect);
}


double distanceBetween(const std::vector<double> &point1, const std::vector<double> &point2) {
    return std::sqrt(std::pow(point1[0] - point2[0], 2) + std::pow(point1[1] - point2[1], 2));
}

bool isGroupStateCollisionFree(const ob::State *groupState, const std::vector<Rectangle> & barriers) {
    auto compoundState = groupState->as<ob::CompoundState>();
    std::vector<std::vector<double>> positions;

    for (int robotIndex = 0; robotIndex < 4; ++robotIndex) {
        auto robotState = compoundState->as<ob::RealVectorStateSpace::StateType>(robotIndex);
        double posX = robotState->values[0];
        double posY = robotState->values[1];
        if (!isValidCircle(posX, posY, radius, barriers)) return false;
        positions.emplace_back(std::vector<double>{posX, posY});
    }

    for (int i = 0; i < 4; ++i) {
        for (int j = i + 1; j < 4; ++j) {
            double dist = distanceBetween(positions[i], positions[j]);
            if (dist <= 2 * radius) return false;
        }
    }
    return true;
}

bool isSingleRobotStateValid(const ob::State *robotState, const std::vector<Rectangle> & barriers) {
    auto singleState = robotState->as<ob::RealVectorStateSpace::StateType>();
    return isValidCircle(singleState->values[0], singleState->values[1], radius, barriers);
}

void makeObstacles(std::vector<Rectangle> & obstacles)
{
    createObstacle(obstacles, 0, -1, 20,  1);
    createObstacle(obstacles, -1, 0,  1,  20);
    createObstacle(obstacles, 0,  20, 20,  1);
    createObstacle(obstacles, 20, 0,  1,   20);
    createObstacle(obstacles, 0,  5,  7.5, 10);
    createObstacle(obstacles,12.5, 5,  7.5, 10);
}


std::pair<double**, double**> setStartGoal()
{
    double** starts = new double*[4];
    double** goals = new double*[4];
    std::pair<double**, double**> res;

    starts[0] = new double[2]{2.5, 2.5};
    starts[1] = new double[2]{17.5, 2.5};
    starts[2] = new double[2]{2.5, 17.5};
    starts[3] = new double[2]{17.5, 17.5};
    goals[0] = new double[2]{17.5, 17.5};
    goals[1] = new double[2]{2.5, 17.5};
    goals[2] = new double[2]{17.5, 2.5};
    goals[3] = new double[2]{2.5, 2.5};

    res.first = starts;
    res.second = goals;
;
    return res;
}

og::SimpleSetupPtr createMultRob(std::vector<Rectangle> & obstacles,
        double** starts, double** goals)
{
    ob::StateSpacePtr compSS;

    for (int i = 0; i < 4; i++)
    {
        auto r2 = std::make_shared<ob::RealVectorStateSpace>(2);
        ob::RealVectorBounds bounds(2);
        bounds.setLow(0);
        bounds.setHigh(20);
        r2->setBounds(bounds);
        compSS = compSS + r2;
    }

    auto ssptr(std::make_shared<og::SimpleSetup>(compSS));

    ssptr->setStateValidityChecker(
        [obstacles](const ob::State *state) { return isGroupStateCollisionFree(state, obstacles); });

    ob::ScopedState<> start(compSS);

    ob::ScopedState<> goal(compSS);
    
    for (int i = 0; i < 2 * 4; i++)
    {
        start[i] = starts[i / 2][i % 2];
        goal[i] = goals[i / 2][i % 2];
    }


    ssptr->setStartAndGoalStates(start, goal, 0.20);
    return ssptr;
}

PRMptr setupPRM(std::vector<Rectangle> & obstacles, double** starts, double** goals)
{
    auto space(std::make_shared<ob::RealVectorStateSpace>(2));
    ob::RealVectorBounds bounds(2);

    bounds.setLow(0);
    bounds.setHigh(20);

    space->setBounds(bounds);
    auto ssptr(std::make_shared<og::SimpleSetup>(space));
    ssptr->setStateValidityChecker(
        [obstacles](const ob::State *state) { return isSingleRobotStateValid(state, obstacles); });
    ompl::base::ScopedState<> start(space);
    ompl::base::ScopedState<> goal(space);
    start[0] = starts[0][0];
    start[1] = starts[0][1];
    goal[0] = goals[0][0];
    goal[1] = goals[0][1];
    ssptr->setStartAndGoalStates(start, goal);
    PRMptr PRMplanner(std::make_shared<og::PRM>(ssptr->getSpaceInformation()));
    ssptr->setPlanner(PRMplanner);
    ssptr->setup();
    ob::PlannerStatus solved = ssptr->solve(10.0);

    if (!solved)
    {
        std::cerr << "No valid PRM roadmap found" << std::endl;
        
    }
    og::PRM::Graph roadmap = PRMplanner->getRoadmap();
    
    return PRMplanner;
}

void plan(og::SimpleSetupPtr & ss, const PRMptr& PRMplanner, int planner)
{

    switch (planner) {
        case 1:
            ss->setPlanner(std::make_shared<og::RRT>(ss->getSpaceInformation()));
            break;
        case 2:
            auto planner = std::make_shared<og::DRRT>(ss->getSpaceInformation(), 4, PRMplanner);
            ss->setPlanner(planner);
            break;
    }

    ss->setup();
    ob::PlannerStatus solved;

    solved = ss->solve(120.0);


    if (solved)
    {

        std::cout << "Found solution:" << std::endl;
        ss->getSolutionPath().printAsMatrix(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;
}

int main(int /* argc */, char ** /* argv */)
{
    std::vector<Rectangle> obstacles;
    makeObstacles(obstacles);
    int planner;
    do {
        std::cout << "Choose a planner: " << std::endl;
        std::cout << " (1) RRT" << std::endl;
        std::cout << " (2) dRRT" << std::endl;
        std::cin >> planner;
    } while (planner < 1 || planner > 2);
    
    auto startGoal = setStartGoal();
    og::SimpleSetupPtr ss = nullptr;
    
    PRMptr PRMplanner = setupPRM(obstacles, startGoal.first, startGoal.second);
    
    ss = createMultRob(obstacles, startGoal.first, startGoal.second);
    plan(ss, PRMplanner, planner);


    return 0;
}