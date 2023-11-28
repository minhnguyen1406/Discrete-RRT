
#include <iostream>
#include <cmath>

#include "CollisionChecking.h"
#include "DRRT.h"
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/config.h>
#include <ompl/tools/benchmark/Benchmark.h>

const double radius = 2;

void createObstacle(vector<Rectangle> &obstacles, double x, double y, double width, double height)
{
    Rectangle rect;
    rect.x = x;
    rect.y = y;
    rect.width = width;
    rect.height = height;
    obstacles.push_back(rect);
}


double distanceBetween(const vector<double> &point1, const vector<double> &point2) {
    return sqrt(pow(point1[0] - point2[0], 2) + pow(point1[1] - point2[1], 2));
}

bool isGroupStateCollisionFree(const ob::State *groupState, const vector<Rectangle> & barriers) {
    auto compoundState = groupState->as<ob::CompoundState>();
    vector<vector<double>> positions;

    for (int robotIndex = 0; robotIndex < 4; ++robotIndex) {
        auto robotState = compoundState->as<ob::RealVectorStateSpace::StateType>(robotIndex);
        double posX = robotState->values[0];
        double posY = robotState->values[1];
        if (!isValidCircle(posX, posY, radius, barriers)) return false;
        positions.emplace_back(vector<double>{posX, posY});
    }

    for (int i = 0; i < 4; ++i) {
        for (int j = i + 1; j < 4; ++j) {
            double dist = distanceBetween(positions[i], positions[j]);
            if (dist <= 2 * radius) return false;
        }
    }
    return true;
}

bool isSingleRobotStateValid(const ob::State *robotState, const vector<Rectangle> & barriers) {
    auto singleState = robotState->as<ob::RealVectorStateSpace::StateType>();
    return isValidCircle(singleState->values[0], singleState->values[1], radius, barriers);
}

void makeObstacles(vector<Rectangle> & obstacles)
{
    createObstacle(obstacles, 0,  5,  7, 10);
    createObstacle(obstacles,12.5, 5,  7, 10);
}

og::SimpleSetup createRobots(vector<Rectangle> & obstacles,
        double** starts, double** goals)
{
    ob::StateSpacePtr compoundSpace;

    for (int i = 0; i < 4; i++)
    {
        auto r2 = make_shared<ob::RealVectorStateSpace>(2);
        ob::RealVectorBounds bounds(2);
        bounds.setLow(0);
        bounds.setHigh(20);
        r2->setBounds(bounds);
        compoundSpace = compoundSpace + r2;
    }

    og::SimpleSetup setup(compoundSpace);

    setup.setStateValidityChecker(
        [obstacles](const ob::State *state) { return isGroupStateCollisionFree(state, obstacles); });

    ob::ScopedState<> start(compoundSpace);

    ob::ScopedState<> goal(compoundSpace);

    for (int i = 0; i < 2 * 4; i++)
    {
        start[i] = starts[i / 2][i % 2];
        goal[i] = goals[i / 2][i % 2];
    }

    setup.setStartAndGoalStates(start, goal, 4);
    return setup;
}

PRMptr setupPRM(vector<Rectangle> & obstacles, double** starts, double** goals)
{
    auto space(make_shared<ob::RealVectorStateSpace>(2));
    ob::RealVectorBounds bounds(2);

    bounds.setLow(0);
    bounds.setHigh(20);

    space->setBounds(bounds);
    og::SimpleSetup setup(space);
    setup.setStateValidityChecker(
        [obstacles](const ob::State *state) { return isSingleRobotStateValid(state, obstacles); });
    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space);
    start[0] = starts[0][0];
    start[1] = starts[0][1];
    goal[0] = goals[0][0];
    goal[1] = goals[0][1];
    setup.setStartAndGoalStates(start, goal);
    PRMptr PRMplanner(make_shared<og::PRM>(setup.getSpaceInformation()));
    setup.setPlanner(PRMplanner);
    setup.setup();
    ob::PlannerStatus solved = setup.solve(10.0);

    if (!solved)
    {
        cerr << "No valid PRM roadmap found" << endl;

    }

    return PRMplanner;
}

void plan(og::SimpleSetup & setup, const PRMptr& PRMplanner, int planner)
{

    switch (planner) {
        case 1:
            setup.setPlanner(make_shared<og::RRT>(setup.getSpaceInformation()));
            break;
        case 2:
            setup.setPlanner(make_shared<og::DRRT>(setup.getSpaceInformation(), 4, PRMplanner));
            break;
    }

    setup.setup();
    ob::PlannerStatus solved;

    solved = setup.solve(300.0);


    if (solved)
    {

        cout << "Found solution:" << endl;
        setup.getSolutionPath().printAsMatrix(cout);
    }
    else
        cout << "No solution found" << endl;
}

void benchmark(og::SimpleSetup setup, const PRMptr& PRMplanner)
{
    string benchmark_name = "Scenario 1 Benchmarking";
    double runtime_limit = 300.0, memory_limit = 10000.0;
    int run_count = 20;

    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count);
    ompl::tools::Benchmark b(setup, benchmark_name);

    b.addPlanner(make_shared<og::DRRT>(setup.getSpaceInformation(), 4, PRMplanner));

    b.benchmark(request);
    b.saveResultsToFile();
}

int main(int /* argc */, char ** /* argv */)
{
    vector<Rectangle> obstacles;
    makeObstacles(obstacles);
    double** starts = new double*[4];
    double** goals = new double*[4];

    starts[0] = new double[2]{2.5, 2.5};
    starts[1] = new double[2]{17.5, 2.5};
    starts[2] = new double[2]{2.5, 17.5};
    starts[3] = new double[2]{17.5, 17.5};
    goals[0] = new double[2]{17.5, 17.5};
    goals[1] = new double[2]{2.5, 17.5};
    goals[2] = new double[2]{17.5, 2.5};
    goals[3] = new double[2]{2.5, 2.5};


//    starts[0] = new double[2]{0.15, 1.15};
//    starts[1] = new double[2]{1.15, 1.15};
//    starts[2] = new double[2]{0.15, 0.15};
//    starts[3] = new double[2]{1.15, 0.15};
//    goals[0] = new double[2]{1.15, 0.15};
//    goals[1] = new double[2]{0.15, 0.15};
//    goals[2] = new double[2]{1.15, 1.15};
//    goals[3] = new double[2]{0.15, 1.15};
    int choice;
    do
    {
        cout << "Plan or Benchmark? " << endl;
        cout << " (1) Plan" << endl;
        cout << " (2) Benchmark" << endl;

        cin >> choice;
    } while (choice < 1 || choice > 2);

    auto setup = createRobots(obstacles, starts, goals);
    auto PRMplanner = setupPRM(obstacles, starts, goals);
    if (choice == 1) {
        int planner;
        do {
            cout << "Choose a planner: " << endl;
            cout << " (1) RRT" << endl;
            cout << " (2) dRRT" << endl;
            cin >> planner;
        } while (planner < 1 || planner > 2);

        plan(setup, PRMplanner, planner);
    }
    else if (choice == 2)
        benchmark(setup, PRMplanner);
    else
        cerr << "How did you get here? Invalid choice." << endl;
    for (int i = 0; i < 4; ++i) {
        delete[] starts[i]; // Deallocate each double array
        delete[] goals[i];
    }
    delete[] starts;
    delete[] goals;
    return 0;
}