
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

#define PI 3.14159265

const double radius = 1;


double distanceBetween(const vector<double> &point1, const vector<double> &point2) {
    return sqrt(pow(point1[0] - point2[0], 2) + pow(point1[1] - point2[1], 2));
}

bool isGroupStateCollisionFree(const ob::State *groupState, const vector<Rectangle> & barriers) {
    auto compoundState = groupState->as<ob::CompoundState>();
    vector<vector<double>> positions;

    for (int robotIndex = 0; robotIndex < 5; ++robotIndex) {
        auto robotState = compoundState->as<ob::RealVectorStateSpace::StateType>(robotIndex);
        double posX = robotState->values[0];
        double posY = robotState->values[1];
        if (!isValidCircle(posX, posY, radius, barriers)) return false;
        positions.emplace_back(vector<double>{posX, posY});
    }

    for (int i = 0; i < 5; ++i) {
        for (int j = i + 1; j < 5; ++j) {
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

og::SimpleSetup createRobots(vector<Rectangle> & obstacles,
        double** starts, double** goals)
{
    ob::StateSpacePtr compoundSpace;

    for (int i = 0; i < 5; i++)
    {
        auto r2 = make_shared<ob::RealVectorStateSpace>(2);
        ob::RealVectorBounds bounds(2);
        bounds.setLow(-5);
        bounds.setHigh(25);
        r2->setBounds(bounds);
        compoundSpace = compoundSpace + r2;
    }

    og::SimpleSetup setup(compoundSpace);

    setup.setStateValidityChecker(
        [obstacles](const ob::State *state) { return isGroupStateCollisionFree(state, obstacles); });

    ob::ScopedState<> start(compoundSpace);

    ob::ScopedState<> goal(compoundSpace);

    for (int i = 0; i < 2 * 5; i++)
    {
        start[i] = starts[i / 2][i % 2];
        goal[i] = goals[i / 2][i % 2];
        cerr << start[i] << endl;
    }

    setup.setStartAndGoalStates(start, goal, 2);
    return setup;
}

PRMptr setupPRM(vector<Rectangle> & obstacles, double** starts, double** goals)
{
    auto space(make_shared<ob::RealVectorStateSpace>(2));
    ob::RealVectorBounds bounds(2);

    bounds.setLow(-5);
    bounds.setHigh(25);

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
            setup.setPlanner(make_shared<og::DRRT>(setup.getSpaceInformation(), 5, PRMplanner));
            break;
    }

    setup.setup();
    ob::PlannerStatus solved;

    solved = setup.solve(7200.0);


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
    string benchmark_name = "Scenario 2 Benchmarking";
    double runtime_limit = 120.0, memory_limit = 10000.0;
    int run_count = 20;

    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count);
    ompl::tools::Benchmark b(setup, benchmark_name);

    b.addPlanner(make_shared<og::DRRT>(setup.getSpaceInformation(), 5, PRMplanner));

    b.benchmark(request);
    b.saveResultsToFile();
}

int main(int /* argc */, char ** /* argv */)
{
    vector<Rectangle> obstacles;
    double cx = 10, cy = 10, r = 9;
    
    double** starts = new double*[5];
    double** goals = new double*[5];

    for (int i = 0; i < 5; ++i) {
        double angle = (2 * i * PI) / float(5);
        starts[i] = new double[2];
        starts[i][0] = cx + (r*cos(angle));
        starts[i][1] = cy + (r*sin(angle));
        
        goals[i] = new double[2];
        goals[i][0] = cx + (r*cos(PI + angle));
        goals[i][1] = cy + (r*sin(PI + angle));
    }

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
    for (int i = 0; i < 5; ++i) {
        delete[] starts[i]; // Deallocate each double array
        delete[] goals[i];
    }
    delete[] starts;
    delete[] goals;
    return 0;
}