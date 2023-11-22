///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 6
// Authors: Minh, Wesley, Ayush
//////////////////////////////////////

#include "DRRT.h"
#include <limits>
#include <cmath>
#include <queue>
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"


og::DRRT::DRRT(const base::SpaceInformationPtr &si, int robotCount, const PRMptr& PRMplanner):
    base::Planner(si, "DRRT"), robotCount_(robotCount), PRMplanner_(move(PRMplanner))
{
    specs_.directed = true;
}

void og::DRRT::freeMemory()
{
    if (nn_)
    {
        vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
        for (auto &motion : intermediateMotions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }

    }
}

og::DRRT::~DRRT()
{
    freeMemory();
}

void og::DRRT::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
    intermediateMotions.clear();
}

void og::DRRT::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}



double og::DRRT::euclideanDistance(const ob::RealVectorStateSpace::StateType *state1, const ob::RealVectorStateSpace::StateType *state2, int dim) {
    double sum = 0;
    for (int i = 0; i < dim; i++)
        sum += pow(state1->values[i] - state2->values[i], 2);
    return sqrt(sum);
}


// Expand the tree towards a given random state
og::DRRT::Motion* og::DRRT::expand(Motion* nearestMotion, base::State* randomState, base::State* newState, const og::PRM::Graph& g) {

    auto nearestState = nearestMotion->state;
    for (int i = 0; i < robotCount_; i++) {
        auto qnew = newState->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(i);
        auto qrand = randomState->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(i);
        auto qnear = nearestState->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(i);
        oracle(qrand, qnear, qnew, g);
    }

    if (!si_->isValid(newState) || localConnector(nearestState, newState).empty()) {
        return nullptr;
    }

    auto *motion = new Motion(si_);
    si_->copyState(motion->state, newState);
    motion->parent = nearestMotion;
    nn_->add(motion);
    return motion;
}

void og::DRRT::moveRobotsToNearestInitialStates(const og::PRM::Graph& g, const base::State* state) {
    auto compoundState = state->as<ob::CompoundState>();
    boost::property_map<og::PRM::Graph, og::PRM::vertex_state_t>::const_type state_map = boost::get(og::PRM::vertex_state_t(), g);

    for (int i = 0; i < robotCount_; i++) {
        auto robotState = compoundState->as<ob::RealVectorStateSpace::StateType>(i);
        double minDist = numeric_limits<double>::infinity();
        ob::RealVectorStateSpace::StateType* closestState = nullptr;

        for (auto v : boost::make_iterator_range(vertices(g))) {
            auto candidateState = get(state_map, v)->as<ob::RealVectorStateSpace::StateType>();
            double dist = euclideanDistance(robotState, candidateState, si_->getStateDimension());
            if (dist < minDist) {
                minDist = dist;
                closestState = candidateState;
            }
        }

        if (closestState) {
            robotState->values[0] = closestState->values[0];
            robotState->values[1] = closestState->values[1];
        }
    }
}

vector<og::DRRT::Motion*> og::DRRT::constructSolutionPath(Motion* goalMotion) {
    vector<Motion*> mpath;
    Motion* currentMotion = goalMotion;
    auto tempState = si_->allocState();
    while (currentMotion != nullptr) {
        mpath.push_back(currentMotion);

        if (currentMotion->parent != nullptr) {
            auto localPath = localConnector(currentMotion->parent->state, currentMotion->state);
            for (int i = localPath.size() - 2; i >= 0; --i) {

                si_->copyState(tempState, currentMotion->parent->state);
                PRMplanner_->getSpaceInformation()->copyState(
                    tempState->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(localPath[i]),
                    currentMotion->parent->state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(localPath[i])
                );
                auto tempMotion = new Motion(si_);
                si_->copyState(tempMotion->state, tempState);
                mpath.push_back(tempMotion);
                intermediateMotions.push_back(tempMotion);
            }
        }
        currentMotion = currentMotion->parent;
    }

    lastGoalMotion_ = goalMotion;
    si_->freeState(tempState);
    return mpath;
}

ob::PlannerStatus og::DRRT::solve(const base::PlannerTerminationCondition &ptc)
{

    checkValidity();
    const og::PRM::Graph& g = PRMplanner_->getRoadmap();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);
    const base::State *st = pis_.nextStart();

    moveRobotsToNearestInitialStates(g, st);

    auto *motion = new Motion(si_);
    si_->copyState(motion->state, st);
    nn_->add(motion);
    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = numeric_limits<double>::infinity();
    auto *rmotion = new Motion(si_);
    base::State *randomState = rmotion->state;
    base::State *newState = si_->allocState();
    while (!ptc) {
       if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(randomState);
       else
            sampler_->sampleUniform(randomState);

        Motion *newMotion = expand(nn_->nearest(rmotion), randomState, newState, g);
        if (newMotion == nullptr) {
            continue;
        }

        double dist = 0.0;
        if (goal->isSatisfied(newMotion->state, &dist))
        {
            approxdif = dist;
            solution = newMotion;
            break;
        }
        if (dist < approxdif)
        {
            approxdif = dist;
            approxsol = newMotion;
        }
    }

    bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;

        /* construct the solution path */
        auto mpath = constructSolutionPath(solution);

        /* set the solution path */
        auto path(make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
        solved = true;
    }

    si_->freeState(newState);
    if (rmotion->state != nullptr)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return {solved, approximate};

}

// Expansion oracle
void og::DRRT::oracle(ob::RealVectorStateSpace::StateType*& qrand, ob::RealVectorStateSpace::StateType*& qnear, ob::RealVectorStateSpace::StateType*& qnew,
                        const og::PRM::Graph& g)
{

    auto stateMap = boost::get(og::PRM::vertex_state_t(), g);

    double minAngle = numeric_limits<double>::infinity();

    // Iterate over all vertices in the graph to find the vertex that corresponds to qnear.
    for (auto v : boost::make_iterator_range(vertices(g))) {
        auto vState = boost::get(stateMap, v)->as<ob::RealVectorStateSpace::StateType>();

        if (vState->values[0] == qnear->values[0] && vState->values[1] == qnear->values[1]) {

            for (auto av : boost::make_iterator_range(adjacent_vertices(v, g))) {
                auto avState = boost::get(stateMap, av)->as<ob::RealVectorStateSpace::StateType>();

                // Calculate the direction vectors.
                double dir_qrand[2] = { qrand->values[0] - qnear->values[0], qrand->values[1] - qnear->values[1] };
                double dir_av[2] = { avState->values[0] - qnear->values[0], avState->values[1] - qnear->values[1] };

                // Calculate the angle using the dot product and magnitude of vectors.
                double dot = dir_qrand[0] * dir_av[0] + dir_qrand[1] * dir_av[1];
                double mag_qrand = sqrt(dir_qrand[0] * dir_qrand[0] + dir_qrand[1] * dir_qrand[1]);
                double mag_av = sqrt(dir_av[0] * dir_av[0] + dir_av[1] * dir_av[1]);
                double angle = acos(dot / (mag_qrand * mag_av));

                // Check if this angle is the smallest we have found so far.
                if (angle < minAngle) {
                    minAngle = angle;
                    qnew->values[0] = avState->values[0];
                    qnew->values[1] = avState->values[1];
                }
            }
            break;
        }
    }

    // If no adjacent state was closer, qnew will not be updated and should be set to nullptr.
    if (minAngle == numeric_limits<double>::max()) {
        qnew = nullptr;
    }
}

bool og::DRRT::isCollisionFreePath(const ob::State* start, const ob::State* end, const ob::State* obstacleState) {
    int numDivisions = PRMplanner_->getSpaceInformation()->getStateSpace()->validSegmentCount(start, end);

    if (numDivisions < 2) {
        return true; // No need to check if there's not enough division
    }

    ob::State *interpolatedState = PRMplanner_->getSpaceInformation()->allocState();
    for (int i = 1; i < numDivisions; ++i) {
        PRMplanner_->getSpaceInformation()->getStateSpace()->interpolate(start, end, static_cast<double>(i) / numDivisions, interpolatedState);

        auto interpolatedR2State = interpolatedState->as<ob::RealVectorStateSpace::StateType>();
        auto obstacleR2State = obstacleState->as<ob::RealVectorStateSpace::StateType>();

        double distance = sqrt(pow(interpolatedR2State->values[0] - obstacleR2State->values[0], 2) +
                                    pow(interpolatedR2State->values[1] - obstacleR2State->values[1], 2));

        if (distance <= 2 * 1) {
            PRMplanner_->getSpaceInformation()->freeState(interpolatedState);
            return false; // Collision detected
        }
    }

    PRMplanner_->getSpaceInformation()->freeState(interpolatedState);
    return true; // No collision detected
}



vector<int> og::DRRT::localConnector(const ob::State* start, const ob::State* end) {
    // Initialize the adjacency list for the graph representation
    vector<vector<int>> dependencyGraph(robotCount_);
    vector<int> sortedOrder; // This will store the topological sort order of the robots
    vector<bool> onStack(robotCount_, false), visited(robotCount_, false);

    auto compoundStart = start->as<ob::CompoundState>();
    auto compoundEnd = end->as<ob::CompoundState>();

    // Build the graph based on collision checks
    for (int robotA = 0; robotA < robotCount_; ++robotA) {
        for (int robotB = 0; robotB < robotCount_; ++robotB) {
            if (robotA == robotB) continue;  // Skip self-comparison

            auto startStateA = compoundStart->as<ob::RealVectorStateSpace::StateType>(robotA);
            auto endStateA = compoundEnd->as<ob::RealVectorStateSpace::StateType>(robotA);
            auto startStateB = compoundStart->as<ob::RealVectorStateSpace::StateType>(robotB);
            auto endStateB = compoundEnd->as<ob::RealVectorStateSpace::StateType>(robotB);

            // Determine dependencies between robots based on potential collisions
            if (!isCollisionFreePath(startStateA, endStateA, startStateB)) {
                dependencyGraph[robotB].push_back(robotA);  // Robot B should move before Robot A
            }
            if (!isCollisionFreePath(startStateA, endStateA, endStateB)) {
                dependencyGraph[robotA].push_back(robotB);  // Robot A should move before Robot B
            }
        }
    }

    for (int robot = 0; robot < robotCount_; ++robot) {
        if (!visited[robot]) {
            bool cycleDetected = topoSort(robot, dependencyGraph, onStack, visited, sortedOrder);
            if (cycleDetected) {
                return {}; // If a cycle is detected, return an empty vector
            }
        }
    }

    reverse(sortedOrder.begin(), sortedOrder.end()); // reverse to get correct order
    return sortedOrder;
}

bool og::DRRT::topoSort(int robot, const vector<vector<int>>& adjacencyList, vector<bool>& onStack, vector<bool>& visited, vector<int>& sortedOrder) {
    onStack[robot] = true;
    visited[robot] = true;

    for (int neighbor : adjacencyList[robot]) {
        if (!visited[neighbor]) {
            if (topoSort(neighbor, adjacencyList, onStack, visited, sortedOrder)) {
                return true; // Cycle detected
            }
        } else if (onStack[neighbor]) {
            return true; // Cycle detected
        }
    }

    onStack[robot] = false; // Mark the robot as not on the current DFS path
    sortedOrder.push_back(robot); // Add the robot to the topological order
    return false; // No cycle detected
}


void og::DRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    if (lastGoalMotion_ != nullptr)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
    }
}



