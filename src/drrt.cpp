#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/datastructures/NearestNeighborsLinear.h>
#include <ompl/geometric/PathGeometric.h>

#include "drrt.h"


ompl::base::PlannerStatus ompl::geometric::DRRT::solve(const ompl::base::PlannerTerminationCondition &ptc) {
    checkValidity();

    auto si = getSpaceInformation();
    auto problem = getProblemDefinition();

    // default sampler is ompl::base::UniformValidStateSampler
    auto sampler = si->allocStateSampler();

    auto startState = problem->getStartState(0);
    auto startNode = std::make_shared<Node>(startState);
    auto goalState = problem->getGoal()->as<ompl::base::GoalState>();

    // these are the PRM nodes
    std::vector<std::shared_ptr<Node>> nodes;
    for (int i = 0; i < nodesCount; i++) {
        auto state = si->allocState();
        sampler->sampleUniform(state);
        nodes.push_back(std::make_shared<Node>(state));
    }
    ompl::NearestNeighborsLinear<std::shared_ptr<Node>> nn;
    nn.setDistanceFunction([&si](const std::shared_ptr<Node> &a, const std::shared_ptr<Node> &b) {
        return euclideanDistance(a->state, b->state, si->getStateDimension());
    });
    for (const auto &node: nodes)
        nn.add(node);
    for (const auto &node: nodes)
        nn.nearestK(node, k, node->neighbors);

    std::set<std::shared_ptr<Node>> treeNodes = {startNode};
    bool isApproximate = true;
    std::shared_ptr<Node> approxSolution;
    while (!ptc()) {
        // this is qrand
        auto randState = si->allocState();
        sampler->sampleUniform(randState);

        // find qnear
        std::shared_ptr<Node> qnear;
        double qnearDistance = std::numeric_limits<double>::infinity();
        for (const auto &node: treeNodes) {
            auto d = euclideanDistance(randState, node->state, si->getStateDimension());
            if (d < qnearDistance) {
                qnear = node;
                qnearDistance = d;
            }
        }

        // TODO: step 3 (find qnew)
        std::shared_ptr<Node> qnew;
        double qnewAngle = std::numeric_limits<double>::infinity();
        for (auto node: qnear->neighbors) {
            // skip if the line from qnear to qnew is not collision-free
            if (!si->checkMotion(qnear->state, node->state)) continue;
            // double angle = ?
            // if (angle < qnewAngle)
            //     qnew = node
            //     qnewAngle = angle
            // ...
        }

        // TODO: step 4
        // set qnew's treeParent to qnear

        // TODO: check to see if we hit the goal
        // TODO: also get the approximate solution at some point
        double dist = 0.0;
        if (goalState->isSatisfied(qnew->state, &dist)) {
            isApproximate = false;
            break;
        }

        // free qrand, we don't need it anymore
        si->freeState(randState);
    }

    // collect the states in the solution path by backtracking from the final node
    auto solutionPath = std::make_shared<ompl::geometric::PathGeometric>(si);
    std::shared_ptr<Node> currNode = isApproximate ? approxSolution : nodes.back();
    while (currNode) {
        solutionPath->append(currNode->state);
        currNode = currNode->treeParent;
    }
    problem->addSolutionPath(solutionPath);
    return {true, isApproximate};
}

void ompl::geometric::DRRT::getPlannerData(ompl::base::PlannerData &data) const {
    Planner::getPlannerData(data);
}

void ompl::geometric::DRRT::clear() {
    Planner::clear();
}

double ompl::geometric::DRRT::euclideanDistance(ompl::base::State *st1, ompl::base::State *st2, unsigned int dim) {
    auto s1 = st1->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    auto s2 = st2->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    double sum = 0;
    for (int i = 0; i < dim; i++)
        sum += pow(s1[i] - s2[i], 2);
    return sqrt(sum);
}
