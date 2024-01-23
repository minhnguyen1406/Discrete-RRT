///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 6
// Authors: Minh, Wesley, Ayush
//////////////////////////////////////

#ifndef DRRT_DRRT_H
#define DRRT_DRRT_H
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include "ompl/datastructures/NearestNeighbors.h"
#include <vector>
#include "ompl/geometric/planners/PlannerIncludes.h"

using namespace std;
namespace ob = ompl::base;
namespace og = ompl::geometric;
using PRMptr = shared_ptr<og::PRM>;

namespace ompl
{
    namespace geometric
    {
        class DRRT : public base::Planner
        {
        public:
            DRRT(const base::SpaceInformationPtr &si, int robotCount, const PRMptr& PRMplanner);
            //DRRT(const base::SpaceInformationPtr &si, int robotCount, const og::PRM* PRMplanner);

            ~DRRT() override;

            void clear() override;

            void setup() override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void getPlannerData(base::PlannerData &data) const override;

        protected:
            class Motion
            {
            public:
                Motion() = default;

                Motion(const base::SpaceInformationPtr &si) : state(si->allocState())
                {
                }

                ~Motion() = default;

                base::State *state{nullptr};

                Motion *parent{nullptr};
            };

            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            shared_ptr<NearestNeighbors<Motion *>> nn_;

            void freeMemory();

            base::StateSamplerPtr sampler_;

            int robotCount_;

            const PRMptr PRMplanner_;
            //const og::PRM* PRMplanner_;

            double goalBias_{.05};

            //double maxDistance_{0.};

            RNG rng_;

            //og::PRM::Graph g_;
            Motion *lastGoalMotion_{nullptr};
            vector<Motion*> intermediateMotions;

            void oracle(ob::RealVectorStateSpace::StateType*& qrand, ob::RealVectorStateSpace::StateType*& qnear, ob::RealVectorStateSpace::StateType*& qnew, const og::PRM::Graph& g);

            bool isCollisionFreePath(const ob::State* start, const ob::State* end, const ob::State* otherRobotStartState);

            vector<int> localConnector(const ob::State* start, const ob::State* end);

            double euclideanDistance(const ob::RealVectorStateSpace::StateType* state1, const ob::RealVectorStateSpace::StateType* state2, int dim);

            void moveRobotsToNearestInitialStates(const og::PRM::Graph& g, const base::State* state);

            ob::RealVectorStateSpace::StateType* findNearestStateInGraph(const og::PRM::Graph& g,ob::RealVectorStateSpace::StateType* queryState);

            void updateRobotState(ob::RealVectorStateSpace::StateType* robotState,ob::RealVectorStateSpace::StateType* newState);

            Motion* expand(Motion* nearestMotion, base::State* randomState, base::State* newState, const og::PRM::Graph& g);

            vector<Motion*> constructSolutionPath(Motion* goalMotion);

            bool topoSort(int robot, const vector<vector<int>>& adjacencyList, vector<bool>& onStack, vector<bool>& visited, vector<int>& sortedOrder);
        };

    }  // namespace geometric
}  // namespace ompl
#endif //DRRT_DRRT_H
