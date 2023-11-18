#ifndef DRRT_DRRT_H
#define DRRT_DRRT_H

#include "ompl/base/State.h"
#include "ompl/base/Planner.h"
#include "ompl/base/SpaceInformation.h"


namespace ompl::geometric {

    struct Node {
        ompl::base::State *state;
        std::vector<std::shared_ptr<Node>> neighbors;
        std::shared_ptr<Node> treeParent;

        explicit Node(ompl::base::State *state) : state(state) {}
    };

    class DRRT : public base::Planner {
    public:
        int nodesCount;  // total number of nodes to sample in the PRM
        int k;  // for PRM k-nearest neighbors

        explicit DRRT(const base::SpaceInformationPtr &si,
                      int nodesCount, int k) : base::Planner(si, "dRRT"), nodesCount(nodesCount), k(k) {}

        base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc) override;

        void getPlannerData(ompl::base::PlannerData &data) const override;

        void clear() override;

    private:
        static double euclideanDistance(ompl::base::State *st1, ompl::base::State *st2, unsigned int dim);
    };

}

#endif //DRRT_DRRT_H
