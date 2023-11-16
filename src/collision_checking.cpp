#include <vector>
#include "collision_checking.h"


double distanceToNearestObstacle(const std::vector<Rectangle> &obstacles, double x, double y) {
    double ans = std::numeric_limits<double>::max();
    for (auto o : obstacles) {
        auto closestX = std::max(o.x, std::min(x, o.x + o.width));
        auto closestY = std::max(o.y, std::min(y, o.y + o.height));
        ans = std::min(ans, sqrt(pow(x - closestX, 2) + pow(y - closestY, 2)));
    }
    return ans;
}