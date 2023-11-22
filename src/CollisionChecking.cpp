#include <vector>
#include <limits>
#include <math.h>
#include "CollisionChecking.h"

using namespace std;

double distanceToNearestObstacle(const std::vector<Rectangle> &obstacles, double x, double y) {
    double ans = std::numeric_limits<double>::max();
    for (auto o : obstacles) {
        auto closestX = std::max(o.x, std::min(x, o.x + o.width));
        auto closestY = std::max(o.y, std::min(y, o.y + o.height));
        ans = std::min(ans, sqrt(pow(x - closestX, 2) + pow(y - closestY, 2)));
    }
    return ans;
}

bool isValidCircle(double circleX, double circleY, double radius, const std::vector<Rectangle> &obstacles)
{
    for (const auto& obstacle : obstacles) {
        // Manually calculate the nearest X and Y coordinates on the obstacle
        double nearestX = (circleX < obstacle.x) ? obstacle.x : (circleX > obstacle.x + obstacle.width) ? obstacle.x + obstacle.width : circleX;
        double nearestY = (circleY < obstacle.y) ? obstacle.y : (circleY > obstacle.y + obstacle.height) ? obstacle.y + obstacle.height : circleY;

        double deltaX = circleX - nearestX;
        double deltaY = circleY - nearestY;
        double squaredDistance = deltaX * deltaX + deltaY * deltaY;
        double squaredRadius = radius * radius;

        if (squaredDistance <= squaredRadius) return false;
    }
    return true;
}