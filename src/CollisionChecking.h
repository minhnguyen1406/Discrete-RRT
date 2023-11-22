#ifndef DRRT_COLLISION_CHECKING_H
#define DRRT_COLLISION_CHECKING_H

#include <vector>
#include <cmath>
#include <iostream>
#include <utility>
#include <array>
#include <algorithm>

struct Rectangle {
    double x, y, width, height;
};

double distanceToNearestObstacle(const std::vector<Rectangle>& obstacles, double x, double y);

bool isValidCircle(double x, double y, double radius, const std::vector<Rectangle> &obstacles);


#endif //DRRT_COLLISION_CHECKING_H
