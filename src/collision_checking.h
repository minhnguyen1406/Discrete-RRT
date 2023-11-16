#ifndef DRRT_COLLISION_CHECKING_H
#define DRRT_COLLISION_CHECKING_H

struct Rectangle {
    double x, y, width, height;
};

double distanceToNearestObstacle(const std::vector<Rectangle>& obstacles, double x, double y);


#endif //DRRT_COLLISION_CHECKING_H
