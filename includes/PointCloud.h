#ifndef POINTCLOUD_H
#define POINTCLOUD_H
#include <vector>
#include "Point.h"
class PointCloud
{
public:
  std::vector<Point> points;
  void addPoint(const glm::vec3 &position, const glm::vec3 &normal)
  {
    points.emplace_back(position, normal);
  }
};
#endif