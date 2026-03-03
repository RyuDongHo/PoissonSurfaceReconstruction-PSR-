#ifndef POINT_H
#define POINT_H

#include <glm/glm.hpp>

class Point
{
public:
  glm::vec3 position;
  glm::vec3 normal;
  Point(const glm::vec3 &pos, const glm::vec3 &norm)
      : position(pos), normal(norm) {}
};

#endif