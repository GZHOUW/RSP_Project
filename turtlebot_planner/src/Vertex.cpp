#include "turtlebot_planner/Vertex.hpp"

namespace turtlebot_planner {
  Vertex::Vertex(float x, float y, int idx, int parent) {
    this->x = x;
    this->y = y;
    this->idx = idx;
    this->parent = parent;
  }
}
