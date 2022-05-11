#include "turtlebot_planner/Vertex.hpp"

namespace turtlebot_planner {
  Vertex::Vertex(float x, float y, int index, int parent_index) {
    x_ = x;
    y_ = y;
    index_ = index;
    parent_index_ = parent_index;
  }

  void Vertex::set_location(float x, float y) {
    x_ = x;
    y_ = y;
  }

  void Vertex::set_index(int index) {
    index_ = index;
  }

  void Vertex::set_parent(int parent_index) {
    parent_index_ = parent_index;
  }

  std::pair<float, float> Vertex::get_location() {
    return std::pair<float, float>(x_, y_);
  }

  int Vertex::get_index() {
    return index_;
  }

  int Vertex::get_parent() {
    return parent_index_;
  }
}  // namespace turtlebot_planner
