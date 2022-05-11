#include <cmath>
#include <utility>

namespace turtlebot_planner {
class Vertex {
public:
  float x;

  float y;

  int idx;

  int parent;


  Vertex() {}

  Vertex(float x, float y, int idx, int parent);

  bool operator==(const Vertex& v) {
    return (x == v.x && y == v.y && parent == v.parent);
  }

  bool operator!=(const Vertex& v) {
    return (x != v.x || y != v.y || parent != v.parent);
  }
};
} 
