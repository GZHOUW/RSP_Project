#include <turtlebot_planner/Vertex.hpp>

Vertex::Vertex(float x, float y, int idx, int parent){
    this->x = x;
    this->y = y;
    this->idx = idx;
    this->parent = parent;
}