#ifndef INCLUDE_turtlebot_planner_VERTEX_H_
#define INCLUDE_turtlebot_planner_VERTEX_H_

#include <cmath>
#include <utility>

namespace turtlebot_planner {
class Vertex {
 private:
     /**
      * @brief the x coordinate of the vertex
      */
     float x_;

     /**
      * @brief the y coordinate of the vertex
      */
     float y_;

     /**
      * @brief the index of the vertex
      */
     int index_;

     /**
      * @brief the vertex's parent index
      */
     int parent_index_;

 public:
     /**
      * @brief Simple Vertex constructor
      */
     Vertex() {}

     /**
      * @brief the constructor for a Vertex
      * @param x the x coordinate of the vertex
      * @param y the y coordinate of the vertex
      * @param index the index of the vertex
      * @param parent_index the index of the parent vertex
      */
     Vertex(float x, float y, int index, int parent_index);

     /**
      * @brief destructor
      */
     ~Vertex() {}

     /**
      * @brief sets the location
      * @param x x location of vertex
      * @param y y location of vertex
      */
     void set_location(float x, float y);

     /**
      * @brief sets the index of the vertex
      * @param index index of the vertex
      */
     void set_index(int index);

     /**
      * @brief sets the parent vertex
      * @param parent_index index of the parent vertex
      */
     void set_parent(int index);

     /**
      * @brief returns the x,y location of the vertex
      * @return returns std::pair<x,y>
      */
     std::pair<float, float> get_location();

     /**
      * @brief returns the index of the vertex
      * @return index of the vertex
      */
     int get_index();

     /**
      * @brief returns the index of the parent vertex
      * @return returns the index of the parent
      */
     int get_parent();

     /**
      * @brief overload of == operator
      */
       bool operator==(const Vertex& v) {
         return (x_ == v.x_ && y_ == v.y_ && parent_index_ == v.parent_index_);
       }

     /**
      * @brief overload of != operator
      */
     bool operator!=(const Vertex& v) {
       return (x_ != v.x_ || y_ != v.y_ || parent_index_ != v.parent_index_);
     }
};
}  // namespace turtlebot_planner

#endif  // INCLUDE_turtlebot_planner_VERTEX_H_
