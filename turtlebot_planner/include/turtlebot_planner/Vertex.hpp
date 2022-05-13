namespace turtlebot_planner {
  class Vertex {
  public:
    float x, y;

    int idx, parent;
    
    Vertex(float x, float y, int idx, int parent);
  };
} 
