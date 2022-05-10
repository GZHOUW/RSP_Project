
namespace turtlebot_planner{
    class Vertex{

    public:

        float x;
        float y;
        int idx;
        int parent;

        Vertex(float x, float y, int idx, int parent);
    }; 
}
