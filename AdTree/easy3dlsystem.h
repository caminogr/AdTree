#include <easy3d/core/graph.h>
using namespace easy3d;

class LSystem {
public:
    void generateLString(
      const easy3d::Graph& skel,
//      easy3d::Graph::Vertex rootV,
      double accuracy
    );

    void traversal(
      const easy3d::Graph& skel,
      easy3d::Graph::Vertex startV,
      easy3d::Graph::Vertex prevV,
      double accuracy

//      const easy3d::Graph& skeleton,
//      const easy3d::Graph& easy3d::Graph,
//      //dvec3 startV,
//      //dvec3 prevV,
//      Seasy3d::GraphVertexDescriptor startV,
//      Seasy3d::GraphVertexDescriptor prevV,
//      std::unordered_map<Seasy3d::GraphVertexDescriptor, easy3d::Graph::Vertex>  vvmap,
//      double accuracy
    );
    void writeLString(
      easy3d::Graph::Vertex startV,
      easy3d::Graph::Vertex nextV,
      easy3d::Graph::Vertex prevV,
//      Seasy3d::GraphVertexDescriptor startV,
//      Seasy3d::GraphVertexDescriptor nextV,
//      Seasy3d::GraphVertexDescriptor prevV,
      const easy3d::Graph& skel,
      double accuracy
    );
//
//    std::tuple<double, double, double> computeRelationBetweenNodes(
//      easy3d::Graph::Vertex startV,
//      easy3d::Graph::Vertex nextV,
//      easy3d::Graph::Vertex prevV,
//      const easy3d::Graph& skel,
//      double accuracy
//  );

    std::string LString;
    bool degrees = true; // Assuming this is a class member
};
