#ifndef LSystem_H
#define LSystem_H

#include <string>
#include "skeleton.h"



class LSystem
{
public:
    LSystem();
    ~LSystem();

    bool degrees = true;
    void generateLString(const Graph& skel, SGraphVertexDescriptor startV,  double accuracy);

    std::string getLString() const { return LString; }

private:
    std::string LString;

    void traversal(
      const Graph& skel,
      SGraphVertexDescriptor startV,
      SGraphVertexDescriptor prevV,
      double accuracy
    );
    void writeLString(
      SGraphVertexDescriptor startV,
      SGraphVertexDescriptor nextV,
      SGraphVertexDescriptor prevV,
      const Graph& skel,
      double accuracy
    );
};

#endif
