#include "lsystem.h"

#include <fstream>
#include <cmath>

vec3 rotateAroundZ(const vec3&vec , double angle) {
  double cosAngle = std::cos(angle);
  double sinAngle = std::sin(angle);
  return easy3d::mat3(
    cosAngle, -sinAngle, 0,
    sinAngle, cosAngle, 0,
    0, 0, 1
  ) * vec;
}

std::tuple<double, double, double> computeRelationBetweenNodes(
  easy3d::Graph::Vertex startV,
  easy3d::Graph::Vertex nextV,
  easy3d::Graph::Vertex prevV,
  const easy3d::Graph& skel,
  double accuracy
) {
   // 座標の取得
    vec3 coords_start = skel.position(startV);;
    vec3 coords_next = skel.position(nextV);
    vec3 coords_prev = skel.position(prevV);

    // ベクトルの計算
    vec3 vector_to_nextV = coords_next - coords_start;
    vec3 vector_to_startV = coords_start - coords_prev;

  /* auto vector_to_nextV = std::make_tuple(coords_next.x - coords_start.x, coords_next.y - coords_start.y, coords_next.z - coords_start.z); */
  /* auto vector_to_startV = std::make_tuple(coords_start.x - coords_prev.x, coords_start.y - coords_prev.y, coords_start.z - coords_prev.z); */

  /* double branch_length = std::sqrt( */
  /*   std::pow(std::get<0>(vector_to_nextV), 2) + */
  /*   std::pow(std::get<1>(vector_to_nextV), 2) + */
  /*   std::pow(std::get<2>(vector_to_nextV), 2) */
  /* ); */


    // 枝の長さの計算
    double branch_length = vector_to_nextV.length();

 // Project vectors to XY plane
    /* auto projected_vector_to_nextV = std::make_tuple(std::get<0>(vector_to_nextV), std::get<1>(vector_to_nextV), 0); */
    /* auto projected_vector_to_startV = std::make_tuple(std::get<0>(vector_to_startV), std::get<1>(vector_to_startV), 0); */


  // ectorに書き直す
    vec3 projected_vector_to_nextV = vec3(vector_to_nextV.x,vector_to_nextV.y, 0);
    vec3 projected_vector_to_startV = vec3(vector_to_startV.x,vector_to_startV.y, 0);


    vec3 x_axis = vec3(1, 0, 0);
    vec3 z_axis = vec3(0, 0, 1);


    double roll_angle_to_nextV =
       (projected_vector_to_nextV.y < 0) ?
         -std::acos(
          dot( projected_vector_to_nextV, x_axis) 
            / (projected_vector_to_nextV.length() * x_axis.length())
          ) :
        std::acos(
          dot( projected_vector_to_nextV, x_axis) 
              / (projected_vector_to_nextV.length() * x_axis.length())
          );


    double roll_angle_to_startV =
       (projected_vector_to_startV.y < 0) ?
         -std::acos(
          dot( 
            projected_vector_to_startV, x_axis)
              / (projected_vector_to_startV.length() * x_axis.length()
          )
        ) :
        std::acos(
          dot( 
            projected_vector_to_startV, x_axis)
              / (projected_vector_to_startV.length() * x_axis.length()
          )
        );

        /* roll_angle_to_nextV = normalizeAngle(roll_angle_to_nextV); */
        /* roll_angle_to_startV = normalizeAngle(roll_angle_to_startV); */




  // ----

  // xz平面に投影するためには、z軸を中心にロールさせる

    /* vec3 rolled_vector_to_nextV = */ 
      /* vec3(vector_to_nextV.x, vector_to_nextV.y, vector_to_nextV.z - roll_angle_to_nextV); */
    vec3 xz_plane_vector_to_nextV = rotateAroundZ(vector_to_nextV, -roll_angle_to_nextV);
 
    vec3 xz_plane_vecetor_to_startV = rotateAroundZ(vector_to_startV, -roll_angle_to_startV);
      /* vec3(vector_to_startV.x, vector_to_startV.y, vector_to_startV.z - roll_angle_to_startV); */



/*       vec3 rolled_vector_to_nextV = */ 
/*         vec3(vector_to_nextV.x, 0, vector_to_nextV.z); */
/*       vec3 rolled_vector_to_startV = */ 
/*         vec3(vector_to_startV.x, 0, vector_to_startV.z); */


  double rotation_angle_to_nextV =
    (xz_plane_vector_to_nextV.x < 0) ?
       - std::acos(
          dot( xz_plane_vector_to_nextV, z_axis)
            / (xz_plane_vector_to_nextV.length() * z_axis.length())
      ) :
      // acosの戻り地は0からPI(180°)
        std::acos(
          dot( xz_plane_vector_to_nextV, z_axis) 
            / (xz_plane_vector_to_nextV.length() * z_axis.length())
      );

  double rotation_angle_to_startV =
    (xz_plane_vecetor_to_startV.x < 0) ?
       - std::acos(
          dot( xz_plane_vecetor_to_startV, z_axis) 
            / (xz_plane_vecetor_to_startV.length() * z_axis.length())
      ) :
      std::acos(
          dot( xz_plane_vecetor_to_startV, z_axis) 
           / (xz_plane_vecetor_to_startV.length() * z_axis.length())
      );


  /* roll_angle_to_nextV */
  /* rotation_angle_to_nextV = normalizeAngle(rotation_angle_to_nextV); */
  /* rotation_angle_to_startV = normalizeAngle(rotation_angle_to_startV); */
  /* // ---- */


    // Compute relative angles
    double relative_roll_angle = roll_angle_to_nextV - roll_angle_to_startV;
    /* double relative_rotation_angle = rotation_angle_to_nextV - rotation_angle_to_startV; */

    // x軸をもとにしたangleを算出しているのでZ軸からの差分を算出する
    /* double relative_rotation_angle = (rotation_angle_to_startV - rotation_angle_to_nextV); */
    /* double relative_rotation_angle = -(rotation_angle_to_nextV - rotation_angle_to_startV); */
    double relative_rotation_angle = rotation_angle_to_nextV - rotation_angle_to_startV;

  /* if (std::abs(rotation_angle_to_nextV - rotation_angle_to_startV) * 180/M_PI > 150) { */
  /*   std::cout << "ローテーション" << (rotation_angle_to_nextV - rotation_angle_to_startV) * 180/M_PI << std::endl; */
  /* } */

  /* if (std::abs(roll_angle_to_nextV - roll_angle_to_startV) * 180/M_PI > 150) { */
  /*   std::cout << "ロール" << (roll_angle_to_nextV - roll_angle_to_startV) * 180/M_PI << std::endl; */
  /* } */
  /*   std::cout << "next length" << branch_length << std::endl; */

  /*   std::cout << "rotation_angle_to_nextV" << rotation_angle_to_nextV << std::endl; */
  /*   std::cout << "rotation_angle_to_startV" << rotation_angle_to_startV << std::endl; */

  /*   std::cout << "roll_angle_to_nextV" << roll_angle_to_nextV << std::endl; */
  /*   std::cout << "roll_angle_to_startV" << roll_angle_to_startV << std::endl; */
  /*   std::cout << "----" << std::endl; */


    // Round angles close to 360 or 0 degrees to 0
    /* if (std::abs(relative_roll_angle) < accuracy || std::abs(relative_roll_angle - 2* M_PI) < accuracy) relative_roll_angle = 0; */
    /* if (std::abs(relative_rotation_angle) < accuracy || std::abs(relative_rotation_angle - 2*M_PI) < accuracy) relative_rotation_angle = 0; */

    // Compute branch length

    return std::make_tuple(relative_rotation_angle, relative_roll_angle, branch_length);
}

void LSystem::generateLString(
  const easy3d::Graph& skel,
//  easy3d::Graph::Vertex rootV,
  double accuracy
) {

    LString.clear();

auto first_vertex_it = skel.vertices_begin();
if (first_vertex_it != skel.vertices_end()) {
    auto rootV = *first_vertex_it;

    // ここで first_vertex を使用
    // 例えば、その位置を取得する
    //auto rootV = skel.position(first_vertex);

    // その他の操作...
    std::cerr << "start traversal" << std::endl;
    traversal(
      skel,
      rootV,
      rootV,
      accuracy
    );
}


    std::ofstream outFile("out.txt");
    if (!outFile) {
        std::cerr << "Unable to open file for writing!" << std::endl;
        return;
    }
    outFile << this->LString << std::endl;
    outFile.close();
}

void LSystem::traversal(
//  const easy3d::Graph& skeleton,
//  const easy3d::Graph& graph,
////  dvec3 startV,
////  dvec3 prevV,
//  SGraphVertexDescriptor startV,
//  SGraphVertexDescriptor prevV,
//  std::unordered_map<SGraphVertexDescriptor,
//  Graph::Vertex>  vvmap,
//  double accuracy
  const easy3d::Graph& skel,
  easy3d::Graph::Vertex startV,
  easy3d::Graph::Vertex prevV,
  double accuracy
) {
  std::cout << "Traversal started at vertex: " << startV << std::endl;

    //std::vector<dvec3> children;
//    std::vector<SGraphVertexDescriptor> children;
//
//  // edgeを取得
//    auto edgeRange = out_edges(startV, skeleton);
//    for (auto it = edgeRange.first; it != edgeRange.second; ++it) {
//
//        //SGraphVertexDescriptor s = boost::source(*it, skeleton);
//        //SGraphVertexDescriptor t = boost::target(*it, skeleton);
//        //graph.find_edge(vvmap[s], vvmap[t]);
//        // edgeからdiscriptorを取得
//        SGraphVertexDescriptor target = boost::target(*it, skeleton);
//        if (target != prevV) {
//            children.push_back(target);
//        }
//    }
   std::vector<easy3d::Graph::Vertex> children;

//   auto end_vertex_it = skel.vertices_end();
//   if (end_vertex_it != skel.vertices_begin()) {
//    auto endV = *end_vertex_it;
//    if (endV == startV) {
//
//      std::cout << "end" << std::endl;
//      return;
//    }
//   }
//
//   auto first_vertex_it = skel.vertices_begin();
//   if (first_vertex_it != skel.vertices_end()) {
//       auto rootV = *first_vertex_it;
//       if (rootV == startV) {
//
//         std::cout << "start rootV" << std::endl;
//       }
//       if (rootV == prevV) {
//
//         std::cout << "prev rootV" << std::endl;
//       }
//  }
//


    // 子ノードを取得
    std::cout <<  startV << "`s adjacent" << std::endl;
    for (auto e : skel.edges(startV)) {
    easy3d::Graph::Vertex target = skel.to_vertex(e);
    std::cout <<  "adjacent: " << target << std::endl;
    /// TODO startVも含まれている。挙動を把握
        if (target != prevV && target != startV) {
            children.push_back(target);
        }
    }
  std::cout << "Children lenght " << children.size() << std::endl;


//  std::cout << "children size" << children.size() << std::endl;
    if (children.size() <= 1) {
        if (children.size() == 1) {
            writeLString(startV, children[0], prevV, skel, accuracy);
            //traversal(skeleton, easy3d::Graph, children[0], startV, vvmap, accuracy);
            std::cout << "Moving from vertex children - 1 " << startV << " to child vertex " << children[0] << std::endl;
            std::cout << "prevV " << prevV << std::endl;

            traversal(skel, children[0], startV, accuracy);
        }
    } else {
        for (auto child : children) {
       std::cout << "child: " << child << std::endl;
            LString += "[";

            writeLString(startV, child, prevV, skel, accuracy);
//            traversal(skeleton, easy3d::Graph, child, startV, vvmap, accuracy);
             std::cout << "Moving from vertex " << startV << " to child vertex " << child << std::endl;
             std::cout << "prevV " << prevV << std::endl;
             traversal(skel, child, startV, accuracy);
            LString += "]";
        }
    }
    /* std::cout <<  "junction traversal end" << std::endl; */

    // Rest of the logic remains the same
 std::cout << "Traversal finished at vertex: " << startV << std::endl;   // ...
}

void LSystem::writeLString(
  easy3d::Graph::Vertex startV,
  easy3d::Graph::Vertex nextV,
  easy3d::Graph::Vertex prevV,
  const easy3d::Graph& skel,
  double accuracy
) {
    auto movement = computeRelationBetweenNodes(
      startV,
      nextV,
      prevV,
      skel,
      accuracy
    );
    double rotationAngle = std::get<0>(movement);
    double rollAngle = std::get<1>(movement);
    double forwardDistance = std::get<2>(movement);

    double radianRollAngle = rollAngle;
    double radianRotationAngle = rotationAngle;

    if (degrees) {
        rollAngle = radianRollAngle * (180.0 / M_PI);
        rotationAngle = radianRotationAngle * (180.0 / M_PI);
    }

    if (std::abs(radianRollAngle) > accuracy) {
      if (rollAngle > 0) {
          /* LString += "<" + std::to_string(rollAngle); */
          LString += "/" + std::string("(") + std::to_string(rollAngle) + ")";

          //LString += "\\" + std::string("(") + std::to_string(rollAngle) + ")";
          // nextVのroll属性にも同じ値を書き込む
      } else {
          /* LString += ">" + std::to_string(-rollAngle); */
          LString += "\\" + std::string("(") + std::to_string(-rollAngle) + ")";

          //LString += "/" + std::string("(") + std::to_string(-rollAngle) + ")";
          // nextVのroll属性にも同じ値を書き込む
      }
  }

    if (std::abs(radianRotationAngle) > accuracy) {
        if (rotationAngle > 0) {
            /* LString += "+" + std::string("(") + std::to_string(rotationAngle) + ")"; */
            LString += "^" + std::string("(") + std::to_string(rotationAngle) + ")";
            // nextVのrotation属性にも同じ値を書き込む
        } else {
            /* LString += "-" + std::string("(") + std::to_string(-rotationAngle) + ")"; */
            // nextVのrotation属性にも同じ値を書き込む
            LString += "&" + std::string("(") + std::to_string(-rotationAngle) + ")";
        }
    }




    if (forwardDistance > accuracy) {
        LString += "F" + std::string("(") + std::to_string(forwardDistance) + ")";
        // nextVのforward属性にも同じ値を書き込む
    }
}
