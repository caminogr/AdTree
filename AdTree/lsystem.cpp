#include "lsystem.h"
#include <fstream>
#include <tuple>
/* #include <cmath> */
/* #include <easy3d/core/point_cloud.h> */
/* #include <easy3d/core/surface_mesh.h> */
/* #include <easy3d/core/random.h> */
/* #include <easy3d/core/principal_axes.h> */
/* #include <3rd_party/tetgen/tetgen.h> */

#include <boost/graph/graphviz.hpp>
#include <iostream>
/* #include <algorithm> */
/* using namespace boost; */
using namespace easy3d;

// Helper function to roll a vector onto the XZ plane around the Y axis
/* easy3d::vec3 roll_to_XZ_plane(const easy3d::vec3& vec) { */
/*     double angle = -std::atan2(vec.y, vec.x); // Y軸を中心に回転させる角度 */
/*     /1* double angle = -std::atan2(vec.y, vec.x); // Y軸を中心に回転させる角度 *1/ */
/*     double cos_angle = std::cos(angle); */
/*     double sin_angle = std::sin(angle); */
/*     // Y軸を中心に回転 */
/*     return easy3d::vec3( */
/*         vec.x * cos_angle + vec.y * sin_angle, */
/*         0, // Y成分は0になる */
/*         -vec.x * sin_angle + vec.y * cos_angle */
/*     ); */
/* } */
/* easy3d::vec3 roll_to_XZ_plane(const easy3d::vec3& vec, double roll_angle) { */
/*   double cos_angle = std::cos(-roll_angle); */
/*   double sin_angle = std::sin(-roll_angle); */
/*   // 新しいY成分とZ成分を計算 */
/*   double new_y = vec.y * cos_angle - vec.z * sin_angle; */
/*   double new_z = vec.y * sin_angle + vec.z * cos_angle; */
/*   // 新しいベクトルを返す（X成分は変わらない） */
/*   return easy3d::vec3(vec.x, new_y, new_z); */
/* } */

double normalizeAngle(double angle){
  double new_angle = angle;
  if(new_angle < -M_PI){
    new_angle += 2 * M_PI;
  }
  if (new_angle > M_PI){
    new_angle -= 2 * M_PI;
  }
  return new_angle;
}



// Helper function to rotate a vector around the Y-axis by a given angle
/* easy3d::vec3 rotateAroundY(const easy3d::vec3& vector, double angle) { */
/*     // Implement the rotation logic here, possibly using Rodrigues' rotation formula */
/*     // This is a placeholder implementation */
/*     return vector; // Replace with actual rotated vector */
/* } */
// Helper function to roll a vector onto the XZ plane around the Y axis
/* easy3d::vec3 roll_to_XZ_plane(const easy3d::vec3& vec) { */
/*     double angle = -std::atan2(vec.y, vec.x); // Y軸を中心に回転させる角度 */
/*     double cos_angle = std::cos(angle); */
/*     double sin_angle = std::sin(angle); */
/*     // Y軸を中心に回転 */
/*     return easy3d::vec3( */
/*         vec.x * cos_angle + vec.y * sin_angle, */
/*         0, // Y成分は0になる */
/*         -vec.x * sin_angle + vec.y * cos_angle */
/*     ); */
/* } */

easy3d::vec3 rotateAroundZ(const easy3d::vec3&vec , double angle) {
  double cosAngle = std::cos(angle);
  double sinAngle = std::sin(angle);
  return easy3d::mat3(
    cosAngle, -sinAngle, 0,
    sinAngle, cosAngle, 0,
    0, 0, 1
  ) * vec;
}

// TODO: アルゴリズムを見直す
// houdiniの軸
std::tuple<double, double, double> computeRelationBetweenNodes(
  
  SGraphVertexDescriptor startV, SGraphVertexDescriptor nextV, SGraphVertexDescriptor prevV, const Graph& skel, double accuracy) {
   // 座標の取得
    auto coords_start = skel[startV].cVert;
    auto coords_next = skel[nextV].cVert;
    auto coords_prev = skel[prevV].cVert;

    // ベクトルの計算
    easy3d::vec3 vector_to_nextV = easy3d::vec3(coords_next - coords_start);
    easy3d::vec3 vector_to_startV = easy3d::vec3(coords_start - coords_prev);

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
    easy3d::vec3 projected_vector_to_nextV = easy3d::vec3(vector_to_nextV.x,vector_to_nextV.y, 0);
    easy3d::vec3 projected_vector_to_startV = easy3d::vec3(vector_to_startV.x,vector_to_startV.y, 0);


    easy3d::vec3 x_axis = easy3d::vec3(1, 0, 0);
    easy3d::vec3 z_axis = easy3d::vec3(0, 0, 1);


    double roll_angle_to_nextV =
       (projected_vector_to_nextV.y < 0) ?
         -std::acos(
          easy3d::dot( projected_vector_to_nextV, x_axis) 
            / (projected_vector_to_nextV.length() * x_axis.length())
          ) :
        std::acos(
          easy3d::dot( projected_vector_to_nextV, x_axis) 
              / (projected_vector_to_nextV.length() * x_axis.length())
          );


    double roll_angle_to_startV =
       (projected_vector_to_startV.y < 0) ?
         -std::acos(
          easy3d::dot( 
            projected_vector_to_startV, x_axis)
              / (projected_vector_to_startV.length() * x_axis.length()
          )
        ) :
        std::acos(
          easy3d::dot( 
            projected_vector_to_startV, x_axis)
              / (projected_vector_to_startV.length() * x_axis.length()
          )
        );

        /* roll_angle_to_nextV = normalizeAngle(roll_angle_to_nextV); */
        /* roll_angle_to_startV = normalizeAngle(roll_angle_to_startV); */




  // ----

  // xz平面に投影するためには、z軸を中心にロールさせる

    /* easy3d::vec3 rolled_vector_to_nextV = */ 
      /* easy3d::vec3(vector_to_nextV.x, vector_to_nextV.y, vector_to_nextV.z - roll_angle_to_nextV); */
    easy3d::vec3 xz_plane_vector_to_nextV = rotateAroundZ(vector_to_nextV, -roll_angle_to_nextV);
 
    easy3d::vec3 xz_plane_vecetor_to_startV = rotateAroundZ(vector_to_startV, -roll_angle_to_startV);
      /* easy3d::vec3(vector_to_startV.x, vector_to_startV.y, vector_to_startV.z - roll_angle_to_startV); */



/*       easy3d::vec3 rolled_vector_to_nextV = */ 
/*         easy3d::vec3(vector_to_nextV.x, 0, vector_to_nextV.z); */
/*       easy3d::vec3 rolled_vector_to_startV = */ 
/*         easy3d::vec3(vector_to_startV.x, 0, vector_to_startV.z); */


  double rotation_angle_to_nextV =
    (xz_plane_vector_to_nextV.x < 0) ?
       - std::acos(
          easy3d::dot( xz_plane_vector_to_nextV, z_axis)
            / (xz_plane_vector_to_nextV.length() * z_axis.length())
      ) :
      // acosの戻り地は0からPI(180°)
        std::acos(
          easy3d::dot( xz_plane_vector_to_nextV, z_axis) 
            / (xz_plane_vector_to_nextV.length() * z_axis.length())
      );

  double rotation_angle_to_startV =
    (xz_plane_vecetor_to_startV.x < 0) ?
       - std::acos(
          easy3d::dot( xz_plane_vecetor_to_startV, z_axis) 
            / (xz_plane_vecetor_to_startV.length() * z_axis.length())
      ) :
      std::acos(
          easy3d::dot( xz_plane_vecetor_to_startV, z_axis) 
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

  if (isnan(relative_roll_angle)) {
    std::cout << "NAN: relative_roll_angle" << std::endl;
    std::cout << "roll_angle_to_nextV" << std::endl;
    std::cout << roll_angle_to_nextV << std::endl;
    std::cout << "roll_angle_to_startV" << std::endl;
    std::cout << roll_angle_to_startV << std::endl;

    std::cout << "coords_start" << std::endl;
    std::cout << coords_start << std::endl;

    std::cout << "coords_next" << std::endl;
    std::cout << coords_next << std::endl;


  }

  if (isnan(relative_rotation_angle)) {
    std::cout << "NAN: relative_rotation_angle" << std::endl;
    std::cout << "rotation_angle_to_nextV" << std::endl;
    std::cout << rotation_angle_to_nextV << std::endl;
    std::cout << "rotation_angle_to_startV" << std::endl;
    std::cout << rotation_angle_to_startV << std::endl;

    std::cout << "coords_start" << std::endl;
    std::cout << coords_start << std::endl;

    std::cout << "coords_next" << std::endl;
    std::cout << coords_next << std::endl;

    std::cout << "------------------" << std::endl;
  }

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





LSystem::LSystem() {
   LString = "";
}

LSystem::~LSystem() {
}

void LSystem::generateLString(
  const Graph& skeleton,
  SGraphVertexDescriptor rootV,
  double accuracy) {
    LString.clear();
    traversal(skeleton, rootV, rootV, accuracy);

    std::ofstream outFile("out.txt");

      // ファイルが正しく開けたかを確認
      if (!outFile) {
          std::cerr << "Unable to open file for writing!" << std::endl;
          return;
      }

      // ファイルにデータを書き込む
      outFile << this->getLString() << std::endl;

      // ファイルを閉じる
      outFile.close();
    /* traversal(skeleton, startV, boost::graph_traits<Graph>::null_vertex(), accuracy); */
}

void LSystem::traversal(
  const Graph& skeleton,
  SGraphVertexDescriptor startV,
  SGraphVertexDescriptor prevV,
  double accuracy
) {
    std::vector<SGraphVertexDescriptor> children;

  // edgeを取得
    auto edgeRange = out_edges(startV, skeleton);
    for (auto it = edgeRange.first; it != edgeRange.second; ++it) {
        // edgeからdiscriptorを取得
        SGraphVertexDescriptor target = boost::target(*it, skeleton);
        if (target != prevV) {
            children.push_back(target);
        }
    }

    if (children.size() <= 1) {
        if (children.size() == 1) {
            writeLString(startV, children[0], prevV, skeleton, accuracy);
            traversal(skeleton, children[0], startV, accuracy);
        }
    } else {
        for (auto child : children) {
            LString += "[";
            writeLString(startV, child, prevV, skeleton, accuracy);
            traversal(skeleton, child, startV, accuracy);
            LString += "]";
        }
    }
    /* std::cout <<  "junction traversal end" << std::endl; */
}

void LSystem::writeLString(SGraphVertexDescriptor startV,
                           SGraphVertexDescriptor nextV,
                           SGraphVertexDescriptor prevV,
                           const Graph& skel, double accuracy) {
    auto movement = computeRelationBetweenNodes(startV, nextV, prevV, skel, accuracy);
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
  /* } */

    /* if (std::abs(radianRotationAngle) > accuracy) { */
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




    /* if (forwardDistance > accuracy) { */
        LString += "F" + std::string("(") + std::to_string(forwardDistance) + ")";
        // nextVのforward属性にも同じ値を書き込む
    /* } */
}
