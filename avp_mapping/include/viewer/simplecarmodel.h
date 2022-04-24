#pragma once
#include <Eigen/Dense>
#include <fstream>
#include <string>

static float simplecarmodel_point[7676 * 9] = {-0.069059997797012329102,
                                               0.74707001447677612305,
                                               1.2226599454879760742,
                                               1.3890099525451660156};
class SimpleCarModel {
public:
  SimpleCarModel() {
    for (int i = 0; i < 7676; i++) {

      Eigen::Vector3f p1(simplecarmodel_point[i * 9],
                         simplecarmodel_point[i * 9 + 1],
                         simplecarmodel_point[i * 9 + 2]);
      Eigen::Vector3f p2(simplecarmodel_point[i * 9 + 3],
                         simplecarmodel_point[i * 9 + 4],
                         simplecarmodel_point[i * 9 + 5]);
      Eigen::Vector3f p3(simplecarmodel_point[i * 9 + 6],
                         simplecarmodel_point[i * 9 + 7],
                         simplecarmodel_point[i * 9 + 8]);
      std::vector<Eigen::Vector3f> face{p1, p2, p3};
      model_faces_.push_back(face);
    }
  };
  //  bool LoadCarModelStl(const std::string &filename) {

  //    std::ifstream fin;
  //    ;
  //    fin.open(filename);
  //    if (!fin.is_open())
  //      return false;

  //    std::string line;
  //    std::getline(fin, line);

  //    for (int i = 0; i < 7676; i++) {
  //      auto face = GetModelFace(fin);
  //      model_faces_.push_back(face);
  //    }
  //    return true;
  //  }

private:
  std::vector<Eigen::Vector3f> GetModelFace(std::ifstream &fin) {
    std::vector<Eigen::Vector3f> face;
    std::string line;
    std::getline(fin, line);
    std::getline(fin, line);
    for (int i = 0; i < 3; i++) {
      float x, y, z;
      std::getline(fin, line);
      sscanf(line.c_str(), " vertex %f %f %f", &x, &y, &z);
      face.push_back(Eigen::Vector3f(x, y, z));
    }
    std::getline(fin, line);
    std::getline(fin, line);
    return face;
  }

public:
  std::vector<std::vector<Eigen::Vector3f>> model_faces_;
};
