#pragma once

#include "map.h"
namespace FeatureSLAM {

class Map;
class MapAlignment {
public:
  MapAlignment() = delete;
  MapAlignment(Map *map);
  ~MapAlignment() = default;

  void Align(const std::string &trajectory_file) const;

private:
  Map *map_;
};
}
