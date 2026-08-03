#include <Eigen/Core>

#include "a_star/a_star_search.h"

int main() {
  const Eigen::MatrixXd map = Eigen::MatrixXd::Zero(1, 1);
  const Eigen::Vector3i point = Eigen::Vector3i::Zero();

  Astar astar;
  astar.Init(20.0, 1, 1.0, 0.5, map, map, map, map);

  if (!astar.Search(point, point) ||
      astar.GetPathPoints().size() != 1 ||
      astar.GetResultMatrix().rows() != 1) {
    return 1;
  }

  return 0;
}
