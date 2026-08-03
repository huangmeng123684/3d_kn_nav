#include <vector>

#include "common/data_types.h"
#include "trajectory_optimization/gpmp_optimizer/gpmp_optimizer.h"
#include "trajectory_optimization/gpmp_optimizer/gpmp_optimizer_wnoa.h"

int main() {
  const std::vector<PathPoint> empty_path;
  const std::vector<PathPoint> single_point_path(1);

  GPMPOptimizer quintic_optimizer;
  GPMPOptimizerWnoa non_quintic_optimizer;

  if (quintic_optimizer.GenerateTrajectory(empty_path, 200) ||
      quintic_optimizer.GenerateTrajectory(single_point_path, 200) ||
      non_quintic_optimizer.GenerateTrajectory(empty_path, 200) ||
      non_quintic_optimizer.GenerateTrajectory(single_point_path, 200)) {
    return 1;
  }

  return 0;
}
