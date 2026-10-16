#include <cmath>
#include <iostream>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

int main() {
  using gtsam::symbol_shorthand::X;
  using Pose2 = gtsam::Pose2;
  using Smoother = gtsam::IncrementalFixedLagSmoother;

  Smoother smoother(10.0);
  gtsam::NonlinearFactorGraph first_graph;
  gtsam::Values first_values;
  Smoother::KeyTimestampMap first_timestamps;
  const auto prior_noise =
      gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3::Constant(1e-3));
  first_graph.emplace_shared<gtsam::PriorFactor<Pose2>>(
      X(0), Pose2(0.0, 0.0, 0.0), prior_noise);
  first_values.insert(X(0), Pose2(0.0, 0.0, 0.0));
  first_timestamps.emplace(X(0), 0.0);
  smoother.update(first_graph, first_values, first_timestamps);

  gtsam::NonlinearFactorGraph second_graph;
  gtsam::Values second_values;
  Smoother::KeyTimestampMap second_timestamps;
  const auto odometry_noise =
      gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3::Constant(0.1));
  second_graph.emplace_shared<gtsam::BetweenFactor<Pose2>>(
      X(0), X(1), Pose2(1.0, 0.0, 0.0), odometry_noise);
  second_values.insert(X(1), Pose2(1.0, 0.0, 0.0));
  second_timestamps.emplace(X(1), 0.1);
  smoother.update(second_graph, second_values, second_timestamps);

  const gtsam::Values estimate = smoother.calculateEstimate();
  const Pose2 pose = estimate.at<Pose2>(X(1));
  if (std::abs(pose.x() - 1.0) > 1e-6 || std::abs(pose.y()) > 1e-6 ||
      std::abs(pose.theta()) > 1e-6) {
    std::cerr << "fixed-lag smoother returned an unexpected estimate\n";
    return 1;
  }
  return 0;
}
