// Copyright 2026 WheelOS. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");

#include "candidates/local/motion_bootstrap_error.h"

#include <algorithm>
#include <cmath>
#include <map>

#include "candidates/local/local_motion.h"
#include "candidates/local/motion_initializer.h"

namespace openloc::localization::unified {
namespace {

using PoseLoading = Eigen::Matrix<double, 15, 6>;
using PointLoading = Eigen::Matrix<double, 15, 3>;

Eigen::Matrix<double, 15, 1> Difference(
    const MotionInitialization& nominal, const MotionInitialization& value) {
  Eigen::Matrix<double, 15, 1> error =
      Eigen::Matrix<double, 15, 1>::Zero();
  error.segment<3>(3) = value.velocity - nominal.velocity;
  error.segment<3>(6) =
      LogRotation(nominal.orientation.conjugate() * value.orientation);
  error.segment<3>(9) = value.gyro_bias - nominal.gyro_bias;
  error.segment<3>(12) = value.accel_bias - nominal.accel_bias;
  return error;
}

Result InitializerLoadings(
    const MotionInitializationConfig& policy,
    const std::vector<RelativeLidarPose>& poses,
    const std::vector<ImuSample>& imu, const MotionInitialization& nominal,
    std::vector<PoseLoading>* pose_loadings,
    std::vector<PoseLoading>* imu_loadings) {
  constexpr double h = 1e-5;
  auto changed_poses = poses;
  auto changed_imu = imu;
  const auto evaluate = [&](MotionInitialization* value) -> Result {
    const auto decision =
        InitializeFromRelativeMotion(policy, changed_poses, changed_imu, value);
    if (!decision.ok()) return decision;
    if (value->lidar_stationary_initialization !=
        nominal.lidar_stationary_initialization)
      return {Reason::CORRELATION_UNQUALIFIED,
              "Startup derivative crosses the rest/motion selection boundary"};
    return {};
  };
  pose_loadings->assign(poses.size(), PoseLoading::Zero());
  imu_loadings->assign(imu.size(), PoseLoading::Zero());
  for (std::size_t i = 0; i < poses.size(); ++i) {
    for (int axis = 0; axis < 6; ++axis) {
      MotionInitialization plus, minus;
      for (const int sign : {1, -1}) {
        changed_poses[i] = poses[i];
        if (axis < 3) changed_poses[i].reference_from_base.translation()[axis] += sign * h;
        else {
          Eigen::Vector3d angle = Eigen::Vector3d::Zero();
          angle[axis - 3] = sign * h;
          changed_poses[i].reference_from_base.linear() =
              ExpRotation(angle).toRotationMatrix() *
              poses[i].reference_from_base.linear();
        }
        const auto valid = evaluate(sign > 0 ? &plus : &minus);
        if (!valid.ok()) return valid;
      }
      (*pose_loadings)[i].col(axis) =
          (Difference(nominal, plus) - Difference(nominal, minus)) / (2 * h);
      changed_poses[i] = poses[i];
    }
  }
  for (std::size_t i = 0; i < imu.size(); ++i) {
    for (int axis = 0; axis < 6; ++axis) {
      MotionInitialization plus, minus;
      for (const int sign : {1, -1}) {
        changed_imu[i] = imu[i];
        if (axis < 3) changed_imu[i].acceleration[axis] += sign * h;
        else changed_imu[i].angular_velocity[axis - 3] += sign * h;
        const auto valid = evaluate(sign > 0 ? &plus : &minus);
        if (!valid.ok()) return valid;
      }
      (*imu_loadings)[i].col(axis) =
          (Difference(nominal, plus) - Difference(nominal, minus)) / (2 * h);
      changed_imu[i] = imu[i];
    }
  }
  return {};
}

}  // namespace

Result RelativeBootstrapCovariance(
    const MotionInitializationConfig& policy, const LocalConfig& config,
    const std::vector<RelativeLidarPose>& poses,
    const std::vector<ImuSample>& imu, const MotionInitialization& nominal,
    const RelativeBootstrapEvidence& evidence,
    Eigen::Matrix<double, 21, 21>* covariance) {
  if (!evidence.assessment.ok()) return evidence.assessment;
  if (!covariance || evidence.first_pose_frame > evidence.frames.size() ||
      poses.size() != evidence.frames.size() - evidence.first_pose_frame ||
      evidence.frames.size() > 40 ||
      poses.empty() || poses.size() > 40 || imu.size() > 4096 ||
      evidence.imu.empty() || evidence.imu.size() > 4096 ||
      !std::isfinite(config.native_imu_sample_period) ||
      config.native_imu_sample_period <= 0 ||
      !std::isfinite(config.lidar_point_noise_std) ||
      config.lidar_point_noise_std <= 0)
    return {Reason::CORRELATION_UNQUALIFIED,
            "Startup raw-source model lacks bounded sampled-IMU/scan support"};
  std::map<double, std::size_t> owner_samples;
  for (std::size_t i = 0; i < imu.size(); ++i)
    owner_samples.emplace(imu[i].stamp.time, i);
  std::vector<std::size_t> gyro_indices;
  std::vector<Eigen::Quaterniond> rotations{evidence.first_imu_rotation};
  for (std::size_t i = 0; i < evidence.imu.size(); ++i) {
    const auto& sample = evidence.imu[i];
    const auto found = owner_samples.find(sample.stamp.time);
    if (found == owner_samples.end() ||
        imu[found->second].stamp.sequence != sample.stamp.sequence ||
        !imu[found->second].acceleration.isApprox(sample.acceleration, 0) ||
        !imu[found->second].angular_velocity.isApprox(sample.angular_velocity, 0))
      return {Reason::CORRELATION_UNQUALIFIED,
              "Bootstrap and initializer do not share the same raw IMU samples"};
    gyro_indices.push_back(found->second);
    if (i) {
      const double dt = sample.stamp.time - evidence.imu[i - 1].stamp.time;
      if (dt <= 0 || dt > config.max_imu_gap)
        return {Reason::IMU_GAP, "Startup stochastic support crosses an IMU gap"};
      rotations.push_back((rotations.back() * ExpRotation(
          0.5 * dt * (sample.angular_velocity +
                      evidence.imu[i - 1].angular_velocity))).normalized());
    }
  }
  std::vector<std::size_t> ends;
  for (std::size_t i = 0; i < evidence.frames.size(); ++i) {
    const auto& frame = evidence.frames[i];
    const auto found = std::lower_bound(
        evidence.imu.begin(), evidence.imu.end(), frame.scan.stamp.time,
        [](const ImuSample& sample, double time) { return sample.stamp.time < time; });
    if (found == evidence.imu.end() || found->stamp.time != frame.scan.stamp.time ||
        (i >= evidence.first_pose_frame &&
         (poses[i - evidence.first_pose_frame].time != frame.scan.stamp.time ||
          !poses[i - evidence.first_pose_frame].reference_from_base.matrix()
               .isApprox(frame.pose.matrix(), 0))) ||
        frame.scan.stamp.sequence == 0 ||
        frame.scan.stamp.clock_id != imu.back().stamp.clock_id ||
        frame.scan.epoch != evidence.frames.front().scan.epoch ||
        frame.scan.frame_id != config.lidar_frame ||
        frame.scan.calibration_id != config.lidar_calibration_id ||
        !frame.scan.base_from_lidar.matrix().allFinite() ||
        !(frame.scan.base_from_lidar.linear().transpose() *
          frame.scan.base_from_lidar.linear()).isApprox(Eigen::Matrix3d::Identity(), 1e-9) ||
        std::abs(frame.scan.base_from_lidar.linear().determinant() - 1) > 1e-9 ||
        (i && frame.scan.stamp.sequence <= evidence.frames[i - 1].scan.stamp.sequence) ||
        frame.scan.points.empty() || frame.scan.points.size() > 100000 ||
        !frame.imu_rotation.isApprox(rotations[found - evidence.imu.begin()], 1e-10) ||
        (i && (!frame.registration.assessment.ok() ||
               frame.registration.source_point_jacobians.size() != frame.scan.points.size() ||
               frame.registration.target_point_jacobians.size() !=
                   evidence.frames[i - 1].scan.points.size())))
      return {Reason::CORRELATION_UNQUALIFIED,
              "Startup poses lack their actual frozen raw registration support"};
    ends.push_back(found - evidence.imu.begin());
  }
  std::vector<PoseLoading> pose_gradients, imu_gradients;
  auto valid = InitializerLoadings(
      policy, poses, imu, nominal, &pose_gradients, &imu_gradients);
  if (!valid.ok()) return valid;
  pose_gradients.insert(pose_gradients.begin(), evidence.first_pose_frame,
                        PoseLoading::Zero());
  std::vector<PointLoading> rotation_gradients(evidence.imu.size(), PointLoading::Zero());
  Matrix15d lidar_covariance = Matrix15d::Zero();
  std::vector<PointLoading> current_points(
      evidence.frames.back().scan.points.size(), PointLoading::Zero());
  for (std::size_t reverse = evidence.frames.size(); reverse > 0; --reverse) {
    const std::size_t i = reverse - 1;
    const auto& frame = evidence.frames[i];
    std::vector<PointLoading> previous_points;
    if (i) {
      previous_points.assign(evidence.frames[i - 1].scan.points.size(), PointLoading::Zero());
      const auto& previous_pose = evidence.frames[i - 1].pose;
      const Eigen::Vector3d displacement =
          frame.pose.translation() - previous_pose.translation();
      Matrix6d previous = Matrix6d::Identity();
      previous.topRightCorner<3, 3>() = -Skew(displacement);
      Matrix6d relative = Matrix6d::Zero();
      relative.topLeftCorner<3, 3>() = previous_pose.linear();
      relative.bottomRightCorner<3, 3>() = previous_pose.linear();
      const PoseLoading match_gradient = pose_gradients[i] * relative;
      pose_gradients[i - 1].noalias() += pose_gradients[i] * previous;
      const PoseLoading seed = match_gradient * frame.registration.initial_jacobian;
      const PointLoading seed_rotation =
          seed.rightCols<3>() * evidence.frames[i - 1].imu_rotation.toRotationMatrix().transpose();
      rotation_gradients[ends[i]] += seed_rotation;
      rotation_gradients[ends[i - 1]] -= seed_rotation;
      for (std::size_t point = 0; point < current_points.size(); ++point)
        current_points[point].noalias() +=
            match_gradient * frame.registration.source_point_jacobians[point];
      for (std::size_t point = 0; point < previous_points.size(); ++point)
        previous_points[point].noalias() +=
            match_gradient * frame.registration.target_point_jacobians[point];
    }
    const Eigen::Matrix3d inverse_end = frame.imu_rotation.toRotationMatrix().transpose();
    PointLoading velocity_gradient = PointLoading::Zero();
    for (std::size_t point = 0; point < current_points.size(); ++point) {
      const auto& raw = frame.scan.points[point];
      const auto upper = std::lower_bound(
          evidence.imu.begin(), evidence.imu.end(), raw.time,
          [](const ImuSample& sample, double time) { return sample.stamp.time < time; });
      if (upper == evidence.imu.end() || raw.time > frame.scan.stamp.time ||
          (upper == evidence.imu.begin() && upper->stamp.time != raw.time))
        return {Reason::CORRELATION_UNQUALIFIED, "Startup point lacks its raw IMU bracket"};
      const std::size_t right = upper - evidence.imu.begin();
      Eigen::Quaterniond at = rotations[right];
      const bool interpolated = upper->stamp.time != raw.time;
      double alpha = 0;
      if (interpolated) {
        alpha = (raw.time - evidence.imu[right - 1].stamp.time) /
            (upper->stamp.time - evidence.imu[right - 1].stamp.time);
        at = rotations[right - 1].slerp(alpha, at);
      }
      const PointLoading loading = current_points[point];
      const PointLoading raw_loading =
          loading * inverse_end * at.toRotationMatrix() * frame.scan.base_from_lidar.linear();
      lidar_covariance.noalias() += config.lidar_point_noise_std *
          config.lidar_point_noise_std * raw_loading * raw_loading.transpose();
      const PointLoading rotation =
          loading * inverse_end *
          Skew(at * (frame.scan.base_from_lidar * raw.position));
      rotation_gradients[ends[i]] += rotation;
      if (!interpolated) {
        rotation_gradients[right] -= rotation;
      } else {
        Eigen::Matrix3d start, end;
        valid = RotationInterpolationJacobians(
            rotations[right - 1], rotations[right], alpha, &start, &end);
        if (!valid.ok()) return valid;
        rotation_gradients[right - 1].noalias() -= rotation * at.toRotationMatrix() *
            start * rotations[right - 1].toRotationMatrix().transpose();
        rotation_gradients[right].noalias() -= rotation * at.toRotationMatrix() *
            end * rotations[right].toRotationMatrix().transpose();
      }
      velocity_gradient.noalias() += loading * (raw.time - frame.scan.stamp.time);
    }
    if (evidence.translation_deskew && i >= 2) {
      const auto& previous = evidence.frames[i - 1];
      const auto& earlier = evidence.frames[i - 2];
      const double dt = previous.scan.stamp.time - earlier.scan.stamp.time;
      const Eigen::Matrix3d gyro_axes =
          previous.pose.linear() * previous.imu_rotation.toRotationMatrix().transpose();
      const Eigen::Matrix3d reference_from_end =
          gyro_axes * frame.imu_rotation.toRotationMatrix();
      const Eigen::Vector3d velocity =
          (previous.pose.translation() - earlier.pose.translation()) / dt;
      const PointLoading translation = velocity_gradient * reference_from_end.transpose() / dt;
      pose_gradients[i - 1].leftCols<3>() += translation;
      pose_gradients[i - 2].leftCols<3>() -= translation;
      const PointLoading attitude =
          velocity_gradient * reference_from_end.transpose() * Skew(velocity);
      pose_gradients[i - 1].rightCols<3>() += attitude;
      rotation_gradients[ends[i]].noalias() += attitude * gyro_axes;
      rotation_gradients[ends[i - 1]].noalias() -= attitude * gyro_axes;
    }
    current_points = std::move(previous_points);
  }
  for (std::size_t i = evidence.imu.size() - 1; i > 0; --i) {
    const double dt = evidence.imu[i].stamp.time - evidence.imu[i - 1].stamp.time;
    const Eigen::Vector3d angle =
        0.5 * dt * (evidence.imu[i - 1].angular_velocity + evidence.imu[i].angular_velocity);
    const PointLoading sample =
        0.5 * dt * rotation_gradients[i] * rotations[i - 1].toRotationMatrix() *
        RightRotationJacobian(-angle);
    imu_gradients[gyro_indices[i]].rightCols<3>() += sample;
    imu_gradients[gyro_indices[i - 1]].rightCols<3>() += sample;
    rotation_gradients[i - 1] += rotation_gradients[i];
  }
  Matrix6d sample_covariance = Matrix6d::Zero();
  sample_covariance.diagonal().head<3>().setConstant(
      config.accel_noise * config.accel_noise / config.native_imu_sample_period);
  sample_covariance.diagonal().tail<3>().setConstant(
      config.gyro_noise * config.gyro_noise / config.native_imu_sample_period);
  Eigen::Matrix<double, 21, 21> joint = Eigen::Matrix<double, 21, 21>::Zero();
  joint.topLeftCorner<15, 15>() = lidar_covariance;
  for (const auto& gradient : imu_gradients)
    joint.topLeftCorner<15, 15>().noalias() +=
        gradient * sample_covariance * gradient.transpose();
  joint.topRightCorner<15, 6>() = imu_gradients.back() * sample_covariance;
  joint.bottomLeftCorner<6, 15>() = joint.topRightCorner<15, 6>().transpose();
  joint.bottomRightCorner<6, 6>() = sample_covariance;
  if (!joint.allFinite())
    return {Reason::CORRELATION_UNQUALIFIED, "Startup raw covariance overflow"};
  *covariance = (0.5 * (joint + joint.transpose())).eval();
  return {};
}

}  // namespace openloc::localization::unified
