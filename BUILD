load("@com_google_protobuf//bazel:cc_proto_library.bzl", "cc_proto_library")
load("@com_google_protobuf//bazel:proto_library.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:private"])

cc_library(
    name = "localization_types",
    srcs = ["src/localization/candidates/common/types.cc"],
    hdrs = ["src/localization/candidates/common/types.h"],
    includes = ["src/localization"],
    deps = ["@eigen"],
)

cc_library(
    name = "four_wheel_kinematics",
    srcs = ["src/localization/candidates/local/four_wheel_kinematics.cc"],
    hdrs = ["src/localization/candidates/local/four_wheel_kinematics.h"],
    deps = [
        ":localization_types",
        "@eigen",
    ],
)

cc_library(
    name = "planar_wheel_dr",
    srcs = ["src/localization/candidates/local/planar_wheel_dr.cc"],
    hdrs = ["src/localization/candidates/local/planar_wheel_dr.h"],
    deps = [":localization_types"],
)

cc_test(
    name = "planar_wheel_dr_tests",
    size = "small",
    srcs = ["test/localization/candidates/local/planar_wheel_dr_test.cc"],
    deps = [
        ":planar_wheel_dr",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "four_wheel_kinematics_tests",
    size = "small",
    srcs = ["test/localization/candidates/local/four_wheel_kinematics_test.cc"],
    deps = [
        ":four_wheel_kinematics",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "observed_subspace",
    srcs = ["src/localization/candidates/common/observed_subspace.cc"],
    hdrs = ["src/localization/candidates/common/observed_subspace.h"],
    includes = ["src/localization"],
    deps = [
        ":localization_types",
        "@eigen",
    ],
)

cc_library(
    name = "local_motion",
    srcs = ["src/localization/candidates/local/local_motion.cc"],
    hdrs = ["src/localization/candidates/local/local_motion.h"],
    includes = ["src/localization"],
    deps = [
        ":localization_types",
        "@eigen",
    ],
)

cc_library(
    name = "continuous_output",
    srcs = ["src/localization/candidates/local/continuous_output.cc"],
    hdrs = ["src/localization/candidates/local/continuous_output.h"],
    includes = ["src/localization"],
    deps = [
        ":localization_types",
        "@eigen",
    ],
)

cc_library(
    name = "stationarity_detector",
    srcs = ["src/localization/candidates/local/stationarity_detector.cc"],
    hdrs = ["src/localization/candidates/local/stationarity_detector.h"],
    includes = ["src/localization"],
    deps = [
        ":localization_types",
        "@eigen",
    ],
)

cc_library(
    name = "local_lio",
    srcs = ["src/localization/candidates/local/local_lio.cc"],
    hdrs = ["src/localization/candidates/local/local_lio.h"],
    includes = ["src/localization"],
    deps = [
        ":localization_types",
        ":observed_subspace",
        "@eigen",
    ],
)

cc_library(
    name = "local_estimator",
    srcs = ["src/localization/candidates/local/local_estimator.cc"],
    hdrs = ["src/localization/candidates/local/local_estimator.h"],
    includes = ["src/localization"],
    deps = [
        ":continuous_output",
        ":local_lio",
        ":local_motion",
        ":localization_types",
        ":stationarity_detector",
        "@eigen",
    ],
)

cc_library(
    name = "global_types",
    srcs = ["src/localization/candidates/global/global_types.cc"],
    hdrs = ["src/localization/candidates/global/global_types.h"],
    includes = ["src/localization"],
    deps = [
        ":localization_types",
        "@eigen",
    ],
)

cc_library(
    name = "global_alignment",
    srcs = [
        "src/localization/candidates/global/fixed_lag_graph.cc",
        "src/localization/candidates/global/global_alignment.cc",
    ],
    hdrs = [
        "src/localization/candidates/global/fixed_lag_graph.h",
        "src/localization/candidates/global/global_alignment.h",
    ],
    includes = ["src/localization"],
    deps = [
        ":global_types",
        ":local_motion",
        ":localization_types",
        "@eigen",
        "@gtsam",
    ],
)

cc_library(
    name = "lane_relative",
    srcs = ["src/localization/candidates/relative/lane_relative.cc"],
    hdrs = ["src/localization/candidates/relative/lane_relative.h"],
    includes = ["src/localization"],
    deps = [
        ":localization_types",
        "@eigen",
    ],
)

cc_library(
    name = "likelihood_field",
    srcs = ["src/localization/candidates/map/likelihood_field.cc"],
    hdrs = ["src/localization/candidates/map/likelihood_field.h"],
    includes = ["src/localization"],
    deps = [
        ":localization_types",
        "@eigen",
        "@local_config_pcl//:common",
        "@local_config_pcl//:kdtree",
    ],
)

cc_library(
    name = "particle_relocalizer",
    srcs = ["src/localization/candidates/map/particle_relocalizer.cc"],
    hdrs = ["src/localization/candidates/map/particle_relocalizer.h"],
    includes = ["src/localization"],
    deps = [
        ":likelihood_field",
        ":localization_types",
        "@eigen",
        "@local_config_pcl//:common",
    ],
)

cc_library(
    name = "particle_hypothesis_tracker",
    srcs = ["src/localization/candidates/map/particle_hypothesis_tracker.cc"],
    hdrs = ["src/localization/candidates/map/particle_hypothesis_tracker.h"],
    includes = ["src/localization"],
    deps = [
        ":localization_types",
        ":particle_relocalizer",
        "@eigen",
    ],
)

cc_library(
    name = "map_matcher",
    srcs = ["src/localization/candidates/map/map_matcher.cc"],
    hdrs = ["src/localization/candidates/map/map_matcher.h"],
    includes = ["src/localization"],
    deps = [
        ":localization_types",
        ":observed_subspace",
        "@eigen",
        "@local_config_pcl//:common",
        "@local_config_pcl//:filters",
        "@local_config_pcl//:io",
        "@local_config_pcl//:kdtree",
        "@local_config_pcl//:registration",
    ],
)

cc_library(
    name = "scan_to_map",
    srcs = ["src/localization/candidates/map/scan_to_map.cc"],
    hdrs = ["include/openloc/localization/map/scan_to_map.h"],
    includes = [
        "include",
        "src/localization",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":localization_types",
        ":map_matcher",
        ":observed_subspace",
        "@eigen",
        "@local_config_pcl//:common",
        "@local_config_pcl//:kdtree",
    ],
)

cc_library(
    name = "online_localization_types",
    srcs = ["src/localization/online/validation.cc"],
    hdrs = [
        "include/openloc/localization/online/types.h",
        "include/openloc/localization/online/validation.h",
    ],
    includes = ["include"],
    deps = ["@eigen"],
)

cc_library(
    name = "online_odometry_buffer",
    srcs = ["src/localization/online/odometry_buffer.cc"],
    hdrs = ["include/openloc/localization/online/odometry_buffer.h"],
    includes = ["include"],
    deps = [
        ":online_localization_types",
        "@eigen",
    ],
)

cc_library(
    name = "online_alignment_smoother",
    srcs = [
        "src/localization/online/alignment_recovery.cc",
        "src/localization/online/alignment_smoother.cc",
    ],
    hdrs = [
        "src/localization/online/alignment_recovery.h",
        "src/localization/online/alignment_smoother.h",
    ],
    includes = ["src/localization/online"],
    visibility = ["//visibility:private"],
    deps = [
        ":online_localization_types",
        "@eigen",
        "@gtsam",
    ],
)

cc_library(
    name = "localization_online",
    srcs = ["src/localization/online/localization_engine.cc"],
    hdrs = ["include/openloc/localization/online/localization_engine.h"],
    includes = ["include"],
    visibility = ["//visibility:public"],
    deps = [
        ":online_alignment_smoother",
        ":online_localization_types",
        ":online_odometry_buffer",
        "@eigen",
    ],
)

proto_library(
    name = "localization_health_proto",
    srcs = ["proto/openloc/localization/health/localization_health.proto"],
    strip_import_prefix = "/proto",
    visibility = ["//visibility:public"],
)

cc_proto_library(
    name = "localization_health_proto_cc",
    visibility = ["//visibility:public"],
    deps = [":localization_health_proto"],
)

cc_library(
    name = "health_reason_aggregator",
    srcs = ["src/health/reason_aggregator.cc"],
    hdrs = ["include/openloc/localization/health/reason_aggregator.h"],
    includes = ["include"],
    deps = [":localization_health_proto_cc"],
)

cc_library(
    name = "health_independent_checker",
    srcs = ["src/health/independent_checker.cc"],
    hdrs = ["include/openloc/localization/health/independent_checker.h"],
    includes = ["include"],
    deps = [
        ":health_reason_aggregator",
        ":localization_health_proto_cc",
    ],
)

cc_library(
    name = "directional_constraint_tracker",
    srcs = ["src/health/directional_constraint_tracker.cc"],
    hdrs = ["include/openloc/localization/health/directional_constraint_tracker.h"],
    includes = ["include"],
    deps = [
        ":health_reason_aggregator",
        ":localization_health_proto_cc",
    ],
)

cc_library(
    name = "health_capability_deriver",
    srcs = ["src/health/capability_deriver.cc"],
    hdrs = ["include/openloc/localization/health/capability_deriver.h"],
    includes = ["include"],
    deps = [
        ":health_reason_aggregator",
        ":localization_health_proto_cc",
    ],
)

cc_library(
    name = "health_state_machine",
    srcs = ["src/health/health_state_machine.cc"],
    hdrs = ["include/openloc/localization/health/health_state_machine.h"],
    includes = ["include"],
    deps = [
        ":health_reason_aggregator",
        ":localization_health_proto_cc",
    ],
)

cc_library(
    name = "localization_health",
    srcs = ["src/health/localization_health.cc"],
    hdrs = ["include/openloc/localization/health/localization_health.h"],
    includes = ["include"],
    visibility = ["//visibility:public"],
    deps = [
        ":directional_constraint_tracker",
        ":health_capability_deriver",
        ":health_independent_checker",
        ":health_reason_aggregator",
        ":health_state_machine",
        ":localization_health_proto_cc",
    ],
)

cc_test(
    name = "online_contracts_test",
    size = "small",
    srcs = ["test/localization/online/contracts_test.cc"],
    deps = [
        ":online_localization_types",
        ":online_odometry_buffer",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "openloc_3dmakerpro_smoke_test",
    size = "small",
    srcs = ["test/integration/3dmakerpro/data_smoke_test.cc"],
    deps = ["@com_google_googletest//:gtest_main"],
)

cc_test(
    name = "online_gnss_alignment_dataset_diagnostic_test",
    size = "small",
    srcs = ["test/integration/3dmakerpro/online_gnss_alignment_test.cc"],
    deps = [
        ":localization_online",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "online_alignment_recovery_test",
    size = "small",
    srcs = ["test/localization/online/alignment_recovery_test.cc"],
    deps = [
        ":online_alignment_smoother",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "online_alignment_smoother_test",
    size = "small",
    srcs = ["test/localization/online/alignment_smoother_test.cc"],
    deps = [
        ":online_alignment_smoother",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "online_localization_test",
    size = "small",
    srcs = ["test/localization/online/localization_engine_test.cc"],
    deps = [
        ":localization_online",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "online_gtsam_fixed_lag_spike_test",
    size = "small",
    srcs = ["test/localization/online/gtsam_fixed_lag_spike_test.cc"],
    copts = ["-std=c++17"],
    deps = [
        "@gtsam",
    ],
)

cc_test(
    name = "local_estimator_tests",
    size = "medium",
    timeout = "long",
    srcs = ["test/localization/candidates/local/local_estimator_test.cc"],
    deps = [
        ":local_estimator",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "localization_algorithm_tests",
    size = "medium",
    timeout = "long",
    srcs = glob([
        "test/localization/candidates/common/*_test.cc",
        "test/localization/candidates/global/*_test.cc",
        "test/localization/candidates/local/*_test.cc",
        "test/localization/candidates/relative/*_test.cc",
    ]),
    deps = [
        ":continuous_output",
        ":four_wheel_kinematics",
        ":global_alignment",
        ":planar_wheel_dr",
        ":lane_relative",
        ":local_estimator",
        ":local_lio",
        ":local_motion",
        ":localization_types",
        ":observed_subspace",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "localization_health_tests",
    size = "small",
    srcs = glob(["test/health/*_test.cc"]),
    deps = [
        ":localization_health",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "localization_map_tests",
    size = "small",
    srcs = glob(
        ["test/localization/candidates/map/*_test.cc"],
        exclude = ["test/localization/candidates/map/scan_to_map_test.cc"],
    ),
    deps = [
        ":likelihood_field",
        ":localization_types",
        ":map_matcher",
        ":particle_hypothesis_tracker",
        ":particle_relocalizer",
        ":scan_to_map",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "scan_to_map_tests",
    size = "small",
    srcs = ["test/localization/candidates/map/scan_to_map_test.cc"],
    deps = [
        ":scan_to_map",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_binary(
    name = "public_xy_dataset_replay",
    testonly = True,
    srcs = ["test/integration/map/public_xy_replay.cc"],
    deps = [":localization_online"],
)

cc_binary(
    name = "horizontal_alignment_dataset_replay",
    testonly = True,
    srcs = ["test/integration/map/horizontal_alignment_replay.cc"],
    deps = [":online_alignment_smoother"],
)

cc_binary(
    name = "scan_to_map_dataset_replay",
    srcs = ["test/integration/map/scan_to_map_dataset_replay.cc"],
    deps = [
        ":scan_to_map",
        "@local_config_pcl//:io",
    ],
)

cc_binary(
    name = "three_source_dataset_replay",
    srcs = ["test/integration/map/three_source_replay.cc"],
    deps = [
        ":global_alignment",
        ":scan_to_map",
        "@local_config_pcl//:io",
    ],
)

cc_binary(
    name = "local_estimator_dataset_replay",
    srcs = ["test/integration/chassis_imu_dr/local_estimator_replay.cc"],
    deps = [":local_estimator"],
)

cc_binary(
    name = "planar_wheel_dataset_replay",
    srcs = ["test/integration/chassis_imu_dr/planar_wheel_replay.cc"],
    deps = [":planar_wheel_dr"],
)
