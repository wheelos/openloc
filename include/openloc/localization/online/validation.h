#ifndef OPENLOC_LOCALIZATION_ONLINE_VALIDATION_H_
#define OPENLOC_LOCALIZATION_ONLINE_VALIDATION_H_

#include "openloc/localization/online/types.h"

namespace openloc {
namespace localization {

Status ValidateTimestamp(const Timestamp& timestamp);
Status ValidateFrameId(const FrameId& frame);
Status ValidatePose(const Pose& pose, double quaternion_norm_tolerance = 1e-6);
Status ValidateCovariance(const Eigen::MatrixXd& covariance,
                          Eigen::Index dimension);
Status ValidateMeasurement(const Measurement& measurement);
Status ValidateOdometrySample(const OdometrySample& sample);

Eigen::Index MeasurementDimension(MeasurementType type);

}  // namespace localization
}  // namespace openloc

#endif  // OPENLOC_LOCALIZATION_ONLINE_VALIDATION_H_
