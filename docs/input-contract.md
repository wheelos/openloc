# Openloc Sensor Input Contract

**Status:** working input contract for G100/Mid-360 localization, not a
qualified device specification. Raw sensor values and provenance must remain
available to adapters; uncertain units, frames, or clock domains must not be
silently normalized or guessed.

## Contract for every input

- Preserve measurement time, declared clock domain, frame ID, units, sensor
  identity, calibration identity, and source quality/status. Distinguish
  sensor measurement timestamps from bag or host receive timestamps.
- Represent transforms as `T_parent_child`; record both frames and the
  calibration source. Do not infer a physical frame from a ROS `frame_id`,
  PCD `VIEWPOINT`, filename, or identity-valued transform.
- Keep optional sources optional. A missing camera, GNSS receiver, or
  reference observation must not invalidate a LiDAR/IMU-only relative result.
- Keep sensor-invalid/sentinel records identifiable. Report counts before and
  after each filter, and make any range or quality policy explicit and
  configurable.
- Do not treat a vendor estimate, a fitted map alignment, or an estimator's
  self-consistency as independent accuracy truth.
- Runtime GNSS/RTK may constrain the separate global alignment
  `T_enu_odom`; it must never alter the local ODOM owner. Convert geodetic
  positions using an explicit local ENU origin and declared datum. A prior
  point-cloud map or map-to-ENU anchor is not required for this target.

  For this recording, 162 `/GPSGGA` epochs report receiver quality 4
  (RTK-fixed), but GNSS measurement uncertainty is uncalibrated and there is no
  independent route reference. GNSS factors remain disabled by the current
  admission decision. A full-recording GNSS-off FAST-LIO2 baseline and
  same-record consistency diagnostic now exist; they do not exercise Openloc's
  own L1 implementation or GNSS fusion. See
  [`plan/g100-rtk-data-validation.md`](plan/g100-rtk-data-validation.md).

## Selected development dataset: 2026-10-08 G100 recording

The active development dataset is
`/mnt/synology/3dmakerpro/20261008141912/`. Initial read-only inventory found
`LIDAR_20261008141912.bag`, `GPS_20261008141912.bag`,
`RTK_20261008141912.bin`, `diff_data.rtcm`, `gga.txt`,
`calibration/calib.json`, and thumbnail outputs. Schema, sensor, time, and
RTK findings are maintained in
`/mnt/synology/3dmakerpro/schema/confirmed-findings.md`,
`evidence-details.md`, `fast-lio2-mid360-profile-20261008.yaml`, and
`rtk-factor-admission-20261008.md`.

The current validation scope is LiDAR/IMU ODOM plus GNSS-aided global
alignment; no prior map is required and the thumbnail PLY is not needed.
GNSS must be converted to a declared local ENU frame before estimating
`T_enu_odom`. For this dataset, 162 `/GPSGGA` epochs report RTK fixed, but
uncertainty is uncalibrated and independent truth is unavailable; GNSS factors
are currently disabled. A GNSS-off FAST-LIO2 trajectory has been checked for
basic continuity and compared with the same-record GNSS track, but this is not
an Openloc algorithm result or an accuracy qualification. See
[`plan/g100-rtk-data-validation.md`](plan/g100-rtk-data-validation.md) for
the single-dataset validation gates.

For NMEA GGA, preserve the reported altitude `H` and geoid separation `N`
separately, and derive ellipsoidal height as `h = H + N` only after validating
the sentence checksum and meter units. The selected `gga.txt` sample
`H=93.5572 m`, `N=-2.0266 m` yields `h=91.5306 m`. This adapter-level
conversion does not establish the local ENU origin, vertical uncertainty, or
permission to admit a vertical GNSS factor; those remain gated separately.

## 3DMakerPro and Mid-360 evidence boundary

The available schema directory contains
`0729-使用手册 RayZoom G100 Manual V1.3.01.pdf` and
`0804-用户指南-RayStudio For Windows Manual V1.2.01.pdf`. It does **not**
contain the Mid-360 user manual v1.2 or its quick-start guide. The G100
manual's PDF pages 12–14 cover export UI, not the requested Mid-360 coordinate
or tag diagrams. Mid-360-specific statements below were supplied by the user
as manual findings; retain their qualifications until the cited Mid-360 PDF
figures and text can be checked.

The RayStudio manual available in the schema directory documents the RTK
processing flow, `RTK_*.bin` and `GPS_*.bag` export names, base-station files
with `.25o`/`.25p` extensions, and registration with at least three UTM
control-point coordinates. It describes the base as stationary. This supports
requiring base data for the vendor RTK workflow, but the `.bin` protocol and
whether its payload is rover observations, correction data, or a mixture are
not established by the manual text or current file scan. Treat PPK as the
current likely interpretation, not a decoded format fact.

RayStudio's control-point operation transforms a point cloud into a world
coordinate system. It is an alignment input, not proof of a graph-optimization
factor. Points used to fit that transform cannot also score accuracy; reserve
independent points for evaluation. Treat `recalculate_trj.txt` as a
vendor-estimated comparison baseline, not independent truth.

## Conditional Mid-360 contract

Apply these constraints only after the device's LiDAR and IMU are identified
as a Mid-360. The RayZoom G100's use of a Mid-360 remains an inference from
reported scan/IMU characteristics; confirm it with device serial number or
firmware/model information. An additional separately mounted IMU would not
inherit the Mid-360 internal-IMU extrinsic.

| Input property | Current interpretation | Evidence and required handling |
|---|---|---|
| LiDAR axes and origin | Reported convention is a right-handed O-XYZ frame with +X forward, +Y left, +Z up. The reported spherical convention is zenith angle from +Z and azimuth from +X. | User-cited Mid-360 manual v1.2, but its page-12 figure is not available here. Do not assume Z is vertical in the G100 installation or any exported map until the figure and mounting transform are verified. |
| Internal IMU frame and lever arm | The user-reported manual says the internal IMU axes match the point-cloud axes and places the IMU origin at `p_L_I = [0.011, 0.02329, -0.04412] m`. Under that convention, `R_L_I = I`; the inverse translation is `p_I_L = [-0.011, -0.02329, 0.04412] m`. | Conditional on Mid-360 internal IMU and manual interpretation. The value in `lidar2imu.txt` matches the inverse translation numerically, but direction, units, sensor applicability, and calibration revision are not declared in that file. Confirm the vendor figure and the active runtime configuration before using it. |
| G100-to-Mid-360 identity | For the 20261008141912 profile, host device JMK2 serial JMK2001144 is identified with Livox Mid-360 and ICM-40609 IMU. | See `fast-lio2-mid360-profile-20261008.yaml`. Do not generalize this identity to other sessions without checking their provenance. |
| Point coordinates and sentinels | The user-reported manual says no-return/out-of-range points are emitted as `(0,0,0)`. One inspected session has 12.3486% exact-zero XYZ. | Treat zero XYZ as a likely no-return sentinel, not generic corruption, pending the manual source check. Exclude zero-range points from geometric estimation and retain their count as a scene/data-quality metric. The measured ratio is session-specific. |
| `tag` | Provisional bit layout: `[7:6]` reserved, `[5:4]` other, `[3:2]` rain/fog/dust, `[1:0]` close-object adhesion; values `0/1/2/3` reportedly mean good/medium/poor/reserved. | The page-14 table has not been checked. The user also reports FAST-LIO retains `(tag & 0x30)` values `0x00` or `0x10`, which examines only `[5:4]`. No corresponding FAST-LIO runtime configuration was found in the inspected Openloc/module paths. Do not hard-code these semantics or filter policy; keep raw tags and report group/value counts until the table and active implementation are verified. |
| Range policy | User-reported limits say near returns below 0.2 m are reference-only; about 0.5 m may be more practical for a hand-held G100. The reported range is 40 m at 10% reflectivity and 70 m at 80%, with shorter range toward the upper vertical field. | These are candidate policies/specifications, not Openloc defaults or per-session guarantees. Make any threshold explicit and measure excluded counts. The observed 51.5 m maximum is not by itself an invalid-point signal. |
| Measurement uncertainty | User-reported specifications are range error `1σ <= 2 cm` at 10 m and angular error `1σ <= 0.15°`. | Sensor specifications are not a calibrated per-session covariance. Use only as provisional weighting priors; account for range-dependent lateral uncertainty (about 8 cm at 30 m for 0.15°) and validate against independent data. |
| Reflectivity | User-reported manual interpretation maps values 0–150 to diffuse returns and 151–255 to retroreflection, with larger error within 2 m. | Provisional until the manual table and point-field mapping are checked. It may support a camera-independent reflective-target experiment, but does not establish marker identity or detection performance. |
| IMU | For the profiled 20261008141912 device, ICM-40609 is recorded at 200 Hz; acceleration source unit is `g`, gyro is `rad/s`, and orientation is invalid. | Apply the profile's FAST-LIO2 adapter scale exactly once (`9.80665` to m/s²) and use its initialization rules. GPS/PPS discipline remains unverified. |
| LiDAR/IMU time | Audited bag headers/timebase use Unix/POSIX UTC nanoseconds; sampled LiDAR record times are about 0.10 s later than embedded scan header time. | Associate with embedded sensor timestamps, not bag record time. Numeric overlap with GPS/RMC does not prove hardware PPS synchronization or absolute timing accuracy. |
| Stored extrinsic settings | The user-reported manual says the ranging unit stores but does not apply its extrinsic setting. | Raw LiDAR-frame output is therefore a candidate interpretation, not guaranteed: the ROS driver or exporter may still transform points. Inspect the active driver/export configuration and verify against raw messages. |
| Factory calibration | The user reports the device is factory-calibrated and calibration can be re-imported from USB/RayStudio. `calib.json` has identical hashes in the nine inspected sessions. | The file appears camera-oriented; neither its sensor coverage nor whether it contains a LiDAR-IMU extrinsic is established. Identical files do not prove session applicability or independent calibration validation. |
| Camera transform | The user-reported interpretation of `calib.json` is `T_camera_lidar`; its rotation suggests the camera optical axis is tilted by about 30° relative to the LiDAR +X axis. | This remains an inference until the matrix parent/child frames and direction are confirmed. Do not compose it with the conditional Mid-360 transform or infer the G100 body mounting transform yet. |
| Scan-view settings | Point-cloud size/density, coloring, and background settings are reported to change only the live view, not recorded scan data. | Exclude these display-only controls from raw input lineage; keep actual export/filter/processing configuration distinct. This behavior is manual-reported, not independently verified against device output. |
| Cameras | Continuous capture is reported as 12 MP for coloring/Gaussian splatting; fixed-point capture as 48 MP for panoramas. Night mode may disable cameras, a lower-tier model may omit left/right/top cameras, and frame rate is configurable. | Availability, dimensions, and timing are per-session/device evidence, not fixed input requirements. Image and marker observations are optional; do not require a camera or marker detector for LiDAR/IMU validity. |

## GNSS, vendor products, and truth

- Treat the dataset's 162 `/GPSGGA` epochs marked quality 4 as
  receiver-reported RTK-fixed status, not calibrated position uncertainty or
  independent truth. The existing track diagnostic is GNSS-only and does not
  validate LIO or fusion.
- The RTK binary contains NMEA and CRC-valid RTCM3 observations/navigation
  frames; the exact vendor framing and receiver role remain unproven. No
  matching base RINEX files or verified PPK solve are available. Do not use
  the binary or built-in GPS as independent pose truth.
- Treat vendor recalculation/filter/sampling/color products as derived
  outputs. `.amap` remains private and unparsed. Do not promote a vendor
  trajectory or a control-point-fitted map to independent truth.
- Record which control points participated in alignment and score only on
  held-out points with known coordinate provenance.

## Open input gates

1. Verify the selected bag, FAST-LIO2 executable/configuration, and resolved
   parameters against the profile's recorded identities before replay.
2. Verify first-sample initialization suitability and preserve embedded
   measurement timestamps; bag record timestamps are not sensor timestamps.
3. Establish GPS/PPS lock and absolute clock accuracy, the GNSS antenna lever
   arm, and independent/calibrated GNSS uncertainty before production factor
   admission.
4. Obtain independent route reference or calibrate GNSS uncertainty against
   such a reference before making absolute accuracy claims.

## Offline checks before consuming the recording

- For a genuinely stationary segment, calculate the mean acceleration vector
  and its norm in the original IMU frame. Compare candidate unit scales and
  orientations without silently rotating into a presumed vehicle frame; a
  single pose cannot establish all axis signs or the G100 mounting transform.
- Compare LiDAR header time, `timebase`, per-point `offset_time`, IMU header
  time, and GPS RMC time only after recording each field's numeric unit and
  candidate epoch. A numeric match or GPS-like sentence does not by itself
  prove a shared hardware clock, PPS lock, or UTC epoch.
- Report exact-zero returns, candidate near-range exclusions (for example
  `<0.2 m` and `<0.5 m`), and each provisional `tag` group separately. Keep
  these as per-session diagnostics until the tag figure, point-field mapping,
  and chosen filtering policy are confirmed.
- If camera data are present, compare dimensions and timestamps by session;
  do not make marker detection a required path. A reflective-return experiment
  can be evaluated independently but must not be called marker detection
  without an explicit detection model and validation.

Until these gates close, Openloc may consume clearly labelled IMU-relative
data for a GNSS-off relative experiment when its required LiDAR/IMU transform
and time relationship are supplied and validated. IMU-to-body is not required
for an IMU-relative local map; it is required only for body-frame output or
vehicle-frame gravity interpretation.
