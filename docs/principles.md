# Openloc Functional Principles

These principles define the localization outcomes Openloc must provide.
Architecture and implementation choices are subordinate to them.

## Relative localization serves the drivable area

The primary outcome is a reliable relative estimate sufficient to determine
the vehicle's drivable area. Openloc does not require globally absolute
position to remain centimeter-accurate in every operating condition. Absolute
or map-referenced position is useful when available, but its uncertainty must
be represented honestly and it must not be a prerequisite for useful relative
motion estimation.

Accuracy and availability requirements are scenario-specific and must be
measured against declared operating needs; this principle does not invent a
universal numeric threshold.

## The system assesses and reports its own trustworthiness

Openloc must expose enough state and evidence to assess whether an output is
usable, degraded, or untrusted. It must not present low-confidence output as
normal-quality localization. When confidence or observability is below the
applicable operating requirement, the reported state and downstream output
must degrade explicitly and provide evaluable reasons.

A numerical covariance alone is not proof of calibrated accuracy.

## Global-reference recovery is a first-class capability

Loss of GNSS or a material global-reference change must lead to an explicit
reacquisition and recovery process, not permanent failure or an implicit
return to trusted status. The system reports recovery progress, confirms
when the global estimate is trustworthy again, and otherwise remains
degraded or untrusted. This does not require prior-map relocalization.

GNSS reacquisition preserves local ODOM ownership and continuity. Any
required ENU-origin or global-alignment reset is explicit and sequenced so
consumers can retire stale state.
