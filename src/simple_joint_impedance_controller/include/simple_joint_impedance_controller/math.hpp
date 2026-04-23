// Pure-math helpers for the simple joint impedance controller.
//
// Header-only on purpose: the gtest unit tests include this file directly
// and therefore do not need to link against the controller shared library
// or any ROS runtime. See docs/DECISIONS.md ADR-0008 for the scope that
// justifies each helper below.

#ifndef SIMPLE_JOINT_IMPEDANCE_CONTROLLER__MATH_HPP_
#define SIMPLE_JOINT_IMPEDANCE_CONTROLLER__MATH_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace simple_joint_impedance_controller {

// Critical-damping auto-fill: any negative entry in `d` is replaced with
// 2 * sqrt(max(k[i], 0)). Non-negative entries pass through unchanged.
// Returns false on length mismatch so a caller mistake cannot produce a
// garbage gain vector.
inline bool auto_fill_critical_damping(const std::vector<double>& k, std::vector<double>& d) {
    if (k.size() != d.size()) {
        return false;
    }
    for (std::size_t i = 0; i < d.size(); ++i) {
        if (d[i] < 0.0) {
            const double k_i = std::max(k[i], 0.0);
            d[i] = 2.0 * std::sqrt(k_i);
        }
    }
    return true;
}

// Elementwise clamp of `q_d` into [lower, upper]. Returns false on any
// length mismatch or inverted bound.
inline bool clamp_to_limits(std::vector<double>& q_d, const std::vector<double>& lower,
                            const std::vector<double>& upper) {
    if (q_d.size() != lower.size() || q_d.size() != upper.size()) {
        return false;
    }
    for (std::size_t i = 0; i < q_d.size(); ++i) {
        if (lower[i] > upper[i]) {
            return false;
        }
        q_d[i] = std::min(std::max(q_d[i], lower[i]), upper[i]);
    }
    return true;
}

// Per-joint PD law: tau[i] = k[i] * (q_d[i] - q[i]) + d[i] * (qdot_d[i] - qdot[i]).
// qdot_d may be empty (treated as zero feed-forward velocity).
inline bool compute_pd_torque(const std::vector<double>& q, const std::vector<double>& q_d,
                              const std::vector<double>& qdot, const std::vector<double>& qdot_d,
                              const std::vector<double>& k, const std::vector<double>& d,
                              std::vector<double>& tau) {
    const std::size_t n = q.size();
    if (q_d.size() != n || qdot.size() != n || k.size() != n || d.size() != n) {
        return false;
    }
    const bool has_ff = !qdot_d.empty();
    if (has_ff && qdot_d.size() != n) {
        return false;
    }
    tau.assign(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        const double qdot_err = has_ff ? (qdot_d[i] - qdot[i]) : (-qdot[i]);
        tau[i] = k[i] * (q_d[i] - q[i]) + d[i] * qdot_err;
    }
    return true;
}

// Rate limit: clamp (tau - tau_prev) to [-max_delta, max_delta] per joint.
// Produces a fresh vector so callers can log both pre- and post-saturation.
inline bool saturate_torque_rate(const std::vector<double>& tau,
                                 const std::vector<double>& tau_prev,
                                 const std::vector<double>& max_delta, std::vector<double>& out) {
    const std::size_t n = tau.size();
    if (tau_prev.size() != n || max_delta.size() != n) {
        return false;
    }
    out.assign(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        if (max_delta[i] < 0.0) {
            return false;
        }
        const double delta = tau[i] - tau_prev[i];
        const double clamped = std::min(std::max(delta, -max_delta[i]), max_delta[i]);
        out[i] = tau_prev[i] + clamped;
    }
    return true;
}

// Absolute clamp of each torque entry into [-tau_max[i], +tau_max[i]].
// Runs last in the pipeline so the hardware interface never sees a value
// outside the hard safety limit.
inline bool saturate_torque_abs(std::vector<double>& tau, const std::vector<double>& tau_max) {
    if (tau.size() != tau_max.size()) {
        return false;
    }
    for (std::size_t i = 0; i < tau.size(); ++i) {
        if (tau_max[i] < 0.0) {
            return false;
        }
        tau[i] = std::min(std::max(tau[i], -tau_max[i]), tau_max[i]);
    }
    return true;
}

// Validate a sensor_msgs/JointState-shaped target (passed as raw fields so the
// helper stays ROS-free and unit-testable). Returns true iff the message is
// usable as a new reference configuration for `expected_joints`, in which case
// `q_d_out` is populated and `qdot_d_out` is either the feed-forward velocity
// (if the message carried a matching-length velocity vector) or empty.
//
// Rules (ADR-0008 keep-list #7):
//  - msg_positions must have length equal to expected_joints.size();
//  - if msg_names is non-empty, it must equal expected_joints element-wise
//    (strict, order-matching — no permutation reordering in M3);
//  - msg_velocities is optional: empty means "zero feed-forward", otherwise
//    it must match the same length and be finite;
//  - all position and velocity entries must be finite.
// Length or order mismatch, non-finite entries, or empty expected_joints
// return false without touching the output vectors.
inline bool validate_target_joint_state(const std::vector<std::string>& msg_names,
                                        const std::vector<double>& msg_positions,
                                        const std::vector<double>& msg_velocities,
                                        const std::vector<std::string>& expected_joints,
                                        std::vector<double>& q_d_out,
                                        std::vector<double>& qdot_d_out) {
    const std::size_t n = expected_joints.size();
    if (n == 0) {
        return false;
    }
    if (msg_positions.size() != n) {
        return false;
    }
    if (!msg_names.empty()) {
        if (msg_names.size() != n) {
            return false;
        }
        for (std::size_t i = 0; i < n; ++i) {
            if (msg_names[i] != expected_joints[i]) {
                return false;
            }
        }
    }
    for (double p : msg_positions) {
        if (!std::isfinite(p)) {
            return false;
        }
    }
    if (!msg_velocities.empty()) {
        if (msg_velocities.size() != n) {
            return false;
        }
        for (double v : msg_velocities) {
            if (!std::isfinite(v)) {
                return false;
            }
        }
        qdot_d_out = msg_velocities;
    } else {
        qdot_d_out.clear();
    }
    q_d_out = msg_positions;
    return true;
}

} // namespace simple_joint_impedance_controller

#endif // SIMPLE_JOINT_IMPEDANCE_CONTROLLER__MATH_HPP_
