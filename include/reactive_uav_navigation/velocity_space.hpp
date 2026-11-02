// velocity_space.hpp
// maximum velocity space Vs and candidate sampling for dwa-3d

#ifndef VELOCITY_SPACE_HPP_
#define VELOCITY_SPACE_HPP_

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "reactive_uav_navigation/common.hpp"
#include "reactive_uav_navigation/dynamic_window.hpp"

namespace reactive_uav_navigation {

// converts degrees to radians
inline double degToRad(double degrees) {
    return degrees * M_PI / 180.0;
}

// converts radians to degrees
inline double radToDeg(double radians) {
    return radians * 180.0 / M_PI;
}

// maximum velocities of the drone
struct VelocityLimits {
    double vx_max = 0.3;             // m/s
    double vz_max = 0.3;             // m/s
    double wz_max = degToRad(45.0);  // rad/s

    // throws if any limit is invalid
    void validate() const {
        if (!std::isfinite(vx_max) || vx_max <= 0.0) {
            throw std::invalid_argument(
                "vx_max must be greater than 0 (got " + std::to_string(vx_max) + ")");
        }
        if (!std::isfinite(vz_max) || vz_max < 0.0) {
            throw std::invalid_argument(
                "vz_max must be 0 or greater (got " + std::to_string(vz_max) + ")");
        }
        if (!std::isfinite(wz_max) || wz_max < 0.0) {
            throw std::invalid_argument(
                "wz_max must be 0 or greater (got " + std::to_string(wz_max) + ")");
        }
    }
};

// spacing between sampled velocities
struct VelocitySteps {
    double vx_step = 0.05;           // m/s
    double vz_step = 0.05;           // m/s
    double wz_step = degToRad(2.5);  // rad/s

    // throws if any step is invalid
    void validate() const {
        if (!std::isfinite(vx_step) || vx_step <= 0.0) {
            throw std::invalid_argument(
                "vx_step must be greater than 0 (got " + std::to_string(vx_step) + ")");
        }
        if (!std::isfinite(vz_step) || vz_step <= 0.0) {
            throw std::invalid_argument(
                "vz_step must be greater than 0 (got " + std::to_string(vz_step) + ")");
        }
        if (!std::isfinite(wz_step) || wz_step <= 0.0) {
            throw std::invalid_argument(
                "wz_step must be greater than 0 (got " + std::to_string(wz_step) + ")");
        }
    }
};

// builds Vs: vx in [0, vx_max], vz in [-vz_max, vz_max], wz in [-wz_max, wz_max]
inline VelocitySpace computeMaxVelocitySpace(const VelocityLimits& limits) {
    limits.validate();

    VelocitySpace vs;
    vs.min = CandidateVelocity{0.0, -limits.vz_max, -limits.wz_max};
    vs.max = CandidateVelocity{limits.vx_max, limits.vz_max, limits.wz_max};
    return vs;
}

// returns values from lo to hi spaced by step, including both ends and 0 (if inside)
inline std::vector<double> sampleInterval(double lo, double hi, double step) {
    if (!std::isfinite(step) || step <= 0.0) {
        throw std::invalid_argument("step must be greater than 0");
    }

    std::vector<double> values;
    if (lo > hi) {
        return values;
    }

    const double eps = 1e-9;  // floating-point tolerance

    values.push_back(lo);

    // multiples of step strictly between lo and hi
    const int firstK = static_cast<int>(std::ceil((lo + eps) / step));
    const int lastK = static_cast<int>(std::floor((hi - eps) / step));
    for (int k = firstK; k <= lastK; ++k) {
        values.push_back(k * step);
    }

    if (hi > lo + eps) {
        values.push_back(hi);
    }
    return values;
}

// returns every combination of sampled vx, vz and wz values inside the box
inline std::vector<CandidateVelocity> sampleVelocitySpace(const VelocitySpace& space,
                                                          const VelocitySteps& steps) {
    steps.validate();

    const std::vector<double> vxValues = sampleInterval(space.min.vx, space.max.vx, steps.vx_step);
    const std::vector<double> vzValues = sampleInterval(space.min.vz, space.max.vz, steps.vz_step);
    const std::vector<double> wzValues = sampleInterval(space.min.wz, space.max.wz, steps.wz_step);

    std::vector<CandidateVelocity> candidates;
    candidates.reserve(vxValues.size() * vzValues.size() * wzValues.size());

    for (double vx : vxValues) {
        for (double vz : vzValues) {
            for (double wz : wzValues) {
                candidates.push_back(CandidateVelocity{vx, vz, wz});
            }
        }
    }
    return candidates;
}

// Vs ∩ Vd sampled on the step grid; empty if the boxes do not overlap.
// The result still has to go through AdmissibleVelocities::filterAdmissible (Va).
inline std::vector<CandidateVelocity> generateCandidates(
    const VelocityLimits& limits,
    const VelocitySteps& steps,
    const CandidateVelocity& current,
    const DynamicWindow& window) {
    const VelocitySpace vs = computeMaxVelocitySpace(limits);
    const VelocitySpace vd = window.compute(current);
    const IntersectionResult vr = DynamicWindow::intersect(vs, vd);
    if (!vr.valid) {
        return {};
    }
    return sampleVelocitySpace(vr.space, steps);
}

}  // namespace reactive_uav_navigation

#endif  // VELOCITY_SPACE_HPP_
