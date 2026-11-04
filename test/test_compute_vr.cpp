// Checks VssCollector::computeVr = Vs ∩ Vd ∩ Va end to end. A wall at
// x = wall_x stands in for the distance function (see test/fake), so no ROS is
// needed.

#include <cassert>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>

#include "reactive_uav_navigation/vss_collector.hpp"

using namespace reactive_uav_navigation;

static bool near(double a, double b) { return std::fabs(a - b) < 1e-9; }

static std::unique_ptr<MaximumVelocitiesSpace> defaultMaximum() {
    return std::make_unique<MaximumVelocitiesSpace>(VelocityLimits{}, VelocitySteps{});
}

int main() {
    const VssCollector collector{defaultMaximum()};
    DronePose pose;
    pose.position.z = 1.0;  // at (0, 0, 1), yaw 0
    const CandidateVelocity still;

    // Wall far away: everything in Vs ∩ Vd is admissible, so Vr is the full grid.
    {
        const DistanceFunction far(10.0);
        const auto vr = collector.computeVr(far, still, pose);
        assert(vr.size() == 7u * 13u * 37u);

        for (const auto& v : vr) {
            assert(v.vx >= 0.0);  // Vs
            // the predicted pose follows eq. 13 over the shared horizon
            const DronePose p = collector.admissible().predictPose(pose, v);
            const double yaw = v.wz * VELOCITY_HORIZON;
            assert(near(p.yaw, yaw));
            assert(near(p.position.x, v.vx * VELOCITY_HORIZON * std::cos(yaw)));
            assert(near(p.position.z, 1.0 + v.vz * VELOCITY_HORIZON));
        }
    }

    // Wall just ahead: fast candidates fail Va, slow ones survive.
    {
        const DistanceFunction near_wall(0.31);
        const auto vr = collector.computeVr(near_wall, still, pose);
        assert(!vr.empty());
        assert(vr.size() < 7u * 13u * 37u);

        bool has_hover = false;
        for (const auto& v : vr) {
            // every survivor obeys speed <= sqrt(2 * a_max * dCol), with dCol
            // measured from its predicted pose
            const double dCol = near_wall.computeDistMin(
                collector.admissible().predictPose(pose, v).position, 0.0, 0.0);
            const double speed = std::hypot(v.vx, v.vz);
            assert(speed <= collector.admissible().maxAdmissibleSpeed(dCol) + 1e-9 ||
                   speed < 1e-9);
            // {0.3, 0, 0} would end 0.01 m from the wall: too fast to stop
            assert(!(near(v.vx, 0.3) && near(v.vz, 0.0) && near(v.wz, 0.0)));
            if (near(v.vx, 0.0) && near(v.vz, 0.0) && near(v.wz, 0.0)) {
                has_hover = true;
            }
        }
        assert(has_hover);
    }

    // Wall behind the drone: no clearance, so only zero-speed rotations remain.
    {
        const DistanceFunction behind(-1.0);
        const auto vr = collector.computeVr(behind, still, pose);
        assert(vr.size() == 37u);
        for (const auto& v : vr) {
            assert(near(v.vx, 0.0) && near(v.vz, 0.0));
        }
    }

    // Vd feeds through, with Vd and Va sharing one horizon: a drone at
    // vx = 0.3 with a 0.1 s horizon can only pick vx in [0.2, 0.3].
    {
        AdmissibleParams ap;
        ap.dt = 0.1;
        const VssCollector shortCollector{defaultMaximum(), AccelerationLimits{}, ap};
        assert(near(shortCollector.window().params().dt, 0.1));
        assert(near(shortCollector.admissible().params().dt, 0.1));

        const DistanceFunction far(10.0);
        const CandidateVelocity fast{0.3, 0.0, 0.0};
        const auto vr = shortCollector.computeVr(far, fast, pose);
        assert(!vr.empty());
        for (const auto& v : vr) {
            assert(v.vx >= 0.2 - 1e-9 && v.vx <= 0.3 + 1e-9);
        }
    }

    // The default collector uses the shared horizon for both.
    assert(near(collector.window().params().dt, VELOCITY_HORIZON));
    assert(near(collector.admissible().params().dt, VELOCITY_HORIZON));

    // A collector needs its maximum velocity space.
    {
        bool threw = false;
        try {
            VssCollector bad{std::unique_ptr<MaximumVelocitiesSpace>{}};
        } catch (const std::invalid_argument&) {
            threw = true;
        }
        assert(threw);
    }

    std::cout << "computeVr tests passed\n";
    return 0;
}
