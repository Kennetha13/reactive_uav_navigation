// Checks computeVr = Vs ∩ Vd ∩ Va end to end. A wall at x = wall_x stands in
// for the distance function (see test/fake), so no ROS is needed.

#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include "reactive_uav_navigation/velocity_search_space.hpp"

using namespace reactive_uav_navigation;

static bool near(double a, double b) { return std::fabs(a - b) < 1e-9; }

int main() {
    const VelocityLimits limits;
    const VelocitySteps steps;
    const DynamicWindow window{DynamicWindowParams{}};
    const AdmissibleVelocities admissible{AdmissibleParams{}};
    DronePose pose;
    pose.position.z = 1.0;  // at (0, 0, 1), yaw 0
    const CandidateVelocity still;

    // Wall far away: everything in Vs ∩ Vd is admissible, so Vr is the full grid.
    {
        const DistanceFunction far(10.0);
        const auto vr = computeVr(limits, steps, window, admissible, far, still, pose);
        assert(vr.size() == 7u * 13u * 37u);

        for (const auto& a : vr) {
            assert(a.velocity.vx >= 0.0);  // Vs
            // predicted pose follows eq. 13 over the shared horizon
            const double yaw = a.velocity.wz * VELOCITY_HORIZON;
            assert(near(a.predicted.yaw, yaw));
            assert(near(a.predicted.position.x, a.velocity.vx * VELOCITY_HORIZON * std::cos(yaw)));
            assert(near(a.predicted.position.z, 1.0 + a.velocity.vz * VELOCITY_HORIZON));
            // dCol is the distance measured from the predicted pose
            assert(near(a.dCol, 10.0 - a.predicted.position.x));
        }
    }

    // Wall just ahead: fast candidates fail Va, slow ones survive.
    {
        const DistanceFunction near_wall(0.31);
        const auto vr = computeVr(limits, steps, window, admissible, near_wall, still, pose);
        assert(!vr.empty());
        assert(vr.size() < 7u * 13u * 37u);

        bool has_hover = false;
        for (const auto& a : vr) {
            // every survivor obeys speed <= sqrt(2 * a_max * dCol)
            const double speed = std::hypot(a.velocity.vx, a.velocity.vz);
            assert(speed <= admissible.maxAdmissibleSpeed(a.dCol) + 1e-9 || speed < 1e-9);
            // {0.3, 0, 0} would end 0.01 m from the wall: too fast to stop
            assert(!(near(a.velocity.vx, 0.3) && near(a.velocity.vz, 0.0) &&
                     near(a.velocity.wz, 0.0)));
            if (near(a.velocity.vx, 0.0) && near(a.velocity.vz, 0.0) && near(a.velocity.wz, 0.0)) {
                has_hover = true;
            }
        }
        assert(has_hover);
    }

    // Wall behind the drone: no clearance, so only zero-speed rotations remain.
    {
        const DistanceFunction behind(-1.0);
        const auto vr = computeVr(limits, steps, window, admissible, behind, still, pose);
        assert(vr.size() == 37u);
        for (const auto& a : vr) {
            assert(near(a.velocity.vx, 0.0) && near(a.velocity.vz, 0.0));
        }
    }

    // Vd feeds through: a drone at vx = 0.3 with a 0.1 s horizon can only pick vx in [0.2, 0.3].
    {
        DynamicWindowParams wp;
        wp.dt = 0.1;
        AdmissibleParams ap;
        ap.dt = 0.1;
        const DynamicWindow shortWindow(wp);
        const AdmissibleVelocities shortAdmissible(ap);
        const DistanceFunction far(10.0);
        const CandidateVelocity fast{0.3, 0.0, 0.0};
        const auto vr = computeVr(limits, steps, shortWindow, shortAdmissible, far, fast, pose);
        assert(!vr.empty());
        for (const auto& a : vr) {
            assert(a.velocity.vx >= 0.2 - 1e-9 && a.velocity.vx <= 0.3 + 1e-9);
        }
    }

    // Window and admissible must share a horizon.
    {
        AdmissibleParams ap;
        ap.dt = 0.5;
        const AdmissibleVelocities other(ap);
        const DistanceFunction far(10.0);
        bool threw = false;
        try {
            computeVr(limits, steps, window, other, far, still, pose);
        } catch (const std::invalid_argument&) {
            threw = true;
        }
        assert(threw);
    }

    std::cout << "computeVr tests passed\n";
    return 0;
}
