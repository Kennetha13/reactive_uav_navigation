// Checks Vs, Vd, their intersection and candidate sampling. Plain asserts,
// no ROS needed.

#include <cassert>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>

#include "reactive_uav_navigation/vss_dynamic_window.hpp"
#include "reactive_uav_navigation/vss_collector.hpp"

using namespace reactive_uav_navigation;

static bool near(double a, double b) { return std::fabs(a - b) < 1e-9; }

int main() {
    const VelocityLimits limits;
    const MaximumVelocitiesSpace maximum{limits, VelocitySteps{}};

    // Vs from the paper's defaults
    const VelocitySpace vs = maximum.compute();
    assert(near(vs.min.vx, 0.0) && near(vs.max.vx, 0.3));
    assert(near(vs.min.vz, -0.3) && near(vs.max.vz, 0.3));
    assert(near(vs.max.wz, degToRad(45.0)) && near(vs.min.wz, -degToRad(45.0)));

    // sampling includes both ends and 0: vx 0..0.3 step 0.05 -> 7 values
    assert(sampleInterval(0.0, 0.3, 0.05).size() == 7);
    assert(sampleInterval(-0.3, 0.3, 0.05).size() == 13);
    assert(sampleInterval(-degToRad(45.0), degToRad(45.0), degToRad(2.5)).size() == 37);

    // stationary drone: Vd is wider than Vs, so Vr == Vs and the grid is 7*13*37.
    // A far wall keeps every candidate admissible, so computeVr returns the whole grid.
    const VssCollector collector{std::make_unique<MaximumVelocitiesSpace>(limits, VelocitySteps{})};
    const DistanceFunction clear(10.0);
    const DronePose pose;
    const CandidateVelocity still;
    const auto all = collector.computeVr(clear, still, pose);
    assert(all.size() == 7u * 13u * 37u);

    // Vd alone can go negative on vx; Vr never does
    for (const auto& c : all) {
        assert(c.vx >= 0.0);
    }

    // drone at vx = 0.3 with a short window: vx in [0.2, 0.3]
    const CandidateVelocity fast{0.3, 0.0, 0.0};
    DynamicWindowParams shortParams;
    shortParams.dt = 0.1;
    const DynamicWindow shortWindow(shortParams);
    const VelocitySpace vd = shortWindow.compute(fast);
    const IntersectionResult vr = VssCollector::intersectVelocitySpaces(vs, vd);
    assert(vr.valid);
    assert(near(vr.space.min.vx, 0.2) && near(vr.space.max.vx, 0.3));

    // disjoint boxes are invalid, and give no candidates
    VelocitySpace far = vd;
    far.min.vx = far.max.vx = 5.0;
    assert(!VssCollector::intersectVelocitySpaces(vs, far).valid);
    AdmissibleParams shortAdmissible;
    shortAdmissible.dt = 0.1;
    const VssCollector shortCollector{
        std::make_unique<MaximumVelocitiesSpace>(limits, VelocitySteps{}),
        AccelerationLimits{}, shortAdmissible};
    const CandidateVelocity far_cur{5.0, 0.0, 0.0};
    assert(shortCollector.computeVr(clear, far_cur, pose).empty());

    // PX4's limits only ever lower the caps
    {
        MaximumVelocitiesSpace m{VelocityLimits{}, VelocitySteps{}};
        Px4Limits px4;
        px4.vx_max = 12.0;                // PX4 default, far above the 0.3 cap
        px4.vz_max_up = 3.0;
        px4.vz_max_down = 1.5;
        px4.wz_max = degToRad(45.0);
        m.setPx4Limits(px4);
        assert(near(m.limits().vx_max, 0.3));  // cap wins
        assert(near(m.limits().vz_max, 0.3));
        assert(near(m.limits().wz_max, degToRad(45.0)));

        px4.vx_max = 0.2;                 // PX4 slower than the cap: PX4 wins
        px4.vz_max_down = 0.1;            // vz uses the smaller of up and down
        m.setPx4Limits(px4);
        assert(near(m.limits().vx_max, 0.2));
        assert(near(m.limits().vz_max, 0.1));
        assert(near(m.compute().max.vx, 0.2) && near(m.compute().min.vz, -0.1));

        // nothing received yet: caps alone
        MaximumVelocitiesSpace fresh{VelocityLimits{}, VelocitySteps{}};
        assert(near(fresh.limits().vx_max, 0.3));
    }

    // PX4's accelerations only ever lower the caps (1, 1, 100 deg/s^2)
    {
        DynamicWindow w{DynamicWindowParams{}};
        Px4AccelerationLimits px4;
        px4.ax = 3.0;                    // PX4 defaults, above the caps
        px4.az_up = 4.0;
        px4.az_down = 3.0;
        px4.yaw_accel = degToRad(60.0);
        w.setPx4Limits(px4);
        assert(near(w.accel().ax, 1.0));                      // cap wins
        assert(near(w.accel().az, 1.0));
        assert(near(w.accel().yaw_accel, degToRad(60.0)));    // PX4 is lower here

        px4.ax = 0.5;                    // PX4 slower than the cap: PX4 wins
        px4.az_down = 0.2;               // az uses the smaller of up and down
        w.setPx4Limits(px4);
        assert(near(w.accel().ax, 0.5));
        assert(near(w.accel().az, 0.2));
        // and Vd follows: a stationary drone over the 1 s horizon
        const VelocitySpace vd = w.compute(CandidateVelocity{});
        assert(near(vd.max.vx, 0.5 * VELOCITY_HORIZON) && near(vd.min.vz, -0.2 * VELOCITY_HORIZON));

        // nothing received yet: caps alone
        DynamicWindow fresh{DynamicWindowParams{}};
        assert(near(fresh.accel().ax, 1.0));
    }

    // bad caps are rejected up front
    {
        VelocityLimits badLimits;
        badLimits.vx_max = 0.0;
        bool rejected = false;
        try { MaximumVelocitiesSpace m{badLimits, VelocitySteps{}}; } catch (const std::invalid_argument&) { rejected = true; }
        assert(rejected);
    }

    // bad params are rejected up front
    DynamicWindowParams bad;
    bad.dt = 0.0;
    bool threw = false;
    try { DynamicWindow w(bad); } catch (const std::invalid_argument&) { threw = true; }
    assert(threw);

    std::cout << "velocity pipeline tests passed\n";
    return 0;
}
