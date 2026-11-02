// Checks Vs, Vd, their intersection and candidate sampling. Plain asserts,
// no ROS needed.

#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include "reactive_uav_navigation/dynamic_window.hpp"
#include "reactive_uav_navigation/velocity_space.hpp"

using namespace reactive_uav_navigation;

static bool near(double a, double b) { return std::fabs(a - b) < 1e-9; }

int main() {
    const VelocityLimits limits;
    const VelocitySteps steps;
    const DynamicWindow window{DynamicWindowParams{}};

    // Vs from the paper's defaults
    const VelocitySpace vs = computeMaxVelocitySpace(limits);
    assert(near(vs.min.vx, 0.0) && near(vs.max.vx, 0.3));
    assert(near(vs.min.vz, -0.3) && near(vs.max.vz, 0.3));
    assert(near(vs.max.wz, degToRad(45.0)) && near(vs.min.wz, -degToRad(45.0)));

    // sampling includes both ends and 0: vx 0..0.3 step 0.05 -> 7 values
    assert(sampleInterval(0.0, 0.3, 0.05).size() == 7);
    assert(sampleInterval(-0.3, 0.3, 0.05).size() == 13);
    assert(sampleInterval(-degToRad(45.0), degToRad(45.0), degToRad(2.5)).size() == 37);

    // stationary drone: Vd is wider than Vs, so Vr == Vs and the grid is 7*13*37
    const CandidateVelocity still;
    const auto all = generateCandidates(limits, steps, still, window);
    assert(all.size() == 7u * 13u * 37u);

    // drone at vx = 0.3 with a short window: vx in [0.2, 0.3]
    const CandidateVelocity fast{0.3, 0.0, 0.0};
    DynamicWindowParams shortParams;
    shortParams.dt = 0.1;
    const DynamicWindow shortWindow(shortParams);
    const VelocitySpace vd = shortWindow.compute(fast);
    const IntersectionResult vr = DynamicWindow::intersect(vs, vd);
    assert(vr.valid);
    assert(near(vr.space.min.vx, 0.2) && near(vr.space.max.vx, 0.3));

    // Vd alone can go negative on vx; Vr never does
    const auto cands = generateCandidates(limits, steps, still, window);
    for (const auto& c : cands) {
        assert(c.vx >= 0.0);
    }

    // disjoint boxes are invalid
    VelocitySpace far = vd;
    far.min.vx = far.max.vx = 5.0;
    assert(!DynamicWindow::intersect(vs, far).valid);
    CandidateVelocity far_cur{5.0, 0.0, 0.0};
    assert(generateCandidates(limits, steps, far_cur, shortWindow).empty());

    // bad params are rejected up front
    DynamicWindowParams bad;
    bad.dt = 0.0;
    bool threw = false;
    try { DynamicWindow w(bad); } catch (const std::invalid_argument&) { threw = true; }
    assert(threw);

    std::cout << "velocity pipeline tests passed\n";
    return 0;
}
