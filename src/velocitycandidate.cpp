#include <vector>
#include <optional>
#include <algorithm>
#include <cmath>
#include <cassert>
#include <iostream>

/* 

    objective function interface SKELETON

*/

struct VelocityCandidate { 
    // all velocities are in m/s
    double vx = 0.0;
    double vy = 0.0;
    double vz = 0.0;

};

double score_heading(const VelocityCandidate& c) {

    // placeholder for heading function
    return 0.6;
}

double score_distance(const VelocityCandidate& c) {

    // placeholder for distance function
    return 0.7;
}

// max speed in m/s.
// will later match Vs.
constexpr double MAX_SPEED = 1.0;

double score_velocity(const VelocityCandidate& c) {

    double speed = std::sqrt( (c.vx * c.vx) + (c.vy * c.vy) + (c.vz * c.vz));
    double score = speed / MAX_SPEED;
    score = std::clamp(score, 0.0, 1.0);

    return score;
}

// placeholder values for weights
constexpr double ALPHA = 1.0;
constexpr double BETA = 1.0;
constexpr double GAMMA = 1.0;

/*
    scoring algorithm.
    alpha * heading: how much drone cares about moving towards goal
    beta * distance: how much drone cares about clearance from obstacles
    gamma * velocity: how much drone cares about speed
    higher a is more goal-driven, b is more cautious, and g for speed
*/
double score_candidate(const VelocityCandidate& c) {

    double sum = (score_heading(c) * ALPHA) + (score_distance(c) * BETA) + (score_velocity(c) * GAMMA);
    return sum;
}

std::optional<VelocityCandidate> select_best_candidate(const std::vector<VelocityCandidate>& candidates) {

    if (candidates.empty()) return std::nullopt;

    VelocityCandidate best = candidates[0];
    double best_score = score_candidate(candidates[0]);

    for (const auto& cand : candidates) {
        
        double curr_score = score_candidate(cand);
        if (curr_score > best_score) {
            best = cand;
            best_score = curr_score;
        }
    }

    return best;
}

// FOR TESTING PURPOSES

/* 

int main() {

    VelocityCandidate slow{0.2, 0.0, 0.0};
    VelocityCandidate medium{0.5, 0.0, 0.0};
    VelocityCandidate fast{0.9, 0.0, 0.0};

    std::vector<VelocityCandidate> emptyVect{};
    std::vector<VelocityCandidate> v{slow, medium, fast};

    std::optional<VelocityCandidate> empty = select_best_candidate(emptyVect);
    assert(!empty.has_value());

    auto result = select_best_candidate(v);
    assert(result.has_value());
    assert(std::fabs(result->vx - fast.vx) < 1e-9);

    std::cout << "winner vx = " << result->vx << "\n";

    return 0;
}
    */