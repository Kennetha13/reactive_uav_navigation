#pragma once
#include <vector>
#include <optional>
#include <algorithm>
#include <cmath>

/* 

    objective function interface SKELETON

*/
namespace reactive_uav_navigation {

    struct VelocityCandidate { 
        // all velocities are in m/s
        double vx = 0.0;
        double vy = 0.0;
        double vz = 0.0;

    };

    inline double score_heading(const VelocityCandidate& c) {

        // placeholder for heading function
        return 0.6;
    }

    inline double score_distance(const VelocityCandidate& c) {

        // placeholder for distance function
        return 0.7;
    }

    // max speed in m/s.
    // will later match Vs.
    constexpr double MAX_SPEED = 1.0;

    inline double score_velocity(const VelocityCandidate& c) {

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
    inline double score_candidate(const VelocityCandidate& c) {

        double sum = (score_heading(c) * ALPHA) + (score_distance(c) * BETA) + (score_velocity(c) * GAMMA);
        return sum;
    }

    inline std::optional<VelocityCandidate> select_best_candidate(const std::vector<VelocityCandidate>& candidates) {

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

}

