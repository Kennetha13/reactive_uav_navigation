#pragma once
#include "reactive_uav_navigation/common.hpp"
#include <vector>
#include <optional>
#include <algorithm>
#include <cmath>

/* 

    objective function interface

*/
namespace reactive_uav_navigation {


    // weights taken from table 2 in pdf
    constexpr double ALPHA = 0.3;
    constexpr double BETA = 0.6;
    constexpr double GAMMA = 0.1;

    /*
        scoring algorithm.
        alpha * heading: how much drone cares about moving towards goal
        beta * distance: how much drone cares about clearance from obstacles
        gamma * velocity: how much drone cares about speed
        higher a is more goal-driven, b is more cautious, and g for speed
    */
    inline double score_candidate(double heading, double distance, double velocity) {

        double sum = (heading * ALPHA) + (distance * BETA) + (velocity * GAMMA);
        return sum;
    }

    struct CandidateTerms {
        CandidateVelocity cmd;
        double heading_score = 0.0;
        double distance_score = 0.0;
        double velocity_score = 0.0;
    };

    // ties go to first candidate
    // NaN scores are skipped
    inline std::optional<CandidateVelocity> select_best_candidate(const std::vector<CandidateTerms>& candidates) {

        if (candidates.empty()) return std::nullopt;

        std::optional<CandidateVelocity> best;
        double best_score = 0.0;

        for (const auto& cand : candidates) {
            
            double curr_score = score_candidate(cand.heading_score, cand.distance_score, cand.velocity_score);

            if (std::isnan(curr_score)) continue;
            if (!best || curr_score > best_score ) {
                best = cand.cmd;
                best_score = curr_score;
            }
        }

        return best;
    }

}

