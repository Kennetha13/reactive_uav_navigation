#include <gtest/gtest.h>
#include "reactive_uav_navigation/velocity_candidate.hpp"
#include <cmath>
#include <vector>

using namespace reactive_uav_navigation;

// 1. empty vector -> no value
TEST(VelocityCandidate, EmptyReturnsNothing) {
    std::vector<CandidateTerms> emptyVect{};
    std::optional<CandidateVelocity> empty = select_best_candidate(emptyVect);
    EXPECT_FALSE(empty.has_value());
}

// 2. highest G wins
TEST(VelocityCandidate, HighestWins) {
    CandidateTerms a{{0.2, 0.0, 0.0}, 0.5, 0.5, 0.2};
    CandidateTerms b{{0.5, 0.0, 0.0}, 0.6, 0.6, 0.5};
    CandidateTerms c{{0.9, 0.0, 0.0}, 0.8, 0.8, 0.9};
    std::vector<CandidateTerms> v{a, b, c};
    auto result = select_best_candidate(v);
    ASSERT_TRUE(result.has_value());
    EXPECT_NEAR(result->vx, c.cmd.vx, 1e-9);
}

// 3. identical scores -> first wins
TEST(VelocityCandidate, TieGoesToFirst) {
    CandidateTerms tie_a{{0.2, 0.0, 0.0}, 0.5, 0.5, 0.5};
    CandidateTerms tie_b{{0.7, 0.0, 0.0}, 0.5, 0.5, 0.5};
    std::vector<CandidateTerms> tie{tie_a, tie_b};
    auto tie_result = select_best_candidate(tie);
    ASSERT_TRUE(tie_result.has_value());
    EXPECT_NEAR(tie_result->vx, tie_a.cmd.vx, 1e-9);
}

// 4. first has a NaN term -> second wins
TEST(VelocityCandidate, NanSkipped) {
    CandidateTerms nan_first{{0.3, 0.0, 0.0}, std::nan(""), 0.5, 0.5};
    CandidateTerms valid_low{{0.6, 0.0, 0.0}, 0.1, 0.1, 0.1};
    std::vector<CandidateTerms> nan_case{nan_first, valid_low};
    auto nan_result = select_best_candidate(nan_case);
    ASSERT_TRUE(nan_result.has_value());
    EXPECT_NEAR(nan_result->vx, valid_low.cmd.vx, 1e-9);
}

// 5. all NaN -> no value
TEST(VelocityCandidate, NanRetNothing) {
    CandidateTerms bad_a{{0.4, 0.0, 0.0}, std::nan(""), 0.5, 0.5};
    CandidateTerms bad_b{{0.8, 0.0, 0.0}, 0.5, std::nan(""), 0.5};
    std::vector<CandidateTerms> all_nan{bad_a, bad_b};
    EXPECT_FALSE(select_best_candidate(all_nan).has_value());

}
