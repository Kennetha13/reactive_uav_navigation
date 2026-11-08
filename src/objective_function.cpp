#include "reactive_uav_navigation/objective_function.hpp"

#include <algorithm>
#include <cmath>

#include "reactive_uav_navigation/heading_function.hpp"
#include "reactive_uav_navigation/distance_function.hpp"
#include "reactive_uav_navigation/velocity_function.hpp"


/*
initialize with goal also:
ObjectiveFunction(this, *drone_state_, goal);


 Before each velocity candidate is run, do:
 objective_function_->setGoal(target);
 objective_function_->prepareCandidates(vr);



*/
namespace reactive_uav_navigation
{

ObjectiveFunction::ObjectiveFunction(
    rclcpp::Node* node,
    const DroneState& drone_state,
    const geometry_msgs::msg::Point& goal
)
    : drone_state_(drone_state)
{
    // Instantiate the three scoring functions.
    heading_function_ =
        std::make_unique<HeadingFunction>(drone_state_, goal);

    distance_function_ =
        std::make_unique<DistanceFunction>(DistanceParams{});

    velocity_function_ =
        std::make_unique<VelocityFunction>(node);
}

ObjectiveFunction::~ObjectiveFunction() = default;

void ObjectiveFunction::setGoal(const geometry_msgs::msg::Point& goal)
{
    heading_function_->setGoal(goal);
}


void ObjectiveFunction::prepareCandidates(
    const std::vector<CandidateVelocity>& candidates
)
{
    // HeadingFunction needs the maximum vertical error
    // across all candidates to normalize its score.
    max_z_error_ = heading_function_->getMaxZError(candidates);
}


double ObjectiveFunction::computeObjective(
    const CandidateVelocity& candidate
)
{
    return computeScores(candidate).total;
}


ObjectiveScores ObjectiveFunction::computeScores(
    const CandidateVelocity& candidate
)
{
    // -----------------------------------------
    // 1. HEADING SCORE
    // -----------------------------------------

    double heading_score =
        heading_function_->computeHeading(
            candidate,
            max_z_error_
        );


    // -----------------------------------------
    // 2. DISTANCE SCORE
    // -----------------------------------------

    const auto current_position =
    drone_state_.get_current_position();

    // Estimate the drone's future position.
    const double predicted_yaw =
    drone_state_.get_yaw() + candidate.wz * CONTROL_PERIOD;

    geometry_msgs::msg::Vector3 predicted_position;

    predicted_position.x = 
        current_position.x + 
        candidate.vx * CONTROL_PERIOD * std::cos(predicted_yaw);

    predicted_position.y = 
        current_position.y + 
        candidate.vx * CONTROL_PERIOD * std::sin(predicted_yaw);

    predicted_position.z = 
        current_position.z + 
        candidate.vz * CONTROL_PERIOD;

    const double predicted_pitch = std::atan2(candidate.vz, std::abs(candidate.vx));

    double distance_score =
    distance_function_->computeDistance(
        predicted_position,
        predicted_yaw,
        predicted_pitch);


    // -----------------------------------------
    // 3. VELOCITY SCORE
    // -----------------------------------------

    // Convert candidate velocity to the format
    // expected by VelocityFunction.
    geometry_msgs::msg::Vector3 velocity;

    velocity.x = candidate.vx;
    velocity.y = 0.0;
    velocity.z = candidate.vz;

    double yaw_heading =
        heading_function_->computeYawHeading(candidate);

    double velocity_score =
        velocity_function_->computeVelocity(
            velocity,
            heading_function_->K_z,
            heading_function_->K_yaw,
            yaw_heading
        );


    // -----------------------------------------
    // 4. COMBINE SCORES
    // -----------------------------------------

    // Clamp scores to the range [0, 1].
    heading_score =
        std::clamp(heading_score, 0.0, 1.0);

    distance_score =
        std::clamp(distance_score, 0.0, 1.0);

    velocity_score =
        std::clamp(velocity_score, 0.0, 1.0);

    // G(v) = alpha * Heading(v)
    //      + beta  * Distance(v)
    //      + gamma * Velocity(v)

    ObjectiveScores scores;
    scores.heading = heading_score;
    scores.distance = distance_score;
    scores.velocity = velocity_score;
    scores.total = (alpha_ * heading_score)
                 + (beta_ * distance_score)
                 + (gamma_ * velocity_score);
    return scores;
}

} // namespace reactive_uav_navigation
