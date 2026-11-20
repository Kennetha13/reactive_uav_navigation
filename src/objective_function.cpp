#include "reactive_uav_navigation/objective_function.hpp"

#include <algorithm>
#include <cmath>

#include "reactive_uav_navigation/heading_function.hpp"
#include "reactive_uav_navigation/distance_function.hpp"
#include "reactive_uav_navigation/velocity_function.hpp"


namespace reactive_uav_navigation
{

ObjectiveFunction::ObjectiveFunction(
    const DroneState& drone_state,
    const geometry_msgs::msg::Point& goal,
    DistanceFunction& distance_function
)
    : distance_function_(distance_function),
      drone_state_(drone_state),
      goal_(goal)
{
    // Instantiate the three scoring functions.
    heading_function_ =
        std::make_unique<HeadingFunction>(drone_state_, goal);

    velocity_function_ =
        std::make_unique<VelocityFunction>();
}

ObjectiveFunction::~ObjectiveFunction() = default;

void ObjectiveFunction::setGoal(const geometry_msgs::msg::Point& goal)
{
    goal_ = goal;
    heading_function_->setGoal(goal);
}


void ObjectiveFunction::setMaxSpeed(double vel_max)
{
    vel_max_ = vel_max;
}


BestVelocity ObjectiveFunction::selectBest(
    const std::vector<CandidateVelocity>& candidates
)
{
    BestVelocity best;

    // HeadingFunction needs the whole candidate set before any one is scored.
    prepareCandidates(candidates);

    // hovering and turning on the spot are always safe, so check
    // whether anything actually moves the drone
    for (size_t i = 0; i < candidates.size(); i++) {
        if (std::hypot(candidates[i].vx, candidates[i].vz) > 1e-9) {
            best.can_move = true;
        }
    }

    // The waypoint is behind the drone or outside the camera's view: turn
    // toward it on the spot before anything else. A turn moves the drone
    // toward nothing, so it is safe, and scoring it like a movement would
    // hold it back whenever the nose has to swing past a nearby wall.
    const double turn_direction = turnDirectionToGoal();
    if (turn_direction != 0.0) {
        for (size_t i = 0; i < candidates.size(); i++) {
            const bool stays_in_place =
                std::abs(candidates[i].vx) < 1e-9 && std::abs(candidates[i].vz) < 1e-9;
            if (!stays_in_place) {
                continue;
            }

            // the fastest turn in the right direction
            const double turn_rate = candidates[i].wz * turn_direction;
            if (turn_rate > 0.0 && (!best.found || turn_rate > best.velocity.wz * turn_direction)) {
                best.found = true;
                best.velocity = candidates[i];
            }
        }
        if (best.found) {
            best.scores = computeScores(best.velocity);

            // the turn approaches nothing, so it is not held against it that
            // the camera cannot see where the nose will point afterwards
            best.scores.distance = 1.0;
            return best;
        }
    }

    for (size_t i = 0; i < candidates.size(); i++) {
        ObjectiveScores scores = computeScores(candidates[i]);
        if (std::isnan(scores.total)) {
            continue;
        }

        if (!best.found || scores.total > best.scores.total) {
            best.found = true;
            best.velocity = candidates[i];
            best.scores = scores;
        }
    }

    return best;
}


double ObjectiveFunction::turnDirectionToGoal() const
{
    const auto position = drone_state_.get_current_position();
    const double goal_dx = goal_.x - position.x;
    const double goal_dy = goal_.y - position.y;

    // a waypoint almost straight above or below has no direction to turn to
    if (std::hypot(goal_dx, goal_dy) < 0.3) {
        return 0.0;
    }

    // angle from the drone's heading to the waypoint, wrapped into [-pi, pi]
    double yaw_error = std::atan2(goal_dy, goal_dx) - drone_state_.get_yaw();
    while (yaw_error > M_PI) {
        yaw_error = yaw_error - 2.0 * M_PI;
    }
    while (yaw_error < -M_PI) {
        yaw_error = yaw_error + 2.0 * M_PI;
    }

    // The camera can see toward the waypoint and it is in front of the
    // drone: no turn needed first. If the camera cannot see that way, the
    // drone looks before it flies there.
    const double goal_yaw = std::atan2(goal_dy, goal_dx);
    if (std::abs(yaw_error) <= M_PI / 2.0 && distance_function_.isDirectionVisible(goal_yaw)) {
        return 0.0;
    }

    if (yaw_error > 0.0) {
        return 1.0;
    }
    return -1.0;
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

    // Estimate the drone's future position over the same horizon the
    // admissible velocities (Va) use. One control period is too short: every
    // candidate would end at almost the same pose and get the same score.
    const double predicted_yaw =
    drone_state_.get_yaw() + candidate.wz * VELOCITY_HORIZON;

    geometry_msgs::msg::Vector3 predicted_position;

    predicted_position.x =
        current_position.x +
        candidate.vx * VELOCITY_HORIZON * std::cos(predicted_yaw);

    predicted_position.y =
        current_position.y +
        candidate.vx * VELOCITY_HORIZON * std::sin(predicted_yaw);

    predicted_position.z =
        current_position.z +
        candidate.vz * VELOCITY_HORIZON;

    const double predicted_pitch = std::atan2(candidate.vz, std::abs(candidate.vx));

    // distance from the predicted position to the waypoint
    const double goal_dx = goal_.x - predicted_position.x;
    const double goal_dy = goal_.y - predicted_position.y;
    const double goal_dz = goal_.z - predicted_position.z;
    const double distance_to_goal =
        std::sqrt(goal_dx * goal_dx + goal_dy * goal_dy + goal_dz * goal_dz);

    // an obstacle behind the waypoint does not count against the candidate
    double distance_score =
    distance_function_.computeDistanceUpTo(
        predicted_position,
        predicted_yaw,
        predicted_pitch,
        distance_to_goal);

    // The rays only look ahead. Also keep away from obstacles beside the
    // predicted position, such as a corner the drone is turning around.
    const double proximity =
        distance_function_.computeProximity(predicted_position);
    if (proximity < distance_score) {
        distance_score = proximity;
    }


    // -----------------------------------------
    // 3. VELOCITY SCORE
    // -----------------------------------------

    double yaw_heading =
        heading_function_->computeYawHeading(candidate);

    double velocity_score =
        velocity_function_->computeVelocity(
            candidate.vx,
            vel_max_,
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
