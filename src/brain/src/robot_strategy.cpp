#include "robot_strategy.h"
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <limits>
#include <cmath>

RobotStrategy::RobotStrategy(const RobotConstants& constants, const FieldDimensions& fieldDims)
    : robotConstants(constants), fieldDims(fieldDims) {
}

double RobotStrategy::normalizeAngle(double angle) const {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle <= -M_PI) angle += 2 * M_PI;
    return angle;
}

double RobotStrategy::shortestRotation(double from, double to) const {
    double diff = normalizeAngle(to - from);
    return std::abs(diff);
}

Point2D RobotStrategy::getOptimalPositionBehindBall(const Point2D& ballPos) const {
    // Goal position (center of right goal)
    double goalX = fieldDims.length / 2.0;
    double goalY = 0.0;
    
    // Calculate optimal position behind the ball
    double ballToGoalAngle = std::atan2(goalY - ballPos.y, goalX - ballPos.x);
    double optimalDistance = robotConstants.robotRadius + 0.1; // 10cm margin
    
    Point2D optimalPos;
    optimalPos.x = ballPos.x - optimalDistance * std::cos(ballToGoalAngle);
    optimalPos.y = ballPos.y - optimalDistance * std::sin(ballToGoalAngle);
    
    return optimalPos;
}

double RobotStrategy::calculateCostFunction(const Pose2D& robotPose, const Point2D& ballPos) const {
    // Goal position (center of right goal)
    double goalX = fieldDims.length / 2.0;
    double goalY = 0.0;
    
    // Calculate optimal position behind the ball
    double ballToGoalAngle = std::atan2(goalY - ballPos.y, goalX - ballPos.x);
    Point2D optimalPos = getOptimalPositionBehindBall(ballPos);
    
    // PHASE 1: Time to turn to face the optimal position
    double angleToOptimal = std::atan2(optimalPos.y - robotPose.y, optimalPos.x - robotPose.x);
    double turn1Angle = shortestRotation(robotPose.theta, angleToOptimal);
    double turn1Time = turn1Angle / robotConstants.maxAngularSpeed;
    
    // PHASE 2: Time to walk forward to optimal position (robots can't walk backwards)
    double distanceToOptimal = std::sqrt(std::pow(robotPose.x - optimalPos.x, 2) + 
                                        std::pow(robotPose.y - optimalPos.y, 2));
    double walkTime = distanceToOptimal / robotConstants.maxForwardSpeed;
    
    // PHASE 3: Time to turn to face the goal (precise adjustment for kicking)
    double finalOrientation = ballToGoalAngle;  // Face towards goal
    double turn2Angle = shortestRotation(angleToOptimal, finalOrientation);
    double turn2Time = turn2Angle / robotConstants.maxAdjustAngularSpeed;
    
    // Total time is the sum of all phases
    double totalTime = turn1Time + walkTime + turn2Time;
    
    // Add penalties for suboptimal positioning
    double penalty = 0;
    
    // Penalty if robot is currently on wrong side of ball
    double robotToBallAngle = std::atan2(ballPos.y - robotPose.y, ballPos.x - robotPose.x);
    double angleDiffToGoal = shortestRotation(robotToBallAngle, ballToGoalAngle);
    if (angleDiffToGoal > M_PI / 2) {
        penalty += 1.0;  // 1 second penalty for being on wrong side
    }
    
    // Penalty if robot is very far from ball (> 5m)
    if (distanceToOptimal > 5.0) {
        penalty += (distanceToOptimal - 5.0) * 0.2;  // 0.2s per meter beyond 5m
    }
    
    return totalTime + penalty;
}

int RobotStrategy::getMinCostRobot(const std::vector<Pose2D>& robotPoses, const Point2D& ballPos) const {
    if (robotPoses.empty()) {
        return -1;
    }
    
    double minCost = std::numeric_limits<double>::infinity();
    int minRobot = -1;
    
    for (size_t i = 0; i < robotPoses.size(); ++i) {
        double cost = calculateCostFunction(robotPoses[i], ballPos);
        if (cost < minCost) {
            minCost = cost;
            minRobot = static_cast<int>(i);
        }
    }
    
    return minRobot;
}

RobotStrategy::MovementPlan RobotStrategy::getRobotMovementPlan(const Pose2D& robotPose, 
                                                                const Point2D& ballPos) const {
    // Goal position
    double goalX = fieldDims.length / 2.0;
    double goalY = 0.0;
    
    // Calculate optimal position behind the ball
    double ballToGoalAngle = std::atan2(goalY - ballPos.y, goalX - ballPos.x);
    Point2D optimalPos = getOptimalPositionBehindBall(ballPos);
    
    // Phase 1: Turn to face optimal position
    double angleToOptimal = std::atan2(optimalPos.y - robotPose.y, optimalPos.x - robotPose.x);
    double turn1Angle = shortestRotation(robotPose.theta, angleToOptimal);
    double turn1Time = std::max(0.01, turn1Angle / robotConstants.maxAngularSpeed);
    
    // Phase 2: Walk forward to optimal position
    double distanceToOptimal = std::sqrt(std::pow(robotPose.x - optimalPos.x, 2) + 
                                        std::pow(robotPose.y - optimalPos.y, 2));
    double walkTime = std::max(0.01, distanceToOptimal / robotConstants.maxForwardSpeed);
    
    // Phase 3: Turn to face goal
    double finalOrientation = ballToGoalAngle;
    double turn2Angle = shortestRotation(angleToOptimal, finalOrientation);
    double turn2Time = std::max(0.01, turn2Angle / robotConstants.maxAdjustAngularSpeed);
    
    MovementPlan plan;
    plan.totalTime = turn1Time + walkTime + turn2Time;
    plan.optimalPosition = optimalPos;
    plan.finalOrientation = finalOrientation;
    
    // Phase 1: Turn
    MovementPhase phase1;
    phase1.action = "turn";
    std::ostringstream oss1;
    oss1 << "Quick turn " << std::fixed << std::setprecision(1) << (turn1Angle * 180.0 / M_PI) << "° to face target";
    phase1.description = oss1.str();
    phase1.time = turn1Time;
    phase1.startPos = {robotPose.x, robotPose.y};
    phase1.endPos = {robotPose.x, robotPose.y};
    phase1.startAngle = robotPose.theta;
    phase1.endAngle = angleToOptimal;
    plan.phases.push_back(phase1);
    
    // Phase 2: Walk
    MovementPhase phase2;
    phase2.action = "walk";
    std::ostringstream oss2;
    oss2 << "Walk " << std::fixed << std::setprecision(2) << distanceToOptimal << "m forward";
    phase2.description = oss2.str();
    phase2.time = walkTime;
    phase2.startPos = {robotPose.x, robotPose.y};
    phase2.endPos = optimalPos;
    plan.phases.push_back(phase2);
    
    // Phase 3: Turn
    MovementPhase phase3;
    phase3.action = "turn";
    std::ostringstream oss3;
    oss3 << "Precise adjust " << std::fixed << std::setprecision(1) << (turn2Angle * 180.0 / M_PI) << "° to face goal";
    phase3.description = oss3.str();
    phase3.time = turn2Time;
    phase3.startPos = optimalPos;
    phase3.endPos = optimalPos;
    phase3.startAngle = angleToOptimal;
    phase3.endAngle = finalOrientation;
    plan.phases.push_back(phase3);
    
    return plan;
} 