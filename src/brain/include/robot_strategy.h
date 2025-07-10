#pragma once

#include <cmath>
#include <vector>
#include <map>
#include "types.h"

/**
 * Robot physical and performance constants
 */
struct RobotConstants {
    double maxForwardSpeed = 0.9;       // m/s
    double maxAngularSpeed = 1.45;      // rad/s - for facing/movement turns
    double maxAdjustAngularSpeed = 0.9; // rad/s - slower for precise kicking adjustments
    double robotRadius = 0.15;          // m
    double kickRange = 0.3;             // m
};

/**
 * Robot strategy calculator for cost functions and movement planning
 */
class RobotStrategy {
public:
    RobotStrategy(const RobotConstants& constants = RobotConstants(), 
                  const FieldDimensions& fieldDims = FD_ADULTSIZE);

    /**
     * Calculate cost function for robot to reach ball based on actual time required
     * Lower cost = better choice (faster to execute)
     * 
     * Strategy: We are attacking the right goal (+x direction)
     * Process: 1) Turn to face ball, 2) Walk to optimal position, 3) Turn to face goal
     */
    double calculateCostFunction(const Pose2D& robotPose, const Point2D& ballPos) const;

    /**
     * Calculate the optimal position behind the ball for attacking the right goal
     */
    Point2D getOptimalPositionBehindBall(const Point2D& ballPos) const;

    /**
     * Find the robot with minimum cost to reach the ball
     * Returns robot index, or -1 if no robots
     */
    int getMinCostRobot(const std::vector<Pose2D>& robotPoses, const Point2D& ballPos) const;

    /**
     * Get detailed movement plan for a robot to reach ball
     */
    struct MovementPhase {
        std::string action;
        std::string description;
        double time;
        Point2D startPos;
        Point2D endPos;
        double startAngle = 0;
        double endAngle = 0;
    };

    struct MovementPlan {
        std::vector<MovementPhase> phases;
        double totalTime;
        Point2D optimalPosition;
        double finalOrientation;
    };

    MovementPlan getRobotMovementPlan(const Pose2D& robotPose, const Point2D& ballPos) const;

private:
    RobotConstants robotConstants;
    FieldDimensions fieldDims;

    /**
     * Normalize angle to [-PI, PI] range
     */
    double normalizeAngle(double angle) const;

    /**
     * Calculate shortest rotation angle between two angles
     */
    double shortestRotation(double from, double to) const;
}; 