#include <cmath>
#include <limits>
#include "brain_tree.h"
#include "brain.h"
#include "utils/math.h"
#include "utils/print.h"
#include "utils/misc.h"
#include "std_msgs/msg/string.hpp"

/**
 * Here, a macro definition is used to reduce the amount of code in RegisterBuilder.
 * The effect after expanding REGISTER_BUILDER(Test) is as follows:
 * factory.registerBuilder<Test>(  \
 *      "Test",                    \
 *      [this](const string& name, const NodeConfig& config) { return make_unique<Test>(name, config, brain); });
 */
#define REGISTER_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [this](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });

void BrainTree::init()
{
    BehaviorTreeFactory factory;

    // Action Nodes
    REGISTER_BUILDER(RobotFindBall)
    REGISTER_BUILDER(Chase)
    REGISTER_BUILDER(ChaseToTarget)
    REGISTER_BUILDER(SimpleChase)
    REGISTER_BUILDER(Adjust)
    REGISTER_BUILDER(Kick)
    REGISTER_BUILDER(StrikerDecide)
    REGISTER_BUILDER(CamTrackBall)
    REGISTER_BUILDER(CamFindBall)
    REGISTER_BUILDER(CamScanField)
    REGISTER_BUILDER(SelfLocate)
    REGISTER_BUILDER(SetVelocity)
    REGISTER_BUILDER(CheckAndStandUp)
    REGISTER_BUILDER(RotateForRelocate)
    REGISTER_BUILDER(MoveToPoseOnField)
    REGISTER_BUILDER(GoalieDecide)
    REGISTER_BUILDER(WaveHand)
    REGISTER_BUILDER(GoBackInField)
    REGISTER_BUILDER(TurnOnSpot)
    REGISTER_BUILDER(TurnByAngle)
    REGISTER_BUILDER(TurnToAngle)
    REGISTER_BUILDER(GoToTeammateBall)
    REGISTER_BUILDER(FollowTeammate)
    REGISTER_BUILDER(TangentialAdjust)

    // GoalKeeper Nodes
    REGISTER_BUILDER(GoalKeeperPosition)
    REGISTER_BUILDER(GoalKeeperIntercept)
    REGISTER_BUILDER(GoalKeeperTrackAndAdjust)
    REGISTER_BUILDER(GoalKeeperYAxisDefense)

    // Action Nodes for debug
    REGISTER_BUILDER(PrintMsg)

    factory.registerBehaviorTreeFromFile(brain->config->treeFilePath);
    tree = factory.createTree("MainTree");

    // init blackboard entry
    initEntry();
}

void BrainTree::initEntry()
{
    setEntry<string>("player_role", brain->config->playerRole);
    setEntry<int>("player_id", brain->config->playerId);
    setEntry<bool>("ball_location_known", false);
    setEntry<bool>("track_ball", true);
    setEntry<bool>("odom_calibrated", false);
    setEntry<string>("decision", "");
    setEntry<string>("defend_decision", "chase");
    setEntry<double>("ball_range", 0);
    setEntry<double>("ball_yaw_to_robot", 0.0);
    setEntry<double>("ball_x", 0.0);
    setEntry<double>("ball_y", 0.0);
    setEntry<double>("robot_pose_theta", 0.0);
    setEntry<double>("robot_pose_x", 0.0);
    setEntry<double>("robot_pose_y", 0.0);
    setEntry<bool>("is_closer_to_forward_orientation", true);
    setEntry<bool>("is_facing_forward", true);
    setEntry<bool>("ball_within_head_tracking_range", false);
    setEntry<bool>("ball_far_left_or_right", false);

    setEntry<bool>("gamecontroller_isKickOff", true);
    setEntry<bool>("gamecontroller_isKickOffExecuted", true);

    setEntry<string>("gc_game_state", "");
    setEntry<string>("gc_game_sub_state_type", "NONE");
    setEntry<string>("gc_game_sub_state", "");
    setEntry<bool>("gc_is_kickoff_side", false);
    setEntry<bool>("gc_is_sub_state_kickoff_side", false);
    setEntry<bool>("gc_is_under_penalty", false);

    setEntry<bool>("treat_person_as_robot", false);
    setEntry<int>("control_state", 0);
    setEntry<bool>("B_pressed", false);

    // fallRecovery related
    setEntry<bool>("should_recalibrate_after_fall_recovery", false);

    setEntry<bool>("we_just_scored", false);
    setEntry<bool>("wait_for_opponent_kickoff", false);

    // Penalty point localization related
    setEntry<bool>("trigger_penalty_point_localize", false);

    // Time related
    setEntry<double>("current_time", 0.0);

    // Collaboration related
    setEntry<bool>("has_ball_possession", false);
    setEntry<int>("possession_player_id", -1);
    setEntry<double>("ball_cost", 0.0);
    setEntry<bool>("is_master_robot", false);
    
    // Dynamic role assignment related
    setEntry<int>("dynamic_role", -1);                  // 0=striker, 1=goal_keeper, 2=striker_follower (-1 if unknown)
    setEntry<int>("goal_keeper_player_id", -1);         // Player ID assigned as goal keeper (-1 if unknown)
    setEntry<int>("striker_player_id", -1);             // Player ID assigned as main striker (-1 if unknown)
    setEntry<int>("follower_player_id", -1);            // Player ID assigned as follower striker (-1 if unknown)
    setEntry<bool>("is_dynamic_striker", false);        // True if this robot is dynamically assigned as striker
    setEntry<bool>("is_dynamic_goal_keeper", false);    // True if this robot is dynamically assigned as goal keeper
    setEntry<bool>("is_dynamic_follower", false);       // True if this robot is dynamically assigned as follower

    // Goalkeeper positioning related
    setEntry<bool>("is_at_goalkeeper_position", false); // True if robot is at the correct goalkeeper position
}

void BrainTree::tick()
{
    static int cnt = 0;
    cnt++;
    // print states
    if (cnt % 30 == 0)
        prtDebug(format(
            "GameState: %s\tIsKickOffSide: %d\nScore: %d\t JustScored: %d",
            getEntry<string>("gc_game_state").c_str(),
            getEntry<bool>("gc_is_kickoff_side"),
            brain->data->lastScore,
            getEntry<bool>("we_just_scored")));

    tree.tickOnce();
}

NodeStatus SetVelocity::tick()
{
    double x, y, theta;
    vector<double> targetVec;
    getInput("x", x);
    getInput("y", y);
    getInput("theta", theta);

    auto res = brain->client->setVelocity(x, y, theta);
    return NodeStatus::SUCCESS;
}

NodeStatus CamTrackBall::tick()
{
    double pitch, yaw;
    if (!brain->data->ballDetected)
    {
        pitch = brain->data->ball.pitchToRobot;
        yaw = brain->data->ball.yawToRobot;
    }
    else
    {
        const double pixTolerance = 10;

        double deltaX = mean(brain->data->ball.boundingBox.xmax, brain->data->ball.boundingBox.xmin) - brain->config->camPixX / 2;
        double deltaY = mean(brain->data->ball.boundingBox.ymax, brain->data->ball.boundingBox.ymin) - brain->config->camPixY * 2 / 3;

        if (std::fabs(deltaX) < pixTolerance && std::fabs(deltaY) < pixTolerance)
        {
            return NodeStatus::SUCCESS;
        }

        double smoother = 1.5;
        double deltaYaw = deltaX / brain->config->camPixX * brain->config->camAngleX / smoother;
        double deltaPitch = deltaY / brain->config->camPixY * brain->config->camAngleY / smoother;

        pitch = brain->data->headPitch + deltaPitch;
        yaw = brain->data->headYaw - deltaYaw;
    }

    brain->client->moveHead(pitch, yaw);
    return NodeStatus::SUCCESS;
}

CamFindBall::CamFindBall(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain)
{
    double lowPitch = 0.8;
    double highPitch = 0.3;
    double leftYaw = 0.55;
    double rightYaw = -0.55;

    _cmdSequence[0][0] = lowPitch;
    _cmdSequence[0][1] = leftYaw;
    _cmdSequence[1][0] = lowPitch;
    _cmdSequence[1][1] = 0;
    _cmdSequence[2][0] = lowPitch;
    _cmdSequence[2][1] = rightYaw;
    _cmdSequence[3][0] = highPitch;
    _cmdSequence[3][1] = rightYaw;
    _cmdSequence[4][0] = highPitch;
    _cmdSequence[4][1] = 0;
    _cmdSequence[5][0] = highPitch;
    _cmdSequence[5][1] = leftYaw;

    _cmdIndex = 0;
    _cmdIntervalMSec = 800;
    _cmdRestartIntervalMSec = 50000;
    _timeLastCmd = brain->get_clock()->now();
}

NodeStatus CamFindBall::tick()
{
    if (brain->data->ballDetected)
    {
        return NodeStatus::SUCCESS;
    }

    auto curTime = brain->get_clock()->now();
    auto timeSinceLastCmd = (curTime - _timeLastCmd).nanoseconds() / 1e6;
    if (timeSinceLastCmd < _cmdIntervalMSec)
    {
        return NodeStatus::SUCCESS;
    }
    else if (timeSinceLastCmd > _cmdRestartIntervalMSec)
    {
        _cmdIndex = 0;
    }
    else
    {
        _cmdIndex = (_cmdIndex + 1) % (sizeof(_cmdSequence) / sizeof(_cmdSequence[0]));
    }

    brain->client->moveHead(_cmdSequence[_cmdIndex][0], _cmdSequence[_cmdIndex][1]);
    _timeLastCmd = brain->get_clock()->now();
    return NodeStatus::SUCCESS;
}

NodeStatus CamScanField::tick()
{
    auto sec = brain->get_clock()->now().seconds();
    auto msec = static_cast<unsigned long long>(sec * 1000);
    double lowPitch, highPitch, leftYaw, rightYaw;
    getInput("low_pitch", lowPitch);
    getInput("high_pitch", highPitch);
    getInput("left_yaw", leftYaw);
    getInput("right_yaw", rightYaw);
    int msecCycle;
    getInput("msec_cycle", msecCycle);

    int cycleTime = msec % msecCycle;
    double pitch = cycleTime > (msecCycle / 2.0) ? lowPitch : highPitch;
    double yaw = cycleTime < (msecCycle / 2.0) ? (leftYaw - rightYaw) * (2.0 * cycleTime / msecCycle) + rightYaw : (leftYaw - rightYaw) * (2.0 * (msecCycle - cycleTime) / msecCycle) + rightYaw;

    brain->client->moveHead(pitch, yaw);
    return NodeStatus::SUCCESS;
}

NodeStatus Chase::tick()
{
    if (!brain->tree->getEntry<bool>("ball_location_known"))
    {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }
    double vxLimit, vyLimit, vthetaLimit, dist;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);
    getInput("dist", dist);

    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;

    Pose2D target_f, target_r;
    if (brain->data->robotPoseToField.x - brain->data->ball.posToField.x > (_state == "chase" ? 1.0 : 0.0))
    {
        _state = "circle_back";

        target_f.x = brain->data->ball.posToField.x - dist;

        if (brain->data->robotPoseToField.y > brain->data->ball.posToField.y - _dir)
            _dir = 1.0;
        else
            _dir = -1.0;

        target_f.y = brain->data->ball.posToField.y + _dir * dist;
    }
    else
    { // chase
        _state = "chase";
        target_f.x = brain->data->ball.posToField.x - dist;
        target_f.y = brain->data->ball.posToField.y;
    }

    target_r = brain->data->field2robot(target_f);

    double vx = target_r.x;
    double vy = target_r.y;
    double vtheta = ballYaw * 2.0;

    double linearFactor = 1 / (1 + exp(3 * (ballRange * fabs(ballYaw)) - 3));
    vx *= linearFactor;
    vy *= linearFactor;

    vx = cap(vx, vxLimit, -vxLimit);
    vy = cap(vy, vyLimit, -vyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);

    brain->client->setVelocity(vx, vy, vtheta, false, false, false);
    return NodeStatus::SUCCESS;
}

NodeStatus CalcKickDir::tick()
{
    double crossThreshold;
    getInput("cross_threshold", crossThreshold);

    string lastKickType = brain->data->kickType;
    if (lastKickType == "cross") crossThreshold += 0.1;

    auto gpAngles = brain->getGoalPostAngles(0.0);
    auto thetal = gpAngles[0]; auto thetar = gpAngles[1];
    auto bPos = brain->data->ball.posToField;
    auto fd = brain->config->fieldDimensions;
    auto color = 0xFFFFFFFF; // for log

    if (thetal - thetar < crossThreshold && brain->data->ball.posToField.x > fd.circleRadius) {
        brain->data->kickType = "cross";
        color = 0xFF00FFFF;
        brain->data->kickDir = atan2(
            - bPos.y,
            fd.length/2 - fd.penaltyDist/2 - bPos.x
        );
    }
    else if (brain->isDefensing()) {
        brain->data->kickType = "block";
        color = 0xFFFF00FF;
        brain->data->kickDir = atan2(
            bPos.y,
            bPos.x + fd.length/2
        );

    } else { 
        brain->data->kickType = "shoot";
        color = 0x00FF00FF;
        brain->data->kickDir = atan2(
            - bPos.y,
            fd.length/2 - bPos.x
        );
        if (brain->data->ball.posToField.x > brain->config->fieldDimensions.length / 2) brain->data->kickDir = 0; 
    }

    brain->log->setTimeNow();
    brain->log->log(
        "field/kick_dir",
        rerun::Arrows2D::from_vectors({{10 * cos(brain->data->kickDir), -10 * sin(brain->data->kickDir)}})
            .with_origins({{brain->data->ball.posToField.x, -brain->data->ball.posToField.y}})
            .with_colors({color})
            .with_radii(0.01)
            .with_draw_order(31)
    );

    return NodeStatus::SUCCESS;
}


NodeStatus ChaseToTarget::tick()
{
    if (!brain->tree->getEntry<bool>("ball_location_known"))
    {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }
    
    double vxLimit, vyLimit, vthetaLimit, dist, targetX, targetY, turnFactor;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);
    getInput("dist", dist);
    getInput("target_x", targetX);
    getInput("target_y", targetY);
    getInput("turn_factor", turnFactor);
    
    // NEW DRIBBLING PARAMETERS
    bool enableDribbling;
    double dribbleDistance;
    double dribbleKickPower;
    int dribbleKickDuration;
    int dribbleCooldown;
    
    getInput("enable_dribbling", enableDribbling);
    getInput("dribble_distance", dribbleDistance);
    getInput("dribble_kick_power", dribbleKickPower);
    getInput("dribble_kick_duration", dribbleKickDuration);
    getInput("dribble_cooldown", dribbleCooldown);

    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;
    auto currentTime = brain->get_clock()->now();

    // Calculate desired direction from ball to target
    double ballToTargetAngle = atan2(targetY - brain->data->ball.posToField.y, 
                                   targetX - brain->data->ball.posToField.x);
    
    // Calculate current robot-ball angle in field coordinates
    double currentRobotBallAngle = brain->data->robotBallAngleToField;
    
    // Calculate angle difference (how much we need to turn)
    double angleDiff = toPInPI(ballToTargetAngle - currentRobotBallAngle);

    // DRIBBLING LOGIC
    if (enableDribbling) {
        prtDebug(format("ChaseToTarget: Dribbling enabled, state=%s, ballRange=%.3f, dribbleDistance=%.3f", 
                       _state.c_str(), ballRange, dribbleDistance));
        
        // Check if we're currently executing a dribble kick
        if (_state == "dribble_kick") {
            brain->log->logToScreen("ChaseToTarget/main", "Dribble kick finished, entering cooldown", 0xFFFF00FF);
            // Safe time calculation - check if start time is valid
            if (_dribbleKickStartTime.get_clock_type() != RCL_CLOCK_UNINITIALIZED) {
                double kickElapsed = (currentTime - _dribbleKickStartTime).seconds() * 1000.0; // Convert to ms
                prtDebug(format("ChaseToTarget/main: Dribble kick in progress, elapsed=%.0fms, duration=%dms", 
                               kickElapsed, dribbleKickDuration));
                
                if (kickElapsed < dribbleKickDuration) {
                    // Continue the kick
                    double kickVx = vxLimit * dribbleKickPower;
                    double kickVy = 0.0; // Straight forward kick
                    brain->client->setVelocity(kickVx, kickVy, 0, false, false, false);
                    
                    brain->log->logToScreen("ChaseToTarget/main",
                        format("DRIBBLE KICK! Duration: %.0fms", kickElapsed), 0xFF0000FF);
                    prtDebug(format("ChaseToTarget: Executing kick with vx=%.3f, power=%.3f", kickVx, dribbleKickPower));
                    return NodeStatus::SUCCESS;
                } else {
                    // Kick finished, enter cooldown
                    _state = "dribble_cooldown";
                    _lastDribbleKickTime = currentTime;
                    brain->log->logToScreen("ChaseToTarget/main", "Dribble kick finished, entering cooldown", 0xFFFF00FF);
                    prtDebug("ChaseToTarget: Kick finished, entering cooldown");
                }
            } else {
                // Invalid start time, restart kick
                _dribbleKickStartTime = currentTime;
                prtDebug("ChaseToTarget: Invalid kick start time, restarting");
            }
        }
        
        // Check if we're in cooldown period
        if (_state == "dribble_cooldown") {
            // Safe time calculation - check if last kick time is valid
            if (_lastDribbleKickTime.get_clock_type() != RCL_CLOCK_UNINITIALIZED) {
                double cooldownElapsed = (currentTime - _lastDribbleKickTime).seconds() * 1000.0; // Convert to ms
                prtDebug(format("ChaseToTarget: In cooldown, elapsed=%.0fms, cooldown=%dms", 
                               cooldownElapsed, dribbleCooldown));
                
                if (cooldownElapsed < dribbleCooldown) {
                    // Stay in cooldown, continue normal chase behavior but don't kick
                    brain->log->logToScreen("ChaseToTarget/main",
                        format("Dribble cooldown: %.0fms remaining", dribbleCooldown - cooldownElapsed), 0x00FFFF00);
                    // Don't return here - let normal chase logic continue
                } else {
                    // Cooldown finished, return to normal chasing
                    _state = "chase";
                    brain->log->logToScreen("ChaseToTarget/main", "Dribble cooldown finished, resuming chase", 0x00FF00FF);
                    prtDebug("ChaseToTarget: Cooldown finished, resuming chase");
                }
            } else {
                // Invalid last kick time, reset to chase
                _state = "chase";
                prtDebug("ChaseToTarget: Invalid last kick time, resetting to chase");
            }
        }
        
        // Check if we should trigger a dribble kick
        if (_state != "dribble_kick" && _state != "dribble_cooldown" && ballRange <= dribbleDistance) {
            // Check if enough time has passed since last kick
            bool canKick = true;
            if (_lastDribbleKickTime.get_clock_type() != RCL_CLOCK_UNINITIALIZED) {
                double timeSinceLastKick = (currentTime - _lastDribbleKickTime).seconds() * 1000.0; // Convert to ms
                canKick = (timeSinceLastKick >= dribbleCooldown);
                prtDebug(format("ChaseToTarget: Time since last kick=%.0fms, cooldown=%dms, canKick=%d", 
                               timeSinceLastKick, dribbleCooldown, canKick));
            } else {
                prtDebug("ChaseToTarget: No previous kick time, can kick immediately");
            }
            
            if (canKick) {
                // Trigger dribble kick
                _state = "dribble_kick";
                _dribbleKickStartTime = currentTime;
                _isDribbleKicking = true;
                
                brain->log->logToScreen("ChaseToTarget/main", 
                    format("TRIGGERING DRIBBLE KICK! Ball range: %.2f", ballRange), 0xFF0000FF);
                prtDebug(format("ChaseToTarget: Triggering dribble kick, ballRange=%.3f, state=%s", 
                               ballRange, _state.c_str()));
                return NodeStatus::SUCCESS;
            } else {
                prtDebug("ChaseToTarget: Cannot kick yet, still in cooldown period");
            }
        }
        
        // If dribbling is enabled, use smaller distance for closer approach
        dist = min(dist, dribbleDistance - 0.05); // Stay slightly closer than kick trigger distance to ensure dribbling activates
    }

    // NORMAL CHASE LOGIC (modified for dribbling compatibility)
    Pose2D target_f, target_r;
    
    // Same logic as original Chase for determining when to circle back
    if (brain->data->robotPoseToField.x - brain->data->ball.posToField.x > (_state == "chase" ? 1.0 : 0.0))
    {
        if (_state != "dribble_kick" && _state != "dribble_cooldown") {
            _state = "circle_back";
        }

        target_f.x = brain->data->ball.posToField.x - dist;

        // Modify circling direction to favor target direction
        double targetInfluencedDir = _dir;
        if (fabs(angleDiff) > 0.2) {
            targetInfluencedDir = (angleDiff > 0) ? 1.0 : -1.0;
            _dir = _dir * 0.7 + targetInfluencedDir * 0.3;
        }

        target_f.y = brain->data->ball.posToField.y + _dir * dist;
    }
    else
    { // chase
        if (_state != "dribble_kick" && _state != "dribble_cooldown") {
            _state = "chase";
        }
        target_f.x = brain->data->ball.posToField.x - dist;
        
        // Gradually adjust chase position towards target direction
        double maxLateralOffset = dist * 0.8;
        double lateralOffset = sin(angleDiff) * maxLateralOffset * turnFactor;
        
        lateralOffset = cap(lateralOffset, maxLateralOffset, -maxLateralOffset);
        
        target_f.y = brain->data->ball.posToField.y + lateralOffset;
    }

    target_r = brain->data->field2robot(target_f);

    double vx = target_r.x;
    double vy = target_r.y;
    
    // Modify angular velocity to gradually turn towards target
    double baseVtheta = ballYaw * 2.0;
    double targetTurnInfluence = angleDiff * turnFactor * 0.3;
    double vtheta = baseVtheta + targetTurnInfluence;

    double linearFactor = (1 / (1 + exp(3 * ((ballRange * fabs(ballYaw)) - 0.6) - 3))) * 1.5 + 0.1;
    vx *= linearFactor;
    vy *= linearFactor;

    vx = cap(vx, vxLimit, -vxLimit);
    vy = cap(vy, vyLimit, -vyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);

    brain->client->setVelocity(vx, vy, vtheta, false, false, false);
    
    // Enhanced logging for dribbling
    string stateInfo = enableDribbling ? format("State: %s, Dribbling: ON", _state.c_str()) : 
                                       format("State: %s, Dribbling: OFF", _state.c_str());
    brain->log->logToScreen("ChaseToTarget/tmp",
                           format("%s, Range: %.2f, AngleDiff: %.2f°", 
                                  stateInfo.c_str(), ballRange, rad2deg(angleDiff)),
                           0x00FF00FF);
    
    // Comprehensive debug logging
    prtDebug(format("ChaseToTarget: Final ` state=%s, ballRange=%.3f, dist=%.3f, targetPos=(%.2f,%.2f)", 
                   _state.c_str(), ballRange, dist, targetX, targetY));
    prtDebug(format("ChaseToTarget: velocity=(%.3f,%.3f,%.3f), angleDiff=%.2f°, enableDribbling=%d", 
                   vx, vy, vtheta, rad2deg(angleDiff), enableDribbling));
    if (enableDribbling) {
        prtDebug(format("ChaseToTarget: dribbleDistance=%.3f, kickPower=%.2f, kickDuration=%dms, cooldown=%dms", 
                       dribbleDistance, dribbleKickPower, dribbleKickDuration, dribbleCooldown));
    }
    
    return NodeStatus::SUCCESS;
}

NodeStatus SimpleChase::tick()
{
    double stopDist, stopAngle, vyLimit, vxLimit;
    getInput("stop_dist", stopDist);
    getInput("stop_angle", stopAngle);
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);

    if (!brain->tree->getEntry<bool>("ball_location_known"))
    {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    double vx = brain->data->ball.posToRobot.x;
    double vy = brain->data->ball.posToRobot.y;
    double vtheta = brain->data->ball.yawToRobot * 2.0;

    double linearFactor = 1 / (1 + exp(3 * (brain->data->ball.range * fabs(brain->data->ball.yawToRobot)) - 3));
    vx *= linearFactor;
    vy *= linearFactor;

    vx = cap(vx, vxLimit, -0.1);
    vy = cap(vy, vyLimit, -vyLimit);

    if (brain->data->ball.range < stopDist)
    {
        vx = 0;
        vy = 0;
    }

    brain->client->setVelocity(vx, vy, vtheta, false, false, false);
    return NodeStatus::SUCCESS;
}

NodeStatus Adjust::tick()
{
    if (!brain->tree->getEntry<bool>("ball_location_known"))
    {
        return NodeStatus::SUCCESS;
    }

    double turnThreshold, vxLimit, vyLimit, vthetaLimit, maxRange, minRange;
    getInput("turn_threshold", turnThreshold);
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);
    getInput("max_range", maxRange);
    getInput("min_range", minRange);
    string position;
    getInput("position", position);

    double vx = 0, vy = 0, vtheta = 0;
    double kickDir = brain->data->kickDir;
    double dir_rb_f = brain->data->robotBallAngleToField;
    double deltaDir = toPInPI(kickDir - dir_rb_f);
    double dir = deltaDir > 0 ? -1.0 : 1.0;
    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;

    // Calculate speed scaling factor based on angle difference
    double angleDiff = fabs(deltaDir);
    double speedScale = 0.4;


    std::cout << "[DEBUG] speedScale: " << speedScale << ", angleDiff: " << angleDiff << std::endl;

    double s = speedScale;  // Apply speed scaling to base movement speed
    double r = 0.8;

    // Base circling movement
    vx = -s * dir * sin(ballYaw);
    vy = s * dir * cos(ballYaw);

    std::cout << "[DEBUG] vx: " << vx << ", vy: " << vy << std::endl;

    // Maintain distance to maxRange
    if (ballRange > maxRange)
        vx += 0.1;
    if (ballRange < maxRange)
        vx -= 0.1;

    // Continuous turning control
    vtheta = (ballYaw - dir * s) / r;

    vx = cap(vx, vxLimit, -vxLimit);
    vy = cap(vy, vyLimit, -vyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);

    brain->client->setVelocity(vx, vy, vtheta);
    return NodeStatus::SUCCESS;
}

NodeStatus TangentialAdjust::tick()
{
    if (!brain->tree->getEntry<bool>("ball_location_known"))
    {
        return NodeStatus::SUCCESS;
    }

    double turnThreshold, vxLimit, vyLimit, vthetaLimit, maxRange, minRange;
    getInput("turn_threshold", turnThreshold);
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);
    getInput("max_range", maxRange);
    getInput("min_range", minRange);
    string position;
    getInput("position", position);

    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;

    // Calculate desired kick direction
    double kickDir = (position == "defense") ? 
        atan2(brain->data->ball.posToField.y, brain->data->ball.posToField.x + brain->config->fieldDimensions.length / 2) : 
        atan2(-brain->data->ball.posToField.y, brain->config->fieldDimensions.length / 2 - brain->data->ball.posToField.x);
    
    // Current robot-ball angle in field coordinates
    double dir_rb_f = brain->data->robotBallAngleToField;
    double deltaDir = toPInPI(kickDir - dir_rb_f);
    
    // Calculate two possible tangent directions
    double tangent1 = ballYaw + M_PI / 2.0;  // Counterclockwise tangent
    double tangent2 = ballYaw - M_PI / 2.0;  // Clockwise tangent
    
    // Choose the tangent direction that reduces angle difference to target
    double diff1 = fabs(toPInPI(tangent1 - kickDir));
    double diff2 = fabs(toPInPI(tangent2 - kickDir));
    
    double tangentAngle = (diff1 < diff2) ? tangent1 : tangent2;
    
    // Tangential speed
    double s = 0.6;  // Base tangential speed
    double vx = s * cos(tangentAngle);
    double vy = s * sin(tangentAngle);
    
    // Distance control: maintain desired range to ball
    if (ballRange > maxRange) {
        vy += 0.1;  // Move closer to ball
    } else if (ballRange < maxRange) {
        vy -= 0.1;  // Move away from ball
    }
    
    // Rotation control: ensure robot faces tangent direction
    double currentTheta = brain->data->robotPoseToField.theta;
    double targetTheta = currentTheta + tangentAngle;
    double rotationNeeded = toPInPI(targetTheta - currentTheta);
    
    double vtheta = rotationNeeded * 2.0;
    
    // Apply velocity limits
    vx = cap(vx, vxLimit, -vxLimit);
    vy = cap(vy, vyLimit, -vyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);

    brain->client->setVelocity(vx, vy, vtheta);
    
    // Debug information
    double angleDiff = fabs(deltaDir);
    brain->log->logToScreen("TangentialAdjust",
        format("AngleDiff: %.2f°, Tangent: %.2f°, Speed: (%.2f,%.2f,%.2f)", 
               rad2deg(angleDiff), rad2deg(tangentAngle), vx, vy, vtheta),
        0x00FFFF00);
    
    return NodeStatus::SUCCESS;
}

NodeStatus StrikerDecide::tick()
{

    double chaseRangeThreshold;
    getInput("chase_threshold", chaseRangeThreshold);
    double kickRangeThreshold;
    getInput("kick_range_threshold", kickRangeThreshold);
    double dribbleRangeThreshold;
    getInput("dribble_range_threshold", dribbleRangeThreshold);
    string lastDecision, position;
    getInput("decision_in", lastDecision);
    getInput("position", position);

    double kickDir = brain->data->kickDir;;
    double dir_rb_f = brain->data->robotBallAngleToField;

    auto goalPostAngles = brain->getGoalPostAngles(0.5);
    double theta_l = goalPostAngles[0];
    double theta_r = goalPostAngles[1];
    bool angleIsGood = (theta_l > dir_rb_f && theta_r < dir_rb_f);

    auto goalPostAnglesDribble = brain->getGoalPostAngles(1.5);
    double theta_l_dribble = goalPostAnglesDribble[0];
    double theta_r_dribble = goalPostAnglesDribble[1];
    bool angleIsGoodDribble = (theta_l_dribble > dir_rb_f && theta_r_dribble < dir_rb_f);

    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;

    // Calculate angle difference between current direction and desired kick direction
    double angleDiff = fabs(toPInPI(kickDir - dir_rb_f));
    
    // Add range thresholds
    bool ballInKickRange = (ballRange <= kickRangeThreshold);
    bool ballInDribbleRange = (ballRange <= dribbleRangeThreshold);
    bool angleInDribbleRange = (angleDiff <= 0.3); // Use dribble_range_threshold as angle threshold too

    string newDecision;
    auto color = 0xFFFFFFFF; // for log
    if (!brain->tree->getEntry<bool>("ball_location_known"))
    {
        newDecision = "find";
        color = 0x0000FFFF;
    }
    else if (ballRange > chaseRangeThreshold * (lastDecision == "chase" ? 0.9 : 1.0))
    {
        newDecision = "chase";
        color = 0x00FF00FF;
    }
    else if (angleInDribbleRange)
    {
        newDecision = "chasetotarget";
        color = 0xFFFF00FF; // Yellow for dribble
    }
    else if (angleIsGood)
    {
        newDecision = "kick";
        color = 0xFF0000FF;
    }
    else
    {
        newDecision = "adjust";
        color = 0x00FFFFFF;
    }

    setOutput("decision_out", newDecision);
    brain->log->logToScreen("tree/Decide",
                            format("Decision: %s ballrange: %.2f ballyaw: %.2f kickDir: %.2f rbDir: %.2f angleIsGood: %d ballInKickRange: %d angleDiff: %.2f angleInDribbleRange: %d", newDecision.c_str(), ballRange, ballYaw, kickDir, dir_rb_f, angleIsGood, ballInKickRange, angleDiff, angleInDribbleRange),
                            color);
    return NodeStatus::SUCCESS;
}

NodeStatus CheckAndStandUp::tick()
{
    if (brain->tree->getEntry<bool>("gc_is_under_penalty") || brain->data->currentRobotModeIndex == 1) {
        brain->data->needManualRelocate = false;
        brain->tree->setEntry<bool>("should_recalibrate_after_fall_recovery", false);
        brain->data->recoveryPerformed = false;
        brain->data->enterDampingPerformed = false;
        brain->log->log("recovery", rerun::TextLog("reset recovery"));
        return NodeStatus::SUCCESS;
    }

    if (brain->data->needManualRelocate)
    {
        brain->log->log("recovery", rerun::TextLog("need manual relocate"));
        return NodeStatus::FAILURE;
    }

    if (brain->data->recoveryState == RobotRecoveryState::HAS_FALLEN &&
        // brain->data->isRecoveryAvailable && // If fallen, directly try RL stand up (no need to care about recoveryAvailable)
        brain->data->currentRobotModeIndex != 1 && // not in prepare
        !brain->data->recoveryPerformed &&
        !brain->data->enterDampingPerformed) {
        brain->client->standUp();
        brain->data->recoveryPerformed = true;
        brain->data->lastRecoveryTime = brain->get_clock()->now();
        brain->log->log("recovery", rerun::TextLog("Fall detect and stand up"));
    }

    // If not stood up and 5 seconds have passed, enter damping mode, only once
    auto now = brain->get_clock()->now();
    auto seconds_elaps = now.seconds() - brain->data->lastRecoveryTime.seconds();
    if (brain->data->recoveryPerformed &&
        !brain->data->enterDampingPerformed &&
        seconds_elaps >10 &&
        brain->data->recoveryState != RobotRecoveryState::IS_READY) {

        brain->client->enterDamping();
        brain->data->enterDampingPerformed = true;
        brain->tree->setEntry<bool>("should_recalibrate_after_fall_recovery", false);
        brain->log->log("recovery", rerun::TextLog("Enter Damping, seconds_elaps: " + to_string(seconds_elaps) +
        "recoveryState: " + to_string(static_cast<int>(brain->data->recoveryState))));

        // std::cout << "Enter Damping, seconds_elaps: " << seconds_elaps << " recoveryState: " << static_cast<int>(brain->data->recoveryState) << std::endl;
    }

    if (brain->data->recoveryPerformed &&
        !brain->data->enterDampingPerformed &&
        brain->data->recoveryState == RobotRecoveryState::IS_READY) {
        brain->tree->setEntry<bool>("should_recalibrate_after_fall_recovery", true);
        brain->log->log("recovery", rerun::TextLog("Standup success, seconds_elaps: " + to_string(seconds_elaps) +
        "recoveryState: " + to_string(static_cast<int>(brain->data->recoveryState))));
    }

    // Robot is standing and in robocup gait, can reset fall recovery state
    if (brain->data->recoveryState == RobotRecoveryState::IS_READY &&
        brain->data->currentRobotModeIndex == 8) { // in robocup gait
        brain->data->recoveryPerformed = false;
        brain->data->enterDampingPerformed = false;
        brain->log->log("recovery", rerun::TextLog("Reset recovery, recoveryState: " + to_string(static_cast<int>(brain->data->recoveryState))));
    }

    return NodeStatus::SUCCESS;
}

NodeStatus RotateForRelocate::onStart()
{
    this->_lastSuccessfulLocalizeTime = brain->data->lastSuccessfulLocalizeTime;
    this->_startTime = brain->get_clock()->now();
    return NodeStatus::RUNNING;
}

NodeStatus RotateForRelocate::onRunning()
{
    double vyaw_limit;
    getInput("vyaw_limit", vyaw_limit);
    int max_msec_locate;
    getInput("max_msec_locate", max_msec_locate);

    brain->client->moveHead(0.4, 0.0);
    brain->client->setVelocity(0, 0, vyaw_limit);

    if (this->_lastSuccessfulLocalizeTime.nanoseconds() != brain->data->lastSuccessfulLocalizeTime.nanoseconds()) {
        brain->tree->setEntry<bool>("should_recalibrate_after_fall_recovery", false);
        brain->log->log("recovery", rerun::TextLog("Relocated successfully"));
        return NodeStatus::SUCCESS;
    }

    if (brain->msecsSince(this->_startTime) > max_msec_locate) {
        brain->tree->setEntry<bool>("should_recalibrate_after_fall_recovery", false);
        brain->data->needManualRelocate = true;
        brain->client->enterDamping();
        brain->log->log("recovery", rerun::TextLog("Relocated failed for timeout"));
        return NodeStatus::SUCCESS;
    }

    return NodeStatus::RUNNING;
}

void RotateForRelocate::onHalted()
{
    brain->tree->setEntry<bool>("should_recalibrate_after_fall_recovery", false);
}


NodeStatus GoalieDecide::tick()
{

    double chaseRangeThreshold;
    getInput("chase_threshold", chaseRangeThreshold);
    double kickRangeThreshold;
    getInput("kick_range_threshold", kickRangeThreshold);
    double dribbleRangeThreshold;
    getInput("dribble_range_threshold", dribbleRangeThreshold);
    string lastDecision, position;
    getInput("decision_in", lastDecision);

    double kickDir = atan2(brain->data->ball.posToField.y, brain->data->ball.posToField.x + brain->config->fieldDimensions.length / 2);
    double dir_rb_f = brain->data->robotBallAngleToField;
    auto goalPostAngles = brain->getGoalPostAngles(0.3);
    double theta_l = goalPostAngles[0];
    double theta_r = goalPostAngles[1];
    bool angleIsGood = (dir_rb_f > -M_PI / 2 && dir_rb_f < M_PI / 2);
    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;

    // Calculate angle difference between current direction and desired kick direction
    double angleDiff = fabs(toPInPI(kickDir - dir_rb_f));
    
    // Add range thresholds
    bool ballInKickRange = (ballRange <= kickRangeThreshold);
    bool ballInDribbleRange = (ballRange <= dribbleRangeThreshold);
    bool angleInDribbleRange = (angleDiff <= 0.3); // Use dribble_range_threshold as angle threshold too

    string newDecision;
    auto color = 0xFFFFFFFF; // for log
    if (!brain->tree->getEntry<bool>("ball_location_known"))
    {
        newDecision = "find";
        color = 0x0000FFFF;
    }
    else if (brain->data->ball.posToField.x > 0 - static_cast<double>(lastDecision == "retreat"))
    {
        newDecision = "retreat";
        color = 0xFF00FFFF;
    }
    else if (ballRange > chaseRangeThreshold * (lastDecision == "chase" ? 0.9 : 1.0))
    {
        newDecision = "chase";
        color = 0x00FF00FF;
    }
    else if (angleInDribbleRange)
    {
        newDecision = "chasetotarget";
        color = 0xFFFF00FF; // Yellow for dribble
    }
    else if (angleIsGood)
    {
        newDecision = "kick";
        color = 0xFF0000FF;
    }
    else
    {
        newDecision = "adjust";
        color = 0x00FFFFFF;
    }

    setOutput("decision_out", newDecision);
    brain->log->logToScreen("tree/Decide",
                            format("Decision: %s ballrange: %.2f ballyaw: %.2f kickDir: %.2f rbDir: %.2f angleIsGood: %d ballInKickRange: %d angleDiff: %.2f angleInDribbleRange: %d", newDecision.c_str(), ballRange, ballYaw, kickDir, dir_rb_f, angleIsGood, ballInKickRange, angleDiff, angleInDribbleRange),
                            color);
    return NodeStatus::SUCCESS;
}

NodeStatus Kick::onStart()
{
    _startTime = brain->get_clock()->now();

    double vxLimit, vyLimit;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    int minMSecKick;
    getInput("min_msec_kick", minMSecKick);
    double vxFactor = brain->config->vxFactor;
    double yawOffset = brain->config->yawOffset;

    double adjustedYaw = brain->data->ball.yawToRobot - yawOffset;
    double tx = cos(adjustedYaw) * brain->data->ball.range;
    double ty = sin(adjustedYaw) * brain->data->ball.range;

    double vx, vy;

    if (fabs(ty) < 0.01 && fabs(adjustedYaw) < 0.01)
    {
        vx = vxLimit;
        vy = 0.0;
    }
    else
    {
        vy = ty > 0 ? vyLimit : -vyLimit;
        vx = vy / ty * tx * vxFactor;
        if (fabs(vx) > vxLimit)
        {
            vy *= vxLimit / vx;
            vx = vxLimit;
        }
    }

    double speed = norm(vx, vy);

    _msecKick = speed > 1e-5 ? minMSecKick + static_cast<int>(brain->data->ball.range / speed * 1000) : minMSecKick;

    brain->client->setVelocity(vx, vy, 0, false, false, false);
    return NodeStatus::RUNNING;
}

NodeStatus Kick::onRunning()
{
    if (brain->msecsSince(_startTime) < _msecKick)
        return NodeStatus::RUNNING;

    brain->client->setVelocity(0, 0, 0);
    return NodeStatus::SUCCESS;
}

void Kick::onHalted()
{
    _startTime -= rclcpp::Duration(100, 0);
}

NodeStatus RobotFindBall::onStart()
{
    if (brain->data->ballDetected)
    {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }
    _turnDir = brain->data->ball.yawToRobot > 0 ? 1.0 : -1.0;

    return NodeStatus::RUNNING;
}

NodeStatus RobotFindBall::onRunning()
{
    if (brain->data->ballDetected)
    {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    double vyawLimit;
    getInput("vyaw_limit", vyawLimit);

    double vx = 0;
    double vy = 0;
    double vtheta = 0;
    brain->client->setVelocity(0, 0, vyawLimit * _turnDir);
    return NodeStatus::RUNNING;
}

void RobotFindBall::onHalted()
{
    _turnDir = 1.0;
}

NodeStatus SelfLocate::tick()
{
    string mode = getInput<string>("mode").value();
    double xMin = 0.0, xMax = 0.0, yMin = 0, yMax = 0.0, thetaMin = 0.0, thetaMax = 0.0;
    auto markers = brain->data->getMarkers();

    if (mode == "enter_field")
    {

        xMin = -brain->config->fieldDimensions.length / 2;
        xMax = -brain->config->fieldDimensions.circleRadius;

        if (brain->config->playerStartPos == "left")
        {
            yMin = brain->config->fieldDimensions.width / 2;
            yMax = brain->config->fieldDimensions.width / 2 + 1.0;
        }
        else if (brain->config->playerStartPos == "right")
        {
            yMin = -brain->config->fieldDimensions.width / 2 - 1.0;
            yMax = -brain->config->fieldDimensions.width / 2;
        }

        if (brain->config->playerStartPos == "left")
        {
            thetaMin = -M_PI / 2 - M_PI / 6;
            thetaMax = -M_PI / 2 + M_PI / 6;
        }
        else if (brain->config->playerStartPos == "right")
        {
            thetaMin = M_PI / 2 - M_PI / 6;
            thetaMax = M_PI / 2 + M_PI / 6;
        }
    }
    else if (mode == "face_forward")
    {
        xMin = -brain->config->fieldDimensions.length / 2;
        xMax = brain->config->fieldDimensions.length / 2;
        yMin = -brain->config->fieldDimensions.width / 2;
        yMax = brain->config->fieldDimensions.width / 2;
        thetaMin = -M_PI / 4;
        thetaMax = M_PI / 4;
    }
    else if (mode == "trust_direction")
    {
        int msec = static_cast<int>(brain->msecsSince(brain->data->lastSuccessfulLocalizeTime));
        double maxDriftSpeed = 0.1;
        double maxDrift = msec / 1000.0 * maxDriftSpeed;

        xMin = max(-brain->config->fieldDimensions.length / 2, brain->data->robotPoseToField.x - maxDrift);
        xMax = min(brain->config->fieldDimensions.length / 2, brain->data->robotPoseToField.x + maxDrift);
        yMin = max(-brain->config->fieldDimensions.width / 2, brain->data->robotPoseToField.y - maxDrift);
        yMax = min(brain->config->fieldDimensions.width / 2, brain->data->robotPoseToField.y + maxDrift);
        thetaMin = brain->data->robotPoseToField.theta - M_PI / 18;
        thetaMax = brain->data->robotPoseToField.theta + M_PI / 18;
    }
    else if (mode == "fall_recovery")
    {
        int msec = static_cast<int>(brain->msecsSince(brain->data->lastSuccessfulLocalizeTime));
        double maxDriftSpeed = 0.1;                      // m/s
        double maxDrift = msec / 1000.0 * maxDriftSpeed; // Maximum drift distance of odom in this time period

        xMin = -brain->config->fieldDimensions.length / 2 - 2;
        xMax = brain->config->fieldDimensions.length / 2 + 2;
        yMin = -brain->config->fieldDimensions.width / 2 - 2;
        yMax = brain->config->fieldDimensions.width / 2 + 2;
        thetaMin = brain->data->robotPoseToField.theta - M_PI / 180;
        thetaMax = brain->data->robotPoseToField.theta + M_PI / 180;
    }
    else if (mode == "penalty_point_localize")
    {
        // Add call interval limit to prevent frequent calls causing localization to opposite half field
        static rclcpp::Time lastPenaltyLocalizeTime = rclcpp::Time(0LL, RCL_CLOCK_UNINITIALIZED);
        auto currentTime = brain->get_clock()->now();

        // 如果是第一次调用，直接初始化时间
        if (lastPenaltyLocalizeTime.get_clock_type() == RCL_CLOCK_UNINITIALIZED) {
            lastPenaltyLocalizeTime = currentTime;
        }

        auto elapsed = (currentTime - lastPenaltyLocalizeTime).seconds();

        if (elapsed < 3.0) { // 3秒内不允许重复调用
            //prtDebug("penalty_point_localize 调用过于频繁，距离上次调用仅 " + to_string(elapsed) + " 秒，需要等待 3 秒间隔");
            return NodeStatus::SUCCESS; // 直接返回成功，不执行定位
        }

        // Check if we can see a penalty point marker within 5 meters
        FieldMarker penaltyMarker;
        bool foundPenaltyPoint = false;

        // Find a penalty point marker within 5 meters
        for (const auto& marker : markers) {
            if (marker.type == 'P') {
                double distance = sqrt(marker.x * marker.x + marker.y * marker.y);
                brain->log->log("locator/penalty_point", rerun::TextLog("Found penalty point marker, Distance: " + to_string(distance) + " 米"));
                if (distance <= 5.0) {
                    penaltyMarker = marker;
                    foundPenaltyPoint = true;
                    brain->log->log("locator/penalty_point", rerun::TextLog("penalty point marker is within 5 meters: (" + to_string(marker.x) + ", " + to_string(marker.y) + ")"));
                }
            }
        }

        if (foundPenaltyPoint) {
            // 更新最后调用时间
            lastPenaltyLocalizeTime = currentTime;

            // Calculate robot position based on the penalty point
            auto fd = brain->config->fieldDimensions;

            // Penalty point positions in field coordinates:
            // Right penalty point: (fd.length / 2 - fd.penaltyDist, 0.0)
            // Left penalty point: (-fd.length / 2 + fd.penaltyDist, 0.0)
            double rightPenaltyX = fd.length / 2 - fd.penaltyDist;
            double leftPenaltyX = -fd.length / 2 + fd.penaltyDist;

            // Determine which penalty point we're seeing based on the observed penalty point position
            // Use the robot's current orientation and penalty point relative position to determine which side
            double currentTheta = brain->data->robotPoseToField.theta;
            double observedX = penaltyMarker.x;
            double observedY = penaltyMarker.y;

            // Transform the observed penalty point to field coordinates using rough position estimate
            double roughFieldX = brain->data->robotPoseToField.x + (cos(currentTheta) * observedX - sin(currentTheta) * observedY);

            // Determine if it's right or left penalty point based on the rough field position
            bool isRightPenalty = (roughFieldX > 0);

            // Get the actual penalty point position in field coordinates
            double penaltyFieldX = isRightPenalty ? rightPenaltyX : leftPenaltyX;
            double penaltyFieldY = 0.0;

            // Calculate the observed penalty point position in field coordinates for validation
            double observedPenaltyFieldX = brain->data->robotPoseToField.x + (cos(currentTheta) * observedX - sin(currentTheta) * observedY);
            double observedPenaltyFieldY = brain->data->robotPoseToField.y + (sin(currentTheta) * observedX + cos(currentTheta) * observedY);

            // Validate if the observed penalty point is actually near the expected penalty point positions
            double distanceToRightPenalty = sqrt(pow(observedPenaltyFieldX - rightPenaltyX, 2) + pow(observedPenaltyFieldY - 0.0, 2));
            double distanceToLeftPenalty = sqrt(pow(observedPenaltyFieldX - leftPenaltyX, 2) + pow(observedPenaltyFieldY - 0.0, 2));

            // If the observed penalty point is too far from both actual penalty points, it's likely a misidentification
            double validationThreshold = 3; // 1.5 meters tolerance
            if (distanceToRightPenalty > validationThreshold && distanceToLeftPenalty > validationThreshold) {
                brain->log->log("locator/penalty_point", rerun::TextLog("Observed penalty point is too far from actual penalty points. Distance to right: " +
                    to_string(distanceToRightPenalty) + "m, Distance to left: " + to_string(distanceToLeftPenalty) + "m. Likely misidentification, skipping localization."));
                prtDebug("penalty point validation failed: observed at (" + to_string(observedPenaltyFieldX) + ", " + to_string(observedPenaltyFieldY) +
                         ") is too far from actual penalty points");
                return NodeStatus::SUCCESS;
            }

            // Calculate robot position: penalty_field = robot_pose + R * penalty_robot
            // where R is rotation matrix and penalty_robot is the observed marker position
            double distance = sqrt(observedX * observedX + observedY * observedY);

            // Robot position = penalty_field - R * penalty_robot
            double robotX = penaltyFieldX - (cos(currentTheta) * observedX - sin(currentTheta) * observedY);
            double robotY = penaltyFieldY - (sin(currentTheta) * observedX + cos(currentTheta) * observedY);

            // Use current theta with small adjustment tolerance
            double robotTheta = currentTheta;

            brain->log->log("locator/penalty_point", rerun::TextLog("Robot Pose: (" + to_string(robotX) + ", " + to_string(robotY) + ", " + to_string(rad2deg(robotTheta)) + "°)"));

            // Direct localization without using particle filter
            brain->calibrateOdom(robotX, robotY, robotTheta);
            brain->tree->setEntry<bool>("odom_calibrated", true);
            brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();


            brain->log->log("locator/penalty_point", rerun::TextLog("Succes Locate! use" + string(isRightPenalty ? "Right" : "Left") + "Penalty Point"));
            prtDebug("penalty point localize success: " + to_string(robotX) + " " + to_string(robotY) + " " + to_string(rad2deg(robotTheta)) +
                     " penalty: " + (isRightPenalty ? "right" : "left"));

        }
        return NodeStatus::SUCCESS;
    }


    // TODO other modes

    // Locate
    PoseBox2D constraints{xMin, xMax, yMin, yMax, thetaMin, thetaMax};
    double residual;
    auto res = brain->locator->locateRobot(markers, constraints);

    // if (brain->config->rerunLogEnable) {
    if (false)
    {
        brain->log->setTimeNow();
        brain->log->log("locator/time",
                        rerun::Scalar(res.msecs));
        brain->log->log("locator/residual",
                        rerun::Scalar(res.residual));
        brain->log->log("locator/result",
                        rerun::Scalar(res.code));
        brain->log->log("locator/constraints",
                        rerun::TextLog(
                            "xMin: " + to_string(xMin) + " " +
                            "xMax: " + to_string(xMax) + " " +
                            "yMin: " + to_string(yMin) + " " +
                            "yMax: " + to_string(yMax) + " " +
                            "thetaMin: " + to_string(thetaMin) + " " +
                            "thetaMax: " + to_string(thetaMax)));
    }
    prtDebug("locate result: res: " + to_string(res.code) + " time: " + to_string(res.msecs));

    if (!res.success)
        return NodeStatus::SUCCESS; // Do not block following nodes.

    brain->calibrateOdom(res.pose.x, res.pose.y, res.pose.theta);
    brain->tree->setEntry<bool>("odom_calibrated", true);
    brain->data->lastSuccessfulLocalizeTime = brain->get_clock()->now();
    prtDebug("locate success: " + to_string(res.pose.x) + " " + to_string(res.pose.y) + " " + to_string(rad2deg(res.pose.theta)) + " Dur: " + to_string(res.msecs));

    return NodeStatus::SUCCESS;
}

NodeStatus MoveToPoseOnField::tick()
{
    double tx, ty, ttheta, longRangeThreshold, turnThreshold, vxLimit, vyLimit, vthetaLimit, xTolerance, yTolerance, thetaTolerance;
    getInput("x", tx);
    getInput("y", ty);
    getInput("theta", ttheta);
    getInput("long_range_threshold", longRangeThreshold);
    getInput("turn_threshold", turnThreshold);
    getInput("vx_limit", vxLimit);
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);
    getInput("x_tolerance", xTolerance);
    getInput("y_tolerance", yTolerance);
    getInput("theta_tolerance", thetaTolerance);

    brain->client->moveToPoseOnField(tx, ty, ttheta, longRangeThreshold, turnThreshold, vxLimit, vyLimit, vthetaLimit, xTolerance, yTolerance, thetaTolerance);
    return NodeStatus::SUCCESS;
}

NodeStatus WaveHand::tick()
{
    string action;
    getInput("action", action);
    if (action == "start")
        brain->client->waveHand(true);
    else
        brain->client->waveHand(false);
    return NodeStatus::SUCCESS;
}

NodeStatus GoBackInField::onStart()
{
    double valve;
    getInput("valve", valve);
    
    auto fd = brain->config->fieldDimensions;
    double currentX = brain->data->robotPoseToField.x;
    double currentY = brain->data->robotPoseToField.y;
    
    // Check if robot is outside the field
    bool isOutside = false;
    if (currentX > fd.length / 2.0 - valve) {
        _targetDirection = -M_PI; // Move west (negative X)
        isOutside = true;
    } else if (currentX < -fd.length / 2.0 + valve) {
        _targetDirection = 0; // Move east (positive X)
        isOutside = true;
    } else if (currentY > fd.width / 2.0 + valve) {
        _targetDirection = -M_PI / 2.0; // Move south (negative Y)
        isOutside = true;
    } else if (currentY < -fd.width / 2.0 - valve) {
        _targetDirection = M_PI / 2.0; // Move north (positive Y)
        isOutside = true;
    }
    
    if (!isOutside) {
        // Robot is already inside the field
        brain->log->logToScreen("GoBackInField", "Robot already inside field", 0x00FF00FF);
        return NodeStatus::SUCCESS;
    }
    
    // Robot is outside, start moving back
    _isMovingBack = true;
    _startTime = brain->get_clock()->now();
    
    brain->log->logToScreen("GoBackInField", 
        format("Robot outside field at (%.2f, %.2f), moving back with direction %.2f", 
               currentX, currentY, _targetDirection), 0xFFFF00FF);
    
    return NodeStatus::RUNNING;
}

NodeStatus GoBackInField::onRunning()
{
    double valve;
    getInput("valve", valve);
    
    auto fd = brain->config->fieldDimensions;
    double currentX = brain->data->robotPoseToField.x;
    double currentY = brain->data->robotPoseToField.y;
    
    // Check if we've reached the field boundary
    bool isInside = true;
    if (currentX > fd.length / 2.0 - valve) isInside = false;
    else if (currentX < -fd.length / 2.0 + valve) isInside = false;
    else if (currentY > fd.width / 2.0 + valve) isInside = false;
    else if (currentY < -fd.width / 2.0 - valve) isInside = false;
    
    if (isInside) {
        // Successfully returned to field
        brain->client->setVelocity(0, 0, 0);
        brain->log->logToScreen("GoBackInField", 
            format("Successfully returned to field at (%.2f, %.2f)", currentX, currentY), 0x00FF00FF);
        return NodeStatus::SUCCESS;
    }
    
    // Check timeout
    auto currentTime = brain->get_clock()->now();
    double elapsedMs = (currentTime - _startTime).nanoseconds() / 1e6;
    if (elapsedMs > _timeoutMs) {
        brain->client->setVelocity(0, 0, 0);
        brain->log->logToScreen("GoBackInField", 
            format("Timeout after %.1fms, stopping movement", elapsedMs), 0xFF0000FF);
        return NodeStatus::FAILURE;
    }
    
    // Continue moving back to field
    double dir_r = toPInPI(_targetDirection - brain->data->robotPoseToField.theta);
    double vx = 0.4 * cos(dir_r);
    double vy = 0.4 * sin(dir_r);
    brain->client->setVelocity(vx, vy, 0, false, false, false);
    
    return NodeStatus::RUNNING;
}

void GoBackInField::onHalted()
{
    brain->client->setVelocity(0, 0, 0);
    _isMovingBack = false;
    brain->log->logToScreen("GoBackInField", "Movement halted", 0xFFFF00FF);
}

NodeStatus TurnOnSpot::onStart()
{
    _timeStart = brain->get_clock()->now();
    _lastAngle = brain->data->robotPoseToOdom.theta;
    _cumAngle = 0.0;

    bool towardsBall = false;
    _angle = getInput<double>("rad").value();
    getInput("towards_ball", towardsBall);
    if (towardsBall) {
        double ballPixX = (brain->data->ball.boundingBox.xmin + brain->data->ball.boundingBox.xmax) / 2;
        _angle = fabs(_angle) * (ballPixX < brain->config->camPixX / 2 ? 1 : -1);
    }

    brain->client->setVelocity(0, 0, _angle, false, false, true);
    return NodeStatus::RUNNING;
}

NodeStatus TurnOnSpot::onRunning()
{
    double curAngle = brain->data->robotPoseToOdom.theta;
    double deltaAngle = toPInPI(curAngle - _lastAngle);
    _lastAngle = curAngle;
    _cumAngle += deltaAngle;
    double turnTime = brain->msecsSince(_timeStart);
    brain->log->log("debug/turn_on_spot", rerun::TextLog(format(
        "angle: %.2f, cumAngle: %.2f, deltaAngle: %.2f, time: %.2f",
        _angle, _cumAngle, deltaAngle, turnTime
    )));
    if (
        fabs(_cumAngle) - fabs(_angle) > -0.1
        || turnTime > _msecLimit
    ) {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    // else
    brain->client->setVelocity(0, 0, (_angle - _cumAngle)*2);
    return NodeStatus::RUNNING;
}


NodeStatus GoToTeammateBall::onStart()
{
    // 1. Check if we can't see the ball ourselves
    if (brain->data->ballDetected) {
        // If we can see the ball, no need to go to teammate's ball position
        brain->log->logToScreen("GoToTeammateBall", "Can see ball myself, no need to go to teammate position", 0x00FF00FF);
        return NodeStatus::FAILURE;
    }

    // 2. Get teammate ball information
    auto teammateBallInfo = brain->communication->getTeammateBallInfo();

    // 3. Find teammate who can see the ball
    BrainCommunication::TeammateInfo selectedTeammate;
    bool foundTeammateWithBall = false;

    for (const auto& teammate : teammateBallInfo) {
        if (teammate.ballDetected) {
            selectedTeammate = teammate;
            foundTeammateWithBall = true;
            break; // Select the first teammate who sees the ball
        }
    }

    // 4. If no teammate sees the ball, return failure
    if (!foundTeammateWithBall) {
        brain->log->logToScreen("GoToTeammateBall", "No teammate can see the ball", 0xFFFF00FF);
        return NodeStatus::FAILURE;
    }

    // 5. Get parameters
    getInput("long_range_threshold", _longRangeThreshold);
    getInput("turn_threshold", _turnThreshold);
    getInput("vx_limit", _vxLimit);
    getInput("vy_limit", _vyLimit);
    getInput("vtheta_limit", _vthetaLimit);
    getInput("x_tolerance", _xTolerance);
    getInput("y_tolerance", _yTolerance);
    getInput("theta_tolerance", _thetaTolerance);

    // 6. Store target position (ball position seen by teammate)
    _targetX = selectedTeammate.ballPosX;
    _targetY = selectedTeammate.ballPosY;
    _targetTheta = brain->data->robotPoseToField.theta; // Keep current orientation
    _selectedTeammateId = selectedTeammate.playerId;
    _hasValidTarget = true;

    // 7. Log information
    brain->log->logToScreen("GoToTeammateBall",
        format("Starting to go to ball position seen by teammate %d: (%.2f, %.2f)",
               _selectedTeammateId, _targetX, _targetY), 0x00FFFFFF);

    return NodeStatus::RUNNING;
}

NodeStatus GoToTeammateBall::onRunning()
{
    // 1. Check if we found the ball ourselves during movement
    if (brain->data->ballDetected) {
        brain->log->logToScreen("GoToTeammateBall", "Found ball during movement, stopping", 0x00FF00FF);
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    // 2. Check if we don't have a valid target
    if (!_hasValidTarget) {
        brain->log->logToScreen("GoToTeammateBall", "No valid target", 0xFFFF00FF);
        return NodeStatus::FAILURE;
    }

    // 3. Check if we have reached the target position
    double currentX = brain->data->robotPoseToField.x;
    double currentY = brain->data->robotPoseToField.y;
    double currentTheta = brain->data->robotPoseToField.theta;

    bool reachedX = fabs(currentX - _targetX) < _xTolerance;
    bool reachedY = fabs(currentY - _targetY) < _yTolerance;
    bool reachedTheta = fabs(toPInPI(currentTheta - _targetTheta)) < _thetaTolerance;

    if (reachedX && reachedY && reachedTheta) {
        brain->log->logToScreen("GoToTeammateBall",
            format("Reached teammate ball position (%.2f, %.2f)", _targetX, _targetY), 0x00FF00FF);
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    // 4. Continue moving towards target position
    brain->client->moveToPoseOnField(_targetX, _targetY, _targetTheta,
                                   _longRangeThreshold, _turnThreshold,
                                   _vxLimit, _vyLimit, _vthetaLimit,
                                   _xTolerance, _yTolerance, _thetaTolerance);

    return NodeStatus::RUNNING;
}

void GoToTeammateBall::onHalted()
{
    brain->client->setVelocity(0, 0, 0);
    _hasValidTarget = false;
    brain->log->logToScreen("GoToTeammateBall", "Movement halted", 0xFFFF00FF);
}

NodeStatus PrintMsg::tick()
{
    Expected<std::string> msg = getInput<std::string>("msg");
    if (!msg)
    {
        throw RuntimeError("missing required input [msg]: ", msg.error());
    }
    std::cout << "[MSG] " << msg.value() << std::endl;
    return NodeStatus::SUCCESS;
}

// ------------------------------- GOALKEEPER NODES -------------------------------

NodeStatus GoalKeeperPosition::onStart()
{
    // Get all input parameters
    getInput("base_x", _baseX);
    getInput("base_y", _baseY);
    getInput("base_theta", _baseTheta);
    getInput("y_adjustment_factor", _yAdjustmentFactor);
    getInput("max_y_offset", _maxYOffset);
    getInput("vx_limit", _vxLimit);
    getInput("vy_limit", _vyLimit);
    getInput("vtheta_limit", _vthetaLimit);
    getInput("position_tolerance", _positionTolerance);
    getInput("angle_tolerance", _angleTolerance);

    brain->log->logToScreen("GoalKeeperPosition", "Starting goalkeeper positioning", 0x00FFFFFF);

    return NodeStatus::RUNNING;
}

NodeStatus GoalKeeperPosition::onRunning()
{
    // Recalculate target position every tick to respond to ball movement
    _targetX = _baseX;
    _targetY = _baseY;
    _targetTheta = _baseTheta;

    // Adjust Y position based on ball position (if ball is detected)
    if (brain->tree->getEntry<bool>("ball_location_known")) {
        double ballY = brain->data->ball.posToField.y;
        double yAdjustment = ballY * _yAdjustmentFactor;

        // Cap the adjustment to maximum offset
        yAdjustment = cap(yAdjustment, _maxYOffset, -_maxYOffset);
        _targetY = _baseY + yAdjustment;

        brain->log->logToScreen("GoalKeeperPosition",
            format("Ball Y: %.2f, Target Y: %.2f", ballY, _targetY), 0x00FFFFFF);
    }

    // Check if we've reached the target position
    double currentX = brain->data->robotPoseToField.x;
    double currentY = brain->data->robotPoseToField.y;
    double currentTheta = brain->data->robotPoseToField.theta;

    bool reachedX = fabs(currentX - _targetX) < _positionTolerance;
    bool reachedY = fabs(currentY - _targetY) < _positionTolerance;
    bool reachedTheta = fabs(toPInPI(currentTheta - _targetTheta)) < _angleTolerance;

    if (reachedX && reachedY && reachedTheta) {
        brain->client->setVelocity(0, 0, 0);  // Stop movement
        brain->log->logToScreen("GoalKeeperPosition",
            format("Reached target position (%.2f, %.2f, %.2f)", _targetX, _targetY, _targetTheta), 0x00FF00FF);
        return NodeStatus::SUCCESS;
    }

    // Continue moving towards target position
    brain->client->moveToPoseOnField(_targetX, _targetY, _targetTheta,
                                   2.0, 0.4, // long_range_threshold, turn_threshold
                                   _vxLimit, _vyLimit, _vthetaLimit,
                                   _positionTolerance, _positionTolerance, _angleTolerance);

    return NodeStatus::RUNNING;
}

void GoalKeeperPosition::onHalted()
{
    brain->client->setVelocity(0, 0, 0);  // Stop movement when halted
    brain->log->logToScreen("GoalKeeperPosition", "Goalkeeper positioning halted", 0xFFFF00FF);
}

NodeStatus GoalKeeperIntercept::onStart()
{
    double interceptDistance, predictionTime;
    getInput("intercept_distance", interceptDistance);
    getInput("prediction_time", predictionTime);

    _startTime = brain->get_clock()->now();
    _hasValidIntercept = false;
    _phase = TURN_TO_BALL;  // Start with turning phase

    // Check if ball is close enough to intercept
    if (!brain->tree->getEntry<bool>("ball_location_known")) {
        brain->log->logToScreen("GoalKeeperIntercept", "Ball not detected, can't intercept", 0xFFFF00FF);
        return NodeStatus::FAILURE;
    }

    double ballRange = brain->data->ball.range;
    if (ballRange > interceptDistance) {
        brain->log->logToScreen("GoalKeeperIntercept",
            format("Ball too far: %.2f > %.2f", ballRange, interceptDistance), 0xFFFF00FF);
        return NodeStatus::FAILURE;
    }

    // Calculate intercept position
    double ballX = brain->data->ball.posToField.x;
    double ballY = brain->data->ball.posToField.y;

    // Simple prediction: assume ball continues in current direction
    // For now, just intercept at current ball position
    _interceptX = ballX;
    _interceptY = ballY;
    _hasValidIntercept = true;

    // Calculate target angle to face the ball
    _targetAngle = atan2(ballY - brain->data->robotPoseToField.y,
                        ballX - brain->data->robotPoseToField.x);

    brain->log->logToScreen("GoalKeeperIntercept",
        format("Starting intercept at (%.2f, %.2f), target angle: %.2f", _interceptX, _interceptY, _targetAngle), 0x00FF00FF);

    return NodeStatus::RUNNING;
}

NodeStatus GoalKeeperIntercept::onRunning()
{
    if (!_hasValidIntercept) {
        return NodeStatus::FAILURE;
    }

    double vxLimit, vyLimit, vthetaLimit;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);

    // Check timeout (5 seconds max)
    if (brain->msecsSince(_startTime) > 5000) {
        brain->client->setVelocity(0, 0, 0);
        brain->log->logToScreen("GoalKeeperIntercept", "Intercept timeout", 0xFFFF00FF);
        return NodeStatus::FAILURE;
    }

    double currentX = brain->data->robotPoseToField.x;
    double currentY = brain->data->robotPoseToField.y;
    double currentTheta = brain->data->robotPoseToField.theta;

    // Phase 1: Turn to face the ball
    if (_phase == TURN_TO_BALL) {
        double angleDiff = toPInPI(_targetAngle - currentTheta);

        if (fabs(angleDiff) < 0.1) { // Within 5.7 degrees
            _phase = MOVE_TO_BALL;
            brain->log->logToScreen("GoalKeeperIntercept", "Turned to face ball, starting movement", 0x00FF00FF);
        } else {
            // Turn toward the ball
            double vtheta = angleDiff * 0.8; // Proportional control
            vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);
            brain->client->setVelocity(0, 0, vtheta);
            brain->log->logToScreen("GoalKeeperIntercept",
                format("Turning to face ball: angle diff=%.2f", angleDiff), 0x00FFFFFF);
            return NodeStatus::RUNNING;
        }
    }

        // Phase 2: Move toward the ball
    if (_phase == MOVE_TO_BALL) {
        double distanceToIntercept = sqrt(pow(currentX - _interceptX, 2) + pow(currentY - _interceptY, 2));

        if (distanceToIntercept < 0.3) { // Within 30cm of intercept point
            _phase = MOVE_FORWARD_TO_INTERCEPT;
            brain->log->logToScreen("GoalKeeperIntercept", "Reached ball, preparing to move forward", 0x00FF00FF);
            return NodeStatus::RUNNING;
        }

        // Move toward intercept position while maintaining ball-facing orientation
        brain->client->moveToPoseOnField(_interceptX, _interceptY, _targetAngle, // face the ball
                                       2.0, 0.4, // long_range_threshold, turn_threshold
                                       vxLimit, vyLimit, vthetaLimit,
                                       0.3, 0.3, 0.2); // tolerances

        brain->log->logToScreen("GoalKeeperIntercept",
            format("Moving to ball: distance=%.2f", distanceToIntercept), 0x00FFFFFF);
    }

    // Phase 3: Move forward to intercept the ball
    if (_phase == MOVE_FORWARD_TO_INTERCEPT) {
        // Check if ball is still close enough to intercept
        double ballRange = brain->data->ball.range;
        double ballX = brain->data->ball.posToField.x;
        double goalpostX = -6.5; // Goalpost X position (left goal)
        double distanceToGoal = fabs(ballX - goalpostX);
        if (ballRange > 0.5) { // Ball moved away
            brain->log->logToScreen("GoalKeeperIntercept", "Ball moved away, restarting intercept", 0xFFFF00FF);
            _phase = TURN_TO_BALL; // Restart the process
            return NodeStatus::RUNNING;
        }

        // --- Ensure we are facing the opponent's goal (positive X direction) ---
        // Calculate desired kick direction: from ball to opponent's goal (center X = +field length / 2, Y = 0)
        double fieldLength = brain->config->fieldDimensions.length;
        double goalX = fieldLength / 2.0;
        double goalY = 0.0;
        double ballY = brain->data->ball.posToField.y;
        double desiredKickAngle = atan2(goalY - ballY, goalX - ballX);
        double currentTheta = brain->data->robotPoseToField.theta;
        double angleDiff = toPInPI(desiredKickAngle - currentTheta);

        // If not facing the correct direction, turn first
        if (fabs(angleDiff) > 0.2) {
            double vtheta = angleDiff * 0.8;
            vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);
            brain->client->setVelocity(0, 0, vtheta);
            brain->log->logToScreen("GoalKeeperIntercept",
                format("Turning to face opponent's goal: angle diff=%.2f", angleDiff), 0x00FF00FF);
            return NodeStatus::RUNNING;
        }

        // Ball is close and we're facing the opponent's goal - MOVE FORWARD TO INTERCEPT/KICK!
        double vx = vxLimit * 0.8; // Move forward at 80% of max speed
        double vy = 0.0; // No lateral movement
        double vtheta = 0.0; // No turning

        brain->client->setVelocity(vx, vy, vtheta);
        brain->log->logToScreen("GoalKeeperIntercept", "MOVING FORWARD TO INTERCEPT BALL TOWARD OPPONENT'S GOAL!", 0xFF0000FF);

        // Continue moving forward for a short time to intercept
        if (brain->msecsSince(_startTime) > 5000) {
            brain->client->setVelocity(0, 0, 0); // Stop movement
            brain->log->logToScreen("GoalKeeperIntercept", "Intercept completed", 0x00FF00FF);
            return NodeStatus::SUCCESS;
        }

        return NodeStatus::RUNNING;
    }

    return NodeStatus::RUNNING;
}

void GoalKeeperIntercept::onHalted()
{
    brain->client->setVelocity(0, 0, 0);
    _hasValidIntercept = false;
    brain->log->logToScreen("GoalKeeperIntercept", "Intercept halted", 0xFFFF00FF);
}

NodeStatus GoalKeeperTrackAndAdjust::onStart()
{
    getInput("base_x", _baseX);
    getInput("adjustment_speed", _adjustmentSpeed);
    getInput("ball_y_factor", _ballYFactor);
    getInput("max_adjustment", _maxAdjustment);
    getInput("vx_limit", _vxLimit);
    getInput("vy_limit", _vyLimit);
    getInput("vtheta_limit", _vthetaLimit);
    getInput("position_tolerance", _positionTolerance);

    brain->log->logToScreen("GoalKeeperTrackAndAdjust", "Starting goalkeeper tracking and adjustment", 0x00FFFFFF);

    return NodeStatus::RUNNING;
}

NodeStatus GoalKeeperTrackAndAdjust::onRunning()
{
    // Default position - only Y axis movement
    double targetX = _baseX;  // Fixed X position at goal line
    double targetY = 0.0;
    double targetTheta = 1.57; // 90 degrees

    // Adjust position based on ball
    if (brain->tree->getEntry<bool>("ball_location_known")) {
        double ballX = brain->data->ball.posToField.x;
        double ballY = brain->data->ball.posToField.y;
        double ballRange = brain->data->ball.range;

        // Only adjust Y position based on ball Y position
        // No X adjustment - robot stays at goal line (X = -6.5)
        double yAdjustment = ballY * _ballYFactor;
        yAdjustment = cap(yAdjustment, _maxAdjustment, -_maxAdjustment);
        targetY = yAdjustment;

        // No X position adjustment - always stay at base position
        targetX = _baseX;  // Always at goal line

        brain->log->logToScreen("GoalKeeperTrackAndAdjust",
            format("Ball (%.2f, %.2f), Target (%.2f, %.2f) - Y-only movement", ballX, ballY, targetX, targetY), 0x00FFFFFF);
    }

    // Get current position
    double currentX = brain->data->robotPoseToField.x;
    double currentY = brain->data->robotPoseToField.y;
    double currentTheta = brain->data->robotPoseToField.theta;

    // Check if we're close enough to target (for continuous adjustment, we use a smaller tolerance)
    bool closeToTarget = (fabs(currentX - targetX) < _positionTolerance) &&
                        (fabs(currentY - targetY) < _positionTolerance) &&
                        (fabs(toPInPI(currentTheta - targetTheta)) < 0.1);

    // For tracking and adjustment, we continue running even when close to target
    // because the target position changes based on ball movement

    // Calculate velocity for smooth movement - only Y axis
    double vx = 0.0;  // No X movement - stay at goal line
    double vy = (targetY - currentY) * _adjustmentSpeed;
    double vtheta = toPInPI(targetTheta - currentTheta) * 0.5;

    // Apply limits
    vx = 0.0;  // Force X velocity to zero
    vy = cap(vy, _vyLimit, -_vyLimit);
    vtheta = cap(vtheta, _vthetaLimit, -_vthetaLimit);

    brain->client->setVelocity(vx, vy, vtheta);

    // Keep running to continuously adjust position
    return NodeStatus::RUNNING;
}

void GoalKeeperTrackAndAdjust::onHalted()
{
    brain->client->setVelocity(0, 0, 0);
    brain->log->logToScreen("GoalKeeperTrackAndAdjust", "Goalkeeper tracking halted", 0xFFFF00FF);
}

// ------------------------------- FOLLOW TEAMMATE IMPLEMENTATION -------------------------------

NodeStatus FollowTeammate::onStart()
{
    // Get parameters
    getInput("follow_distance", _followDistance);
    getInput("long_range_threshold", _longRangeThreshold);
    getInput("turn_threshold", _turnThreshold);
    getInput("vx_limit", _vxLimit);
    getInput("vy_limit", _vyLimit);
    getInput("vtheta_limit", _vthetaLimit);
    getInput("position_tolerance", _positionTolerance);
    getInput("angle_tolerance", _angleTolerance);

    // Check if we have ball possession (if yes, we shouldn't follow)
    if (brain->tree->getEntry<bool>("has_ball_possession")) {
        brain->log->logToScreen("FollowTeammate", "I have ball possession, no need to follow", 0x00FF00FF);
        return NodeStatus::FAILURE;
    }

    // Get possession player ID
    _possessionPlayerId = brain->tree->getEntry<int>("possession_player_id");
    if (_possessionPlayerId == -1) {
        brain->log->logToScreen("FollowTeammate", "No robot has ball possession", 0xFFFF00FF);
        return NodeStatus::FAILURE;
    }

    _hasValidTarget = false;
    brain->log->logToScreen("FollowTeammate",
        format("Starting to follow robot %d", _possessionPlayerId), 0x00FFFFFF);

    return NodeStatus::RUNNING;
}

NodeStatus FollowTeammate::onRunning()
{
    // Check if we now have ball possession
    if (brain->tree->getEntry<bool>("has_ball_possession")) {
        brain->log->logToScreen("FollowTeammate", "I now have ball possession, stopping follow", 0x00FF00FF);
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    // Get current possession player ID
    int currentPossessionId = brain->tree->getEntry<int>("possession_player_id");
    if (currentPossessionId == -1) {
        brain->log->logToScreen("FollowTeammate", "No robot has ball possession", 0xFFFF00FF);
        return NodeStatus::FAILURE;
    }

    // Update possession player ID if changed
    if (currentPossessionId != _possessionPlayerId) {
        _possessionPlayerId = currentPossessionId;
        brain->log->logToScreen("FollowTeammate",
            format("Ball possession changed, now following robot %d", _possessionPlayerId), 0x00FFFFFF);
    }

    // Get teammate positions
    auto teammates = brain->communication->getTeammatePositions();

    // Find the teammate with ball possession
    BrainCommunication::TeammateInfo possessionTeammate;
    bool foundPossessionTeammate = false;

    for (const auto& teammate : teammates) {
        if (teammate.playerId == _possessionPlayerId && teammate.hasValidPose) {
            possessionTeammate = teammate;
            foundPossessionTeammate = true;
            break;
        }
    }

    if (!foundPossessionTeammate) {
        brain->log->logToScreen("FollowTeammate",
            format("Cannot find position of robot %d with ball possession", _possessionPlayerId), 0xFFFF00FF);
        return NodeStatus::FAILURE;
    }

    // Get ball position (we need this to calculate the optimal follow position)
    double ballX = 0.0, ballY = 0.0;
    bool ballPosKnown = false;

    if (brain->tree->getEntry<bool>("ball_location_known")) {
        ballX = brain->data->ball.posToField.x;
        ballY = brain->data->ball.posToField.y;
        ballPosKnown = true;
    } else {
        // Try to get ball position from teammate information
        auto teammateBallInfo = brain->communication->getTeammateBallInfo();
        for (const auto& teammate : teammateBallInfo) {
            if (teammate.ballDetected) {
                ballX = teammate.ballPosX;
                ballY = teammate.ballPosY;
                ballPosKnown = true;
                break;
            }
        }
    }

    if (!ballPosKnown) {
        brain->log->logToScreen("FollowTeammate", "Ball position unknown, cannot determine follow position", 0xFFFF00FF);
        return NodeStatus::FAILURE;
    }

    // Calculate follow position
    auto followPos = calculateFollowPosition(possessionTeammate.robotPoseX, possessionTeammate.robotPoseY,
                                           ballX, ballY, _followDistance);
    _targetX = followPos.first;
    _targetY = followPos.second;

    // Calculate target orientation to face the ball
    _targetTheta = calculateBallViewingAngle(_targetX, _targetY, ballX, ballY);
    _hasValidTarget = true;

    // Check if we've reached the target position
    double currentX = brain->data->robotPoseToField.x;
    double currentY = brain->data->robotPoseToField.y;
    double currentTheta = brain->data->robotPoseToField.theta;

    bool reachedPosition = (fabs(currentX - _targetX) < _positionTolerance) &&
                          (fabs(currentY - _targetY) < _positionTolerance);
    bool reachedAngle = fabs(toPInPI(currentTheta - _targetTheta)) < _angleTolerance;

    if (reachedPosition && reachedAngle) {
        brain->log->logToScreen("FollowTeammate",
            format("Following robot %d at position (%.2f, %.2f)", _possessionPlayerId, _targetX, _targetY), 0x00FFFFFF);
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::RUNNING; // Keep following (continuous behavior)
    }

    // Move towards target position
    brain->client->moveToPoseOnField(_targetX, _targetY, _targetTheta,
                                   _longRangeThreshold, _turnThreshold,
                                   _vxLimit, _vyLimit, _vthetaLimit,
                                   _positionTolerance, _positionTolerance, _angleTolerance);

    return NodeStatus::RUNNING;
}

void FollowTeammate::onHalted()
{
    brain->client->setVelocity(0, 0, 0);
    _hasValidTarget = false;
    brain->log->logToScreen("FollowTeammate", "Follow teammate halted", 0xFFFF00FF);
}

std::pair<double, double> FollowTeammate::calculateFollowPosition(double teammateX, double teammateY,
                                                                double ballX, double ballY, double followDistance)
{
    // Calculate positions towards our goal direction
    // Assume our goal is at negative X direction (left side)
    double ourGoalX = -brain->config->fieldDimensions.length / 2.0;
    
    // Calculate direction vector from striker to our goal
    double goalDirX = ourGoalX - teammateX;
    double goalDirLength = fabs(goalDirX); // Only use X direction magnitude
    
    // Normalize the direction (only X component matters, Y direction will be perpendicular)
    double dirX = (goalDirLength > 1e-6) ? goalDirX / goalDirLength : -1.0; // Default to left if too close to goal
    
    // Calculate the two candidate positions:
    // Both positions are towards our goal direction from striker, with vertical offset
    double baseFollowX = teammateX + dirX * followDistance;  // Move towards our goal
    double baseFollowY = teammateY;  // Same Y as striker
    
    // Two candidate positions: above and below the striker
    double upPositionX = baseFollowX;
    double upPositionY = baseFollowY + followDistance * 0.5;    // Above striker
    
    double downPositionX = baseFollowX; 
    double downPositionY = baseFollowY - followDistance * 0.5;   // Below striker

    // Calculate distances from current position to both options
    double currentX = brain->data->robotPoseToField.x;
    double currentY = brain->data->robotPoseToField.y;

    double distToUp = sqrt(pow(currentX - upPositionX, 2) + pow(currentY - upPositionY, 2));
    double distToDown = sqrt(pow(currentX - downPositionX, 2) + pow(currentY - downPositionY, 2));

    // Choose the closer position
    if (distToUp < distToDown) {
        brain->log->logToScreen("FollowTeammate",
            format("Following up position towards goal (%.2f, %.2f)", upPositionX, upPositionY), 0x00FFFFFF);
        return {upPositionX, upPositionY};
    } else {
        brain->log->logToScreen("FollowTeammate",
            format("Following down position towards goal (%.2f, %.2f)", downPositionX, downPositionY), 0x00FFFFFF);
        return {downPositionX, downPositionY};
    }
}

double FollowTeammate::calculateBallViewingAngle(double robotX, double robotY, double ballX, double ballY)
{
    // Calculate angle to face the ball
    return atan2(ballY - robotY, ballX - robotX);
}

// ===== TurnByAngle Implementation =====

NodeStatus TurnByAngle::onStart()
{
    _startTime = brain->get_clock()->now();
    _startAngle = brain->data->robotPoseToOdom.theta;
    
    // Get input parameters
    _turnAngle = getInput<double>("rad").value();
    _angularVelocity = getInput<double>("angular_velocity").value();
    _tolerance = getInput<double>("tolerance").value();
    _timeoutMs = getInput<int>("timeout_ms").value();
    
    // Calculate target angle
    _targetAngle = _startAngle + _turnAngle;
    
    // Start turning with specified angular velocity
    double initialVelocity = _turnAngle > 0 ? _angularVelocity : -_angularVelocity;
    brain->client->setVelocity(0, 0, initialVelocity);
    
    brain->log->log("debug/turn_by_angle", rerun::TextLog(format(
        "Starting turn: start_angle=%.3f, turn_angle=%.3f, target_angle=%.3f, velocity=%.3f",
        _startAngle, _turnAngle, _targetAngle, initialVelocity
    )));
    
    return NodeStatus::RUNNING;
}

NodeStatus TurnByAngle::onRunning()
{
    double currentAngle = brain->data->robotPoseToOdom.theta;
    double turnTime = brain->msecsSince(_startTime);
    
    // Check timeout
    if (turnTime > _timeoutMs) {
        brain->client->setVelocity(0, 0, 0);
        brain->log->log("debug/turn_by_angle", rerun::TextLog("Turn timeout"));
        return NodeStatus::FAILURE;
    }
    
    // Calculate angle difference to target
    double angleDiff = toPInPI(_targetAngle - currentAngle);
    
    // Check if we've reached the target
    if (fabs(angleDiff) < _tolerance) {
        brain->client->setVelocity(0, 0, 0);
        brain->log->log("debug/turn_by_angle", rerun::TextLog(format(
            "Turn completed: current_angle=%.3f, target_angle=%.3f, diff=%.3f",
            currentAngle, _targetAngle, angleDiff
        )));
        return NodeStatus::SUCCESS;
    }
    
    // Continue turning with proportional control
    double velocityScale = fabs(angleDiff) > 0.3 ? 1.0 : fabs(angleDiff) / 0.3; // Slow down near target
    double angularVel = (angleDiff > 0 ? _angularVelocity : -_angularVelocity) * velocityScale;
    
    brain->client->setVelocity(0, 0, angularVel);
    
    brain->log->log("debug/turn_by_angle", rerun::TextLog(format(
        "Turning: current=%.3f, target=%.3f, diff=%.3f, vel=%.3f, time=%.0f",
        currentAngle, _targetAngle, angleDiff, angularVel, turnTime
    )));
    
    return NodeStatus::RUNNING;
}

void TurnByAngle::onHalted()
{
    brain->client->setVelocity(0, 0, 0);
    brain->log->log("debug/turn_by_angle", rerun::TextLog("Turn halted"));
}

// ===== TurnToAngle Implementation =====

NodeStatus TurnToAngle::onStart()
{
    _startTime = brain->get_clock()->now();
    
    // Get input parameters
    _targetAngle = getInput<double>("target_angle").value();
    _angularVelocity = getInput<double>("angular_velocity").value();
    _tolerance = getInput<double>("tolerance").value();
    _timeoutMs = getInput<int>("timeout_ms").value();
    
    double currentAngle = brain->data->robotPoseToField.theta;
    double angleDiff = toPInPI(_targetAngle - currentAngle);
    
    // Start turning with specified angular velocity
    double initialVelocity = angleDiff > 0 ? _angularVelocity : -_angularVelocity;
    brain->client->setVelocity(0, 0, initialVelocity);
    
    brain->log->log("debug/turn_to_angle", rerun::TextLog(format(
        "Starting turn to angle: current=%.3f, target=%.3f, diff=%.3f, velocity=%.3f",
        currentAngle, _targetAngle, angleDiff, initialVelocity
    )));
    
    return NodeStatus::RUNNING;
}

NodeStatus TurnToAngle::onRunning()
{
    double currentAngle = brain->data->robotPoseToField.theta;
    double turnTime = brain->msecsSince(_startTime);
    
    // Check timeout
    if (turnTime > _timeoutMs) {
        brain->client->setVelocity(0, 0, 0);
        brain->log->log("debug/turn_to_angle", rerun::TextLog("Turn timeout"));
        return NodeStatus::FAILURE;
    }
    
    // Calculate angle difference to target
    double angleDiff = toPInPI(_targetAngle - currentAngle);
    
    // Check if we've reached the target
    if (fabs(angleDiff) < _tolerance) {
        brain->client->setVelocity(0, 0, 0);
        brain->log->log("debug/turn_to_angle", rerun::TextLog(format(
            "Turn completed: current_angle=%.3f, target_angle=%.3f, diff=%.3f",
            currentAngle, _targetAngle, angleDiff
        )));
        return NodeStatus::SUCCESS;
    }
    
    // Continue turning with proportional control
    double velocityScale = fabs(angleDiff) > 0.3 ? 1.0 : fabs(angleDiff) / 0.3; // Slow down near target
    double angularVel = (angleDiff > 0 ? _angularVelocity : -_angularVelocity) * velocityScale;
    
    brain->client->setVelocity(0, 0, angularVel);
    
    brain->log->log("debug/turn_to_angle", rerun::TextLog(format(
        "Turning: current=%.3f, target=%.3f, diff=%.3f, vel=%.3f, time=%.0f",
        currentAngle, _targetAngle, angleDiff, angularVel, turnTime
    )));
    
    return NodeStatus::RUNNING;
}

void TurnToAngle::onHalted()
{
    brain->client->setVelocity(0, 0, 0);
    brain->log->log("debug/turn_to_angle", rerun::TextLog("Turn halted"));
}

// ------------------------------- GOAL KEEPER Y-AXIS DEFENSE IMPLEMENTATION -------------------------------

NodeStatus GoalKeeperYAxisDefense::onStart()
{
    // Get parameters
    getInput("base_x", _baseX);
    getInput("base_y", _baseY);
    getInput("base_theta", _baseTheta);
    getInput("prediction_distance", _predictionDistance);
    getInput("reaction_speed", _reactionSpeed);
    getInput("vx_limit", _vxLimit);
    getInput("vy_limit", _vyLimit);
    getInput("vtheta_limit", _vthetaLimit);

    // Initialize state
    _phase = ORIENT_Y_AXIS;
    _ballDirection = BALL_UNKNOWN;
    _lastBallX = 0.0;
    _lastBallY = 0.0;
    _lastBallTime = brain->get_clock()->now();

    brain->log->logToScreen("GoalKeeperYAxisDefense", "Starting Y-axis defense strategy", 0x00FF00FF);
    return NodeStatus::RUNNING;
}

NodeStatus GoalKeeperYAxisDefense::onRunning()
{
    // Check if ball is detected
    if (!brain->tree->getEntry<bool>("ball_location_known")) {
        brain->log->logToScreen("GoalKeeperYAxisDefense", "Ball not detected, maintaining position", 0xFFFF00FF);
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::RUNNING;
    }

    double ballRange = brain->data->ball.range;
    double currentX = brain->data->robotPoseToField.x;
    double currentY = brain->data->robotPoseToField.y;
    double currentTheta = brain->data->robotPoseToField.theta;

    // Phase 1: Orient along Y-axis
    if (_phase == ORIENT_Y_AXIS) {
        // Calculate angle difference to Y-axis orientation
        double angleDiff = toPInPI(_baseTheta - currentTheta);
        
        // Adjust Y position based on ball Y position
        double ballY = brain->data->ball.posToField.y;
        double targetY = _baseY + (ballY * 0.3); // Adjust Y position based on ball
        targetY = cap(targetY, 1.0, -1.0); // Limit Y adjustment
        
        // Check if we're properly oriented and positioned
        bool orientedCorrectly = fabs(angleDiff) < 0.1; // Within ~6 degrees
        bool positionedCorrectly = fabs(currentX - _baseX) < 0.2 && fabs(currentY - targetY) < 0.2;
        
        if (orientedCorrectly && positionedCorrectly) {
            _phase = PREDICT_AND_REACT;
            brain->log->logToScreen("GoalKeeperYAxisDefense", "Y-axis orientation complete, starting prediction", 0x00FF00FF);
        } else {
            // Move to proper position with Y-axis orientation
            brain->client->moveToPoseOnField(_baseX, targetY, _baseTheta,
                                           2.0, 0.4, // long_range_threshold, turn_threshold
                                           _vxLimit * 0.6, _vyLimit * 0.6, _vthetaLimit * 0.8,
                                           0.2, 0.2, 0.1); // tolerances
            
            brain->log->logToScreen("GoalKeeperYAxisDefense",
                format("Orienting to Y-axis: angle_diff=%.2f, pos=(%.2f,%.2f)", angleDiff, currentX, currentY), 0x00FFFFFF);
        }
        return NodeStatus::RUNNING;
    }

    // Phase 2: Predict ball direction and react
    if (_phase == PREDICT_AND_REACT) {
        // Predict ball direction
        _ballDirection = predictBallDirection();
        
        // React based on prediction
        if (ballRange <= _predictionDistance) {
            _phase = INTERCEPT;
            brain->log->logToScreen("GoalKeeperYAxisDefense", "Ball close enough, starting intercept", 0xFF0000FF);
        } else {
            executeReaction();
        }
        return NodeStatus::RUNNING;
    }

    // Phase 3: Intercept the ball
    if (_phase == INTERCEPT) {
        executeReaction();
        return NodeStatus::RUNNING;
    }

    return NodeStatus::RUNNING;
}

GoalKeeperYAxisDefense::BallDirection GoalKeeperYAxisDefense::predictBallDirection()
{
    double currentBallX = brain->data->ball.posToField.x;
    double currentBallY = brain->data->ball.posToField.y;
    rclcpp::Time currentTime = brain->get_clock()->now();
    
    // Calculate time difference
    double timeDiff = (currentTime - _lastBallTime).seconds();
    
    if (timeDiff < 0.1) { // Too little time passed
        return _ballDirection;
    }
    
    // Calculate ball velocity
    double ballVelX = (currentBallX - _lastBallX) / timeDiff;
    double ballVelY = (currentBallY - _lastBallY) / timeDiff;
    
    // Update last position and time
    _lastBallX = currentBallX;
    _lastBallY = currentBallY;
    _lastBallTime = currentTime;
    
    // Determine direction based on Y velocity (since we're Y-axis oriented)
    // When robot is Y-axis oriented, front/back corresponds to +Y/-Y direction
    const double velocityThreshold = 0.1; // m/s
    
    if (fabs(ballVelY) < velocityThreshold) {
        return BALL_STATIONARY;
    } else if (ballVelY > 0) {
        // Ball moving toward positive Y (front of robot)
        return BALL_MOVING_FORWARD;
    } else {
        // Ball moving toward negative Y (back of robot)
        return BALL_MOVING_BACKWARD;
    }
}

void GoalKeeperYAxisDefense::executeReaction()
{
    double currentX = brain->data->robotPoseToField.x;
    double currentY = brain->data->robotPoseToField.y;
    double currentTheta = brain->data->robotPoseToField.theta;
    
    // Always maintain Y-axis orientation
    double angleDiff = toPInPI(_baseTheta - currentTheta);
    double vtheta = angleDiff * 0.8;
    vtheta = cap(vtheta, _vthetaLimit, -_vthetaLimit);
    
    // Maintain X position at goal line
    double vx = ((_baseX - currentX) * 0.5);
    vx = cap(vx, _vxLimit * 0.3, -_vxLimit * 0.3); // Gentle X adjustment
    
    double vy = 0.0;
    
    // React based on predicted ball direction - move forward/backward along Y-axis
    if (_ballDirection == BALL_MOVING_FORWARD) {
        // Ball moving toward front (positive Y) - move forward rapidly along Y-axis
        vy = _vyLimit * _reactionSpeed;
        brain->log->logToScreen("GoalKeeperYAxisDefense", "Ball moving to front - MOVING FORWARD (Y+)!", 0xFF0000FF);
    } else if (_ballDirection == BALL_MOVING_BACKWARD) {
        // Ball moving toward back (negative Y) - move backward rapidly along Y-axis
        vy = -_vyLimit * _reactionSpeed;
        brain->log->logToScreen("GoalKeeperYAxisDefense", "Ball moving to back - MOVING BACKWARD (Y-)!", 0xFF0000FF);
    } else {
        // Ball stationary or unknown - slight adjustment based on ball Y position
        double ballY = brain->data->ball.posToField.y;
        double targetY = _baseY + (ballY * 0.2); // Small adjustment factor
        targetY = cap(targetY, 1.0, -1.0);
        vy = (targetY - currentY) * 0.3;
        vy = cap(vy, _vyLimit * 0.4, -_vyLimit * 0.4);
        brain->log->logToScreen("GoalKeeperYAxisDefense", "Ball stationary - minor Y adjustment", 0x00FFFFFF);
    }
    
    // Apply velocity commands
    brain->client->setVelocity(vx, vy, vtheta);
    
    brain->log->logToScreen("GoalKeeperYAxisDefense",
        format("Direction: %d, Velocity: (%.2f, %.2f, %.2f)", _ballDirection, vx, vy, vtheta), 0x00FFFFFF);
}

void GoalKeeperYAxisDefense::onHalted()
{
    brain->client->setVelocity(0, 0, 0);
    brain->log->logToScreen("GoalKeeperYAxisDefense", "Y-axis defense halted", 0xFFFF00FF);
}
