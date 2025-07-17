#include <iostream>
#include <string>
#include <limits>
#include <cmath>

#include "brain.h"
#include "utils/print.h"
#include "utils/math.h"

using namespace std;
using std::placeholders::_1;

Brain::Brain() : rclcpp::Node("brain_node")
{
    // Note that the parameters must be declared here first, otherwise they cannot be read in the program either.
    declare_parameter<int>("game.team_id", 0);
    declare_parameter<int>("game.player_id", 29);
    declare_parameter<string>("game.field_type", "");

    declare_parameter<string>("game.player_role", "");
    declare_parameter<string>("game.player_start_pos", "");
    declare_parameter<string>("game.collaboration_role", "slave");

    declare_parameter<double>("robot.robot_height", 1.0);
    declare_parameter<double>("robot.odom_factor", 1.0);
    declare_parameter<double>("robot.vx_factor", 0.95);
    declare_parameter<double>("robot.yaw_offset", 0.1);

    declare_parameter<bool>("rerunLog.enable", false);
    declare_parameter<string>("rerunLog.server_addr", "");
    declare_parameter<int>("rerunLog.img_interval", 10);

    declare_parameter<bool>("enable_com", true);

    // The tree_file_path is configured in launch.py and not placed in config.yaml.
    declare_parameter<string>("tree_file_path", "");
}

void Brain::init()
{
    // Make sure to load the configuration first, and then the config can be used.
    config = std::make_shared<BrainConfig>();
    loadConfig();

    data = std::make_shared<BrainData>();
    locator = std::make_shared<Locator>();

    log = std::make_shared<BrainLog>(this);
    tree = std::make_shared<BrainTree>(this);
    client = std::make_shared<RobotClient>(this);
    voiceClient = std::make_shared<VoiceClient>(this);
    communication = std::make_shared<BrainCommunication>(this);
    strategy = std::make_shared<RobotStrategy>(RobotConstants(), config->fieldDimensions);

    locator->init(config->fieldDimensions, 4, 0.5);

    tree->init();

    client->init();

    voiceClient->init();

    log->prepare();

    communication->initUDPBroadcast();

    data->lastSuccessfulLocalizeTime = get_clock()->now();

    // Initialize loop timing statistics
    totalLoopTime = 0.0;
    maxLoopTime = 0.0;
    minLoopTime = std::numeric_limits<double>::max();
    loopCount = 0;

    joySubscription = create_subscription<booster_interface::msg::RemoteControllerState>("/remote_controller_state", 10, bind(&Brain::joystickCallback, this, _1));
    gameControlSubscription = create_subscription<game_controller_interface::msg::GameControlData>("/robocup/game_controller", 1, bind(&Brain::gameControlCallback, this, _1));
    detectionsSubscription = create_subscription<vision_interface::msg::Detections>("/booster_vision/detection", 1, bind(&Brain::detectionsCallback, this, _1));
    odometerSubscription = create_subscription<booster_interface::msg::Odometer>("/odometer_state", 1, bind(&Brain::odometerCallback, this, _1));
    lowStateSubscription = create_subscription<booster_interface::msg::LowState>("/low_state", 1, bind(&Brain::lowStateCallback, this, _1));
    imageSubscription = create_subscription<sensor_msgs::msg::Image>("/camera/camera/color/image_raw", 1, bind(&Brain::imageCallback, this, _1));
    depthImageSubscription = create_subscription<sensor_msgs::msg::Image>("/camera/camera/aligned_depth_to_color/image_raw", 1, bind(&Brain::depthImageCallback, this, _1));
    headPoseSubscription = create_subscription<geometry_msgs::msg::Pose>("/head_pose", 1, bind(&Brain::headPoseCallback, this, _1));
    recoveryStateSubscription = create_subscription<booster_interface::msg::RawBytesMsg>("fall_down_recovery_state", 1, bind(&Brain::recoveryStateCallback, this, _1));

}

void Brain::loadConfig()
{
    get_parameter("game.team_id", config->teamId);
    get_parameter("game.player_id", config->playerId);
    get_parameter("game.field_type", config->fieldType);
    get_parameter("game.player_role", config->playerRole);
    get_parameter("game.player_start_pos", config->playerStartPos);
    get_parameter("game.collaboration_role", config->collaborationRole);

    get_parameter("robot.robot_height", config->robotHeight);
    get_parameter("robot.odom_factor", config->robotOdomFactor);
    get_parameter("robot.vx_factor", config->vxFactor);
    get_parameter("robot.yaw_offset", config->yawOffset);

    get_parameter("rerunLog.enable", config->rerunLogEnable);
    get_parameter("rerunLog.server_addr", config->rerunLogServerAddr);
    get_parameter("rerunLog.img_interval", config->rerunLogImgInterval);

    get_parameter("tree_file_path", config->treeFilePath);

    // handle the parameters
    config->handle();

    // debug after handle the parameters
    ostringstream oss;
    config->print(oss);
    prtDebug(oss.str());
}

/**
 * will be called in the Ros2 loop
 */
void Brain::tick()
{
    // Record loop start time
    loopStartTime = get_clock()->now();

    updateMemory();
    updateCollaboration();
    tree->tick();

    // Calculate loop execution time
    auto loopEndTime = get_clock()->now();
    double currentLoopTime = (loopEndTime - loopStartTime).nanoseconds() / 1e6; // Convert to milliseconds

    // Update statistics
    totalLoopTime += currentLoopTime;
    loopCount++;

    if (currentLoopTime > maxLoopTime) {
        maxLoopTime = currentLoopTime;
    }
    if (currentLoopTime < minLoopTime) {
        minLoopTime = currentLoopTime;
    }

    // Output statistics every 1000 loops (approximately every 1-10 seconds depending on loop frequency)
    static int outputCounter = 0;
    outputCounter++;
    if (outputCounter % 1000 == 0) {
        double avgLoopTime = totalLoopTime / loopCount;
        double loopFrequency = 1000.0 / avgLoopTime; // Hz

        prtDebug(format("=== Loop 时间统计 (最近 %d 次循环) ===", loopCount));
        prtDebug(format("当前循环时间: %.2f ms", currentLoopTime));
        prtDebug(format("平均循环时间: %.2f ms", avgLoopTime));
        prtDebug(format("最大循环时间: %.2f ms", maxLoopTime));
        prtDebug(format("最小循环时间: %.2f ms", minLoopTime));
        prtDebug(format("循环频率: %.1f Hz", loopFrequency));
        prtDebug(format("总循环次数: %d", loopCount));

        // Reset statistics for next measurement period
        totalLoopTime = 0.0;
        maxLoopTime = 0.0;
        minLoopTime = std::numeric_limits<double>::max();
        loopCount = 0;
        outputCounter = 0;
    }
}

void Brain::updateCollaboration() {
    // Check if enough time has passed since last collaboration update (100ms interval)
    auto currentTime = get_clock()->now();

    // Initialize the timer on first call
    static bool firstCall = true;
    if (firstCall) {
        lastCollaborationUpdateTime = currentTime;
        firstCall = false;
    }

    // Check if 100ms have passed since last update
    auto timeSinceLastUpdate = (currentTime - lastCollaborationUpdateTime).nanoseconds() / 1e6; // Convert to milliseconds
    if (timeSinceLastUpdate < 1000.0) {
        // Not enough time passed, skip this update
        return;
    }

    // Update the timer
    lastCollaborationUpdateTime = currentTime;

    // Note: Continue collaboration processing even if ball is not detected locally
    if (!data->ballDetected) {
        static int no_ball_counter = 0;
        if (no_ball_counter % 500 == 0) { // 每500次输出一次
            prtDebug(format("协作更新: 自己未检测到球，但继续协作逻辑以维持角色分配 (role: %s)", config->collaborationRole.c_str()));
        }
        no_ball_counter++;
    }

    // Calculate our cost to reach the ball
    calculateBallCost();

    // Debug: 输出当前状态
    static int collab_counter = 0;
    if (collab_counter % 10 == 0) { // 调整为每10次输出一次，因为现在更新频率降低了
        prtDebug(format("协作更新: role='%s', playerId=%d, ballCost=%.2f, possessionPlayerId=%d (100ms timer)",
            config->collaborationRole.c_str(), config->playerId, data->ballCost, data->possessionPlayerId));
    }
    collab_counter++;

    // Process collaboration based on our role
    if (config->collaborationRole == "master") {
        processMasterDecision();
    } else {
        processSlaveUpdates();
    }
}

void Brain::calculateBallCost() {
    Point2D ballPos;
    bool ballPosFound = false;

    // 首先检查自己是否看到球
    if (data->ballDetected) {
        ballPos.x = data->ball.posToField.x;
        ballPos.y = data->ball.posToField.y;
        ballPosFound = true;
        prtDebug("使用自己检测到的球位置计算成本");
    } else {
        // 自己没看到球，尝试使用队友的球信息
        auto startTime = get_clock()->now();
        auto teammatesBallInfo = communication->getTeammateBallInfo();
        auto endTime = get_clock()->now();
        double commTime = (endTime - startTime).nanoseconds() / 1e6; // Convert to milliseconds

        static double totalBallInfoTime = 0.0;
        static int ballInfoCallCount = 0;
        static double maxBallInfoTime = 0.0;
        totalBallInfoTime += commTime;
        ballInfoCallCount++;
        if (commTime > maxBallInfoTime) maxBallInfoTime = commTime;

        if (ballInfoCallCount % 100 == 0) {
            prtDebug(format("getTeammateBallInfo() 时间统计: 当前=%.3fms, 平均=%.3fms, 最大=%.3fms, 队友数=%zu",
                commTime, totalBallInfoTime/ballInfoCallCount, maxBallInfoTime, teammatesBallInfo.size()));
        }

        if (!teammatesBallInfo.empty()) {
            // 找到距离自己最近的球位置
            double minDistance = std::numeric_limits<double>::infinity();
            Point2D closestBallPos;
            int closestTeammateId = -1;

            for (const auto& teammate : teammatesBallInfo) {
                if (teammate.ballDetected) {
                    // 计算球到自己的距离
                    double distance = std::sqrt(
                        std::pow(teammate.ballPosX - data->robotPoseToField.x, 2) +
                        std::pow(teammate.ballPosY - data->robotPoseToField.y, 2)
                    );

                    if (distance < minDistance) {
                        minDistance = distance;
                        closestBallPos.x = teammate.ballPosX;
                        closestBallPos.y = teammate.ballPosY;
                        closestTeammateId = teammate.playerId;
                        ballPosFound = true;
                    }
                }
            }

            if (ballPosFound) {
                ballPos = closestBallPos;
                prtDebug(format("使用队友 %d 检测到的球位置计算成本 (距离: %.2fm)",
                    closestTeammateId, minDistance));
            }
        }
    }

    // 如果所有人都没看到球，返回无穷
    if (!ballPosFound) {
        data->ballCost = std::numeric_limits<double>::infinity();
        prtDebug("所有机器人都没检测到球，成本设为无穷");
        return;
    }

    // 使用选定的球位置计算成本
    Pose2D robotPose = data->robotPoseToField;
    data->ballCost = strategy->calculateCostFunction(robotPose, ballPos);
    data->lastCostCalculation = get_clock()->now();
}

void Brain::processMasterDecision() {
    // Static timer to track how long we've been without valid ball information
    static auto lastValidBallTimeStamp = get_clock()->now();

    // Anti-oscillation threshold for ball cost difference
    constexpr double POSSESSION_COST_DIFF_THRESHOLD = 3; // You can tune this value

    // Collect cost information from all robots (including self)
    std::vector<std::pair<int, double>> robotCosts;
    std::vector<std::pair<int, Pose2D>> robotPoses; // For role assignment

    // Add our own cost and pose
    robotCosts.push_back({config->playerId, data->ballCost});
    robotPoses.push_back({config->playerId, data->robotPoseToField});

    // Add teammates' costs
    auto startTime = get_clock()->now();
    auto teammates = communication->getTeammateCollaborationInfo();
    auto endTime = get_clock()->now();
    double commTime = (endTime - startTime).nanoseconds() / 1e6; // Convert to milliseconds

    static double totalMasterCollabTime = 0.0;
    static int masterCollabCallCount = 0;
    static double maxMasterCollabTime = 0.0;
    totalMasterCollabTime += commTime;
    masterCollabCallCount++;
    if (commTime > maxMasterCollabTime) maxMasterCollabTime = commTime;

    if (masterCollabCallCount % 50 == 0) {
        // prtDebug(format("Master getTeammateCollaborationInfo() 时间统计: 当前=%.3fms, 平均=%.3fms, 最大=%.3fms, 队友数=%zu",
        //     commTime, totalMasterCollabTime/masterCollabCallCount, maxMasterCollabTime, teammates.size()));
    }

    prtDebug(format("Master收集信息: 自己(ID=%d, cost=%.2f), 队友数量=%zu",
        config->playerId, data->ballCost, teammates.size()));

    for (const auto& teammate : teammates) {
        robotCosts.push_back({teammate.playerId, teammate.ballCost});
        robotPoses.push_back({teammate.playerId, {teammate.robotPoseX, teammate.robotPoseY, teammate.robotPoseTheta}});
        // prtDebug(format("队友信息: ID=%d, cost=%.2f, pose=(%.2f,%.2f,%.2f)", 
        //     teammate.playerId, teammate.ballCost, teammate.robotPoseX, teammate.robotPoseY, teammate.robotPoseTheta));
    }

    // Find robot with minimum cost for ball possession
    int bestRobotId = config->playerId;
    double minCost = data->ballCost;

    for (const auto& robotCost : robotCosts) {
        if (robotCost.second < minCost) {
            minCost = robotCost.second;
            bestRobotId = robotCost.first;
        }
    }

    // Check if all costs are infinite (no one can see the ball)
    if (std::isinf(minCost)) {
        // No one can see the ball, clear ball possession but keep role assignments
        int oldPossessionId = data->possessionPlayerId;
        data->possessionPlayerId = -1;
        data->hasBallPossession = false;

        // Keep existing role assignments to maintain collaborative behavior
        // Only clear them if we haven't had valid ball info for a very long time
        auto currentTime = get_clock()->now();
        auto timeSinceValidBall = (currentTime - lastValidBallTimeStamp).seconds();

        // Clear role assignments only after 30 seconds without any ball detection
        if (timeSinceValidBall > 30.0) {
            data->strikerPlayerId = -1;
            data->goalKeeperPlayerId = -1; 
            data->followerPlayerId = -1;
            data->dynamicRole = -1;
            // prtDebug(format("Master决策: 30秒无球信息，清除所有角色分配 (之前球权: %d)", oldPossessionId));
        } else {
            // prtDebug(format("Master决策: 无球信息但保持角色分配，自己角色=%d (之前球权: %d, 无球时间: %.1fs)", 
            //     data->dynamicRole, oldPossessionId, timeSinceValidBall));
        }
    } else {
        // Reset the timer when we have valid ball information
        lastValidBallTimeStamp = get_clock()->now();

        // Anti-oscillation: Only update possession if cost difference is significant
        int oldPossessionId = data->possessionPlayerId;
        double oldPossessionCost = std::numeric_limits<double>::infinity();
        for (const auto& robotCost : robotCosts) {
            if (robotCost.first == oldPossessionId) {
                oldPossessionCost = robotCost.second;
                break;
            }
        }

        bool updatePossession = true;
        if (oldPossessionId != -1 && oldPossessionId != bestRobotId && !std::isinf(oldPossessionCost)) {
            double costDiff = minCost - oldPossessionCost;
            if (costDiff > -POSSESSION_COST_DIFF_THRESHOLD) {
                // The new best is not significantly better, do not update possession
                updatePossession = false;
                prtDebug(format("Master决策: 球权防抖动，bestId=%d, oldId=%d, bestCost=%.2f, oldCost=%.2f, diff=%.2f < 阈值%.2f，保持原球权",
                    bestRobotId, oldPossessionId, minCost, oldPossessionCost, costDiff, POSSESSION_COST_DIFF_THRESHOLD));
            }
        }

        if (updatePossession) {
            data->possessionPlayerId = bestRobotId;
            data->hasBallPossession = (bestRobotId == config->playerId);
        } else {
            // Keep previous possession
            bestRobotId = oldPossessionId;
            minCost = oldPossessionCost;
            data->hasBallPossession = (bestRobotId == config->playerId);
        }

        // Dynamic Role Assignment Logic
        // 1. Robot with ball possession becomes main striker
        data->strikerPlayerId = bestRobotId;

        // 2. Among remaining robots, find the one nearest to our goal (smallest X coordinate)
        // Our goal is at negative X direction (-fieldDimensions.length/2)
        int goalKeeperCandidateId = -1;
        double closestToGoalX = std::numeric_limits<double>::max();

        // 3. Find remaining robot for follower role
        std::vector<int> remainingRobots;

        for (const auto& robotPose : robotPoses) {
            int robotId = robotPose.first;
            if (robotId != bestRobotId) { // Exclude the striker
                remainingRobots.push_back(robotId);

                // Check if this robot is closer to our goal
                double robotX = robotPose.second.x;
                if (robotX < closestToGoalX) {
                    closestToGoalX = robotX;
                    goalKeeperCandidateId = robotId;
                }
            }
        }

        data->goalKeeperPlayerId = goalKeeperCandidateId;

        // The remaining robot becomes follower
        for (int robotId : remainingRobots) {
            if (robotId != goalKeeperCandidateId) {
                data->followerPlayerId = robotId;
                break;
            }
        }

        // Set our own dynamic role
        if (config->playerId == data->strikerPlayerId) {
            data->dynamicRole = 0; // striker
        } else if (config->playerId == data->goalKeeperPlayerId) {
            data->dynamicRole = 1; // goal_keeper
        } else if (config->playerId == data->followerPlayerId) {
            data->dynamicRole = 2; // striker_follower
        } else {
            data->dynamicRole = -1; // unknown
        }

        // // Log decision
        prtDebug(format("Master决策: 球权=%d (cost=%.2f), 角色分配: 主攻=%d, 守门=%d, 跟随=%d, 自己角色=%d", 
            bestRobotId, minCost, data->strikerPlayerId, data->goalKeeperPlayerId, 
            data->followerPlayerId, data->dynamicRole));

        // Log roles to rerun for visualization
        string roleStr = "Unknown";
        if (data->dynamicRole == 0) roleStr = "Striker";
        else if (data->dynamicRole == 1) roleStr = "Goalkeeper";
        else if (data->dynamicRole == 2) roleStr = "Follower";

        log->logToScreen("collaboration/master_role", 
            format("Master Robot %d: Role=%s, Ball Possession=%d, Cost=%.2f", 
                config->playerId, roleStr.c_str(), data->possessionPlayerId, data->ballCost), 
            0x00FF00FF);

        log->logToScreen("collaboration/role_assignments",
            format("Role Assignments: Striker=%d, Goalkeeper=%d, Follower=%d", 
                data->strikerPlayerId, data->goalKeeperPlayerId, data->followerPlayerId),
            0xFFFFFFFF);
    }
}

void Brain::processSlaveUpdates() {
    // Get possession assignment and role assignments from master
    auto startTime = get_clock()->now();
    auto teammates = communication->getTeammateCollaborationInfo();
    auto endTime = get_clock()->now();
    double commTime = (endTime - startTime).nanoseconds() / 1e6; // Convert to milliseconds

    static double totalSlaveCollabTime = 0.0;
    static int slaveCollabCallCount = 0;
    static double maxSlaveCollabTime = 0.0;
    totalSlaveCollabTime += commTime;
    slaveCollabCallCount++;
    if (commTime > maxSlaveCollabTime) maxSlaveCollabTime = commTime;

    if (slaveCollabCallCount % 50 == 0) {
        // prtDebug(format("Slave getTeammateCollaborationInfo() 时间统计: 当前=%.3fms, 平均=%.3fms, 最大=%.3fms, 队友数=%zu",
        //     commTime, totalSlaveCollabTime/slaveCollabCallCount, maxSlaveCollabTime, teammates.size()));
    }

    // prtDebug(format("Slave更新: 收到%zu个队友信息", teammates.size()));

    bool foundMaster = false;
    for (const auto& teammate : teammates) {
        // prtDebug(format("队友信息: ID=%d, masterID=%d, possessionID=%d, 角色分配: 主攻=%d, 守门=%d, 跟随=%d",
        //     teammate.playerId, teammate.masterPlayerId, teammate.possessionPlayerId,
        //     teammate.strikerPlayerId, teammate.goalKeeperPlayerId, teammate.followerPlayerId));

        // Check if this teammate is the master
        if (teammate.masterPlayerId == teammate.playerId) {
            foundMaster = true;
            
            // Update possession status
            int oldPossessionId = data->possessionPlayerId;
            data->possessionPlayerId = teammate.possessionPlayerId;
            data->hasBallPossession = (teammate.possessionPlayerId == config->playerId);
            
            // Update role assignments from master
            data->strikerPlayerId = teammate.strikerPlayerId;
            data->goalKeeperPlayerId = teammate.goalKeeperPlayerId;
            data->followerPlayerId = teammate.followerPlayerId;
            
            // Determine our own dynamic role
            if (config->playerId == data->strikerPlayerId) {
                data->dynamicRole = 0; // striker
            } else if (config->playerId == data->goalKeeperPlayerId) {
                data->dynamicRole = 1; // goal_keeper
            } else if (config->playerId == data->followerPlayerId) {
                data->dynamicRole = 2; // striker_follower
            } else {
                data->dynamicRole = -1; // unknown
            }
            
            prtDebug(format("从Master(ID=%d)收到决策: possession从%d更新为%d, 我的角色=%d",
                teammate.playerId, oldPossessionId, teammate.possessionPlayerId, data->dynamicRole));
                
            // Log slave role to rerun for visualization
            string roleStr = "Unknown";
            if (data->dynamicRole == 0) roleStr = "Striker";
            else if (data->dynamicRole == 1) roleStr = "Goalkeeper";
            else if (data->dynamicRole == 2) roleStr = "Follower";
            
            log->logToScreen("collaboration/slave_role", 
                format("Slave Robot %d: Role=%s, Ball Possession=%d, Cost=%.2f", 
                    config->playerId, roleStr.c_str(), data->possessionPlayerId, data->ballCost), 
                0x0000FFFF);
                
            log->logToScreen("collaboration/received_assignments",
                format("Received Assignments: Striker=%d, Goalkeeper=%d, Follower=%d", 
                    data->strikerPlayerId, data->goalKeeperPlayerId, data->followerPlayerId),
                0xFFFF00FF);
            break;
        }
    }

    if (!foundMaster && teammates.size() > 0) {
        prtDebug("警告: 没有找到Master机器人的信息");
    }

    // Log possession and role status
    static int status_counter = 0;
    if (status_counter % 100 == 0) { // 每100次输出一次状态
        string roleStr = "未知";
        if (data->dynamicRole == 0) roleStr = "主攻";
        else if (data->dynamicRole == 1) roleStr = "守门";
        else if (data->dynamicRole == 2) roleStr = "跟随";
        
        prtDebug(format("当前状态: 角色=%s, 球权机器人=%d, %s", 
            roleStr.c_str(), data->possessionPlayerId,
            data->hasBallPossession ? "我拥有球权" : "我没有球权"));
    }
    status_counter++;
}

void Brain::updateMemory()
{
    // Update current time for behavior tree scripts
    tree->setEntry<double>("current_time", get_clock()->now().seconds());
    
    // Update robot pose for behavior tree scripts
    tree->setEntry<double>("robot_pose_x", data->robotPoseToField.x);
    tree->setEntry<double>("robot_pose_y", data->robotPoseToField.y);
    tree->setEntry<double>("robot_pose_theta", data->robotPoseToField.theta);
    
    // Calculate if robot is closer to forward orientation (90°) than backward (-90°)
    double forward_diff = fabs(data->robotPoseToField.theta - 1.57);  // Distance to 90°
    double backward_diff = fabs(data->robotPoseToField.theta - (-1.57));  // Distance to -90°
    bool is_closer_to_forward = forward_diff < backward_diff;
    tree->setEntry<bool>("is_closer_to_forward_orientation", is_closer_to_forward);
    
    // Calculate if robot is currently facing forward (within ±0.2 radians of 90°)
    bool is_facing_forward = fabs(data->robotPoseToField.theta - 1.57) < 0.2;
    tree->setEntry<bool>("is_facing_forward", is_facing_forward);
    
    // Update collaboration-related blackboard entries
    tree->setEntry<bool>("has_ball_possession", data->hasBallPossession);
    tree->setEntry<int>("possession_player_id", data->possessionPlayerId);
    tree->setEntry<double>("ball_cost", data->ballCost);
    tree->setEntry<bool>("is_master_robot", config->collaborationRole == "master");
    
    // Update dynamic role assignment blackboard entries
    tree->setEntry<int>("dynamic_role", data->dynamicRole);
    tree->setEntry<int>("goal_keeper_player_id", data->goalKeeperPlayerId);
    tree->setEntry<int>("striker_player_id", data->strikerPlayerId);
    tree->setEntry<int>("follower_player_id", data->followerPlayerId);
    tree->setEntry<bool>("is_dynamic_striker", data->dynamicRole == 0);
    tree->setEntry<bool>("is_dynamic_goal_keeper", data->dynamicRole == 1);
    tree->setEntry<bool>("is_dynamic_follower", data->dynamicRole == 2);

    tree->setEntry<bool>("is_at_goalkeeper_position", 
        fabs(data->robotPoseToField.x - (-6.5)) < 2 &&
        fabs(data->robotPoseToField.y - 0.0) < 4);

    updateBallMemory();

    static Point ballPos;
    static rclcpp::Time kickOffTime;
    if (
        tree->getEntry<string>("player_role") == "striker" && ((tree->getEntry<string>("gc_game_state") == "SET" && !tree->getEntry<bool>("gc_is_kickoff_side")) || (tree->getEntry<string>("gc_game_sub_state") == "SET" && !tree->getEntry<bool>("gc_is_sub_state_kickoff_side"))))
    {
        ballPos = data->ball.posToRobot;
        kickOffTime = get_clock()->now();
        tree->setEntry<bool>("wait_for_opponent_kickoff", true);
    }
    else if (tree->getEntry<bool>("wait_for_opponent_kickoff"))
    {
        if (
            norm(data->ball.posToRobot.x - ballPos.x, data->ball.posToRobot.y - ballPos.y) > 0.3 || (get_clock()->now() - kickOffTime).seconds() > 10.0)
        {
            tree->setEntry<bool>("wait_for_opponent_kickoff", false);
        }
    }

    // Log possession and role status periodically
    static int status_counter = 0;
    if (status_counter % 100 == 0) { // Log every 100 ticks
        string roleStr = "Unknown";
        if (data->dynamicRole == 0) roleStr = "Striker";
        else if (data->dynamicRole == 1) roleStr = "Goalkeeper";
        else if (data->dynamicRole == 2) roleStr = "Follower";
        
        log->logToScreen("collaboration/current_status",
            format("Current Status: Role=%s, Ball Owner=%d, %s", 
                roleStr.c_str(), data->possessionPlayerId,
                data->hasBallPossession ? "I have ball possession" : "I don't have ball possession"),
            0x0000FFFF);
    }
    status_counter++;
}


void Brain::updateBallMemory()
{
    // update Pose to field from Pose to robot (based on odom)
    double xfr, yfr, thetafr; // fr = field to robot
    yfr = sin(data->robotPoseToField.theta) * data->robotPoseToField.x - cos(data->robotPoseToField.theta) * data->robotPoseToField.y;
    xfr = -cos(data->robotPoseToField.theta) * data->robotPoseToField.x - sin(data->robotPoseToField.theta) * data->robotPoseToField.y;
    thetafr = -data->robotPoseToField.theta;
    transCoord(
        data->ball.posToField.x, data->ball.posToField.y, 0,
        xfr, yfr, thetafr,
        data->ball.posToRobot.x, data->ball.posToRobot.y, data->ball.posToRobot.z);

    data->ball.range = sqrt(data->ball.posToRobot.x * data->ball.posToRobot.x + data->ball.posToRobot.y * data->ball.posToRobot.y);
    tree->setEntry<double>("ball_range", data->ball.range);
    data->ball.yawToRobot = atan2(data->ball.posToRobot.y, data->ball.posToRobot.x);
    tree->setEntry<double>("ball_yaw_to_robot", data->ball.yawToRobot);
    
    // Set ball position variables for behavior tree scripts
    tree->setEntry<double>("ball_x", data->ball.posToField.x);
    tree->setEntry<double>("ball_y", data->ball.posToField.y);
    
    // Calculate ball tracking range conditions to avoid BehaviorTree parsing errors
    bool ball_within_head_tracking_range = (data->ball.yawToRobot < 0.5 && data->ball.yawToRobot > -0.5);
    tree->setEntry<bool>("ball_within_head_tracking_range", ball_within_head_tracking_range);
    
    bool ball_far_left_or_right = (data->ball.yawToRobot > 0.8 || data->ball.yawToRobot < -0.8);
    tree->setEntry<bool>("ball_far_left_or_right", ball_far_left_or_right);
    data->ball.pitchToRobot = asin(config->robotHeight / data->ball.range);

    // mark ball as lost if long time no see
    if (get_clock()->now().seconds() - data->ball.timePoint.seconds() > config->memoryLength)
    {
        tree->setEntry<bool>("ball_location_known", false);
        data->ballDetected = false;
    }

    // log mem ball pos
    log->setTimeNow();
    log->log("field/memball",
             rerun::LineStrips2D({
                                     rerun::Collection<rerun::Vec2D>{{data->ball.posToField.x - 0.2, -data->ball.posToField.y}, {data->ball.posToField.x + 0.2, -data->ball.posToField.y}},
                                     rerun::Collection<rerun::Vec2D>{{data->ball.posToField.x, -data->ball.posToField.y - 0.2}, {data->ball.posToField.x, -data->ball.posToField.y + 0.2}},
                                 })
                 .with_colors({tree->getEntry<bool>("ball_location_known") ? 0xFFFFFFFF : 0xFF0000FF})
                 .with_radii({0.005})
                 .with_draw_order(30));
}

vector<double> Brain::getGoalPostAngles(const double margin)
{
    double leftX, leftY, rightX, rightY;
    leftX = config->fieldDimensions.length / 2;
    leftY = config->fieldDimensions.goalWidth / 2;
    rightX = config->fieldDimensions.length / 2;
    rightY = -config->fieldDimensions.goalWidth / 2;

    for (int i = 0; i < data->goalposts.size(); i++)
    {
        auto post = data->goalposts[i];
        if (post.info == "oppo-left")
        {
            leftX = post.posToField.x;
            leftY = post.posToField.y;
        }
        else if (post.info == "oppo-right")
        {
            rightX = post.posToField.x;
            rightY = post.posToField.y;
        }
    }

    const double theta_l = atan2(leftY - margin - data->ball.posToField.y, leftX - data->ball.posToField.x);
    const double theta_r = atan2(rightY + margin - data->ball.posToField.y, rightX - data->ball.posToField.x);

    vector<double> vec = {theta_l, theta_r};
    return vec;
}

void Brain::calibrateOdom(double x, double y, double theta)
{
    double x_or, y_or, theta_or; // or = odom to robot
    x_or = -cos(data->robotPoseToOdom.theta) * data->robotPoseToOdom.x - sin(data->robotPoseToOdom.theta) * data->robotPoseToOdom.y;
    y_or = sin(data->robotPoseToOdom.theta) * data->robotPoseToOdom.x - cos(data->robotPoseToOdom.theta) * data->robotPoseToOdom.y;
    theta_or = -data->robotPoseToOdom.theta;

    transCoord(x_or, y_or, theta_or,
               x, y, theta,
               data->odomToField.x, data->odomToField.y, data->odomToField.theta);

    transCoord(
        data->robotPoseToOdom.x, data->robotPoseToOdom.y, data->robotPoseToOdom.theta,
        data->odomToField.x, data->odomToField.y, data->odomToField.theta,
        data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta);

    double placeHolder;
    // ball
    transCoord(
        data->ball.posToRobot.x, data->ball.posToRobot.y, 0,
        data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta,
        data->ball.posToField.x, data->ball.posToField.y, placeHolder);

    // opponents
    for (int i = 0; i < data->opponents.size(); i++)
    {
        auto obj = data->opponents[i];
        transCoord(
            obj.posToRobot.x, obj.posToRobot.y, 0,
            data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta,
            obj.posToField.x, obj.posToField.y, placeHolder);
    }
}

double Brain::msecsSince(rclcpp::Time time)
{
    return (this->get_clock()->now() - time).nanoseconds() / 1e6;
}

bool Brain::executePenaltyPointLocalize()
{
    auto markers = data->getMarkers();

    // Check if we can see a penalty point marker within 5 meters
    FieldMarker penaltyMarker;
    bool foundPenaltyPoint = false;

    // Find a penalty point marker within 5 meters
    for (const auto& marker : markers) {
        if (marker.type == 'P') {
            double distance = sqrt(marker.x * marker.x + marker.y * marker.y);
            if (distance <= 5.0) {
                penaltyMarker = marker;
                foundPenaltyPoint = true;
            }
        }
    }

    if (foundPenaltyPoint) {
        // Calculate robot position based on the penalty point
        auto fd = config->fieldDimensions;

        // Penalty point positions in field coordinates:
        // Right penalty point: (fd.length / 2 - fd.penaltyDist, 0.0)
        // Left penalty point: (-fd.length / 2 + fd.penaltyDist, 0.0)
        double rightPenaltyX = fd.length / 2 - fd.penaltyDist;
        double leftPenaltyX = -fd.length / 2 + fd.penaltyDist;

        // Determine which penalty point we're seeing based on the observed penalty point position
        // Use the robot's current orientation and penalty point relative position to determine which side
        double currentTheta = data->robotPoseToField.theta;
        double observedX = penaltyMarker.x;
        double observedY = penaltyMarker.y;

        // Transform the observed penalty point to field coordinates using rough position estimate
        double roughFieldX = data->robotPoseToField.x + (cos(currentTheta) * observedX - sin(currentTheta) * observedY);

        // Determine if it's right or left penalty point based on the rough field position
        bool isRightPenalty = (roughFieldX > 0);

        // Get the actual penalty point position in field coordinates
        double penaltyFieldX = isRightPenalty ? rightPenaltyX : leftPenaltyX;
        double penaltyFieldY = 0.0;

        // Calculate robot position: penalty_field = robot_pose + R * penalty_robot
        // where R is rotation matrix and penalty_robot is the observed marker position
        double distance = sqrt(observedX * observedX + observedY * observedY);

        // Robot position = penalty_field - R * penalty_robot
        double robotX = penaltyFieldX - (cos(currentTheta) * observedX - sin(currentTheta) * observedY);
        double robotY = penaltyFieldY - (sin(currentTheta) * observedX + cos(currentTheta) * observedY);

        // Use current theta with small adjustment tolerance
        double robotTheta = currentTheta;

        // Direct localization without using particle filter
        calibrateOdom(robotX, robotY, robotTheta);
        tree->setEntry<bool>("odom_calibrated", true);
        data->lastSuccessfulLocalizeTime = get_clock()->now();

        prtDebug("手柄触发罚球点定位成功: " + to_string(robotX) + " " + to_string(robotY) + " " + to_string(rad2deg(robotTheta)) +
                 " penalty: " + (isRightPenalty ? "right" : "left") + " dist: " + to_string(distance));

        return true;
    }
    else {
        prtDebug("手柄触发罚球点定位失败: 未在5米范围内找到罚球点");
        return false;
    }
}

void Brain::joystickCallback(const booster_interface::msg::RemoteControllerState &joy)
{
    prtDebug("joy!!");

    if (!joy.lt && !joy.rt && !joy.lb && !joy.rb)
    {
        if (joy.b)
        {
            tree->setEntry<bool>("B_pressed", true);
            prtDebug("B is pressed");
        }
        else if (!joy.b && tree->getEntry<bool>("B_pressed"))
        {
            tree->setEntry<bool>("B_pressed", false);
            prtDebug("B is released");
        }

        // 添加 START 按键处理 - 测试语音客户端
        if (joy.start)
        {
            prtDebug("START 按键按下 - 测试语音客户端");
            // 让机器人说 "relocate"
            voiceClient->speak("relocate");
            prtDebug("语音客户端测试: 说出 'relocate'");
        }
    }
    else if (joy.lt && !joy.rt && !joy.lb && !joy.rb)
    {
        if (joy.hat_u || joy.hat_d)
        {
            config->vxFactor += 0.01 * (joy.hat_u ? 1.0 : -1.0);
            prtDebug(
                format("vxFactor = %.2f  yawOffset = %.2f", config->vxFactor, config->yawOffset)
            );
        }

        if (joy.hat_l || joy.hat_r)
        {
            config->yawOffset += 0.01 * (joy.hat_r ? 1.0 : -1.0);
            prtDebug(
                format("vxFactor = %.2f  yawOffset = %.2f", config->vxFactor, config->yawOffset)
            );
        }

        if (joy.x)
        {
            tree->setEntry<int>("control_state", 1);
            client->setVelocity(0., 0., 0.);
            client->moveHead(0., 0.);
            prtDebug("State => 1: CANCEL");
        }
        else if (joy.a)
        {
            tree->setEntry<int>("control_state", 2);
            tree->setEntry<bool>("odom_calibrated", false);
            prtDebug("State => 2: RECALIBRATE");
        }
        else if (joy.b)
        {
            tree->setEntry<int>("control_state", 3);
            prtDebug("State => 3: ACTION");
        }
        else if (joy.y)
        {
            string curRole = tree->getEntry<string>("player_role");
            curRole == "striker" ? tree->setEntry<string>("player_role", "goal_keeper") : tree->setEntry<string>("player_role", "striker");
            prtDebug("SWITCH ROLE");
        }
    }
}

void Brain::gameControlCallback(const game_controller_interface::msg::GameControlData &msg)
{
    auto lastGameState = tree->getEntry<string>("gc_game_state");
    vector<string> gameStateMap = {
        "INITIAL", // Initialization state, players are ready outside the field.
        "READY",   // Ready state, players enter the field and walk to their starting positions.
        "SET",     // Stop action, waiting for the referee machine to issue the instruction to start the game.
        "PLAY",    // Normal game.
        "END"      // The game is over.
    };
    string gameState = gameStateMap[static_cast<int>(msg.state)];
    tree->setEntry<string>("gc_game_state", gameState);
    bool isKickOffSide = (msg.kick_off_team == config->teamId);
    tree->setEntry<bool>("gc_is_kickoff_side", isKickOffSide);

    string gameSubStateType = static_cast<int>(msg.secondary_state) == 0 ? "NONE" : "FREE_KICK";
    vector<string> gameSubStateMap = {"STOP", "GET_READY", "SET"};
    string gameSubState = gameSubStateMap[static_cast<int>(msg.secondary_state_info[1])];
    tree->setEntry<string>("gc_game_sub_state_type", gameSubStateType);
    tree->setEntry<string>("gc_game_sub_state", gameSubState);
    bool isSubStateKickOffSide = (static_cast<int>(msg.secondary_state_info[0]) == config->teamId);
    tree->setEntry<bool>("gc_is_sub_state_kickoff_side", isSubStateKickOffSide);

    game_controller_interface::msg::TeamInfo myTeamInfo;
    if (msg.teams[0].team_number == config->teamId)
    {
        myTeamInfo = msg.teams[0];
    }
    else if (msg.teams[1].team_number == config->teamId)
    {
        myTeamInfo = msg.teams[1];
    }
    else
    {

        prtErr("received invalid game controller message");
        return;
    }

    data->penalty[0] = static_cast<int>(myTeamInfo.players[0].penalty);
    data->penalty[1] = static_cast<int>(myTeamInfo.players[1].penalty);
    data->penalty[2] = static_cast<int>(myTeamInfo.players[2].penalty);
    data->penalty[3] = static_cast<int>(myTeamInfo.players[3].penalty);
    double isUnderPenalty = (data->penalty[config->playerId] != 0);
    tree->setEntry<bool>("gc_is_under_penalty", isUnderPenalty);

    int curScore = static_cast<int>(myTeamInfo.score);
    if (curScore > data->lastScore)
    {
        tree->setEntry<bool>("we_just_scored", true);
        data->lastScore = curScore;
    }
    if (gameState == "SET")
    {
        tree->setEntry<bool>("we_just_scored", false);
    }
}

void Brain::detectionsCallback(const vision_interface::msg::Detections &msg)
{
    auto gameObjects = getGameObjects(msg);

    vector<GameObject> balls, goalPosts, persons, robots, obstacles, markings;
    for (int i = 0; i < gameObjects.size(); i++)
    {
        const auto &obj = gameObjects[i];
        if (obj.label == "Ball")
            balls.push_back(obj);
        if (obj.label == "Goalpost")
            goalPosts.push_back(obj);
        if (obj.label == "Person")
        {
            persons.push_back(obj);

            if (tree->getEntry<bool>("treat_person_as_robot"))
                robots.push_back(obj);
        }
        if (obj.label == "Opponent")
            robots.push_back(obj);
        if (obj.label == "LCross" || obj.label == "TCross" || obj.label == "XCross" || obj.label == "PenaltyPoint")
            markings.push_back(obj);
    }

    detectProcessBalls(balls);
    detectProcessMarkings(markings);

    if (!log->isEnabled())
        return;

    // log detection boxes to rerun
    auto detection_time_stamp = msg.header.stamp;
    rclcpp::Time timePoint(detection_time_stamp.sec, detection_time_stamp.nanosec);
    auto now = get_clock()->now();

    map<std::string, rerun::Color> detectColorMap = {
        {"LCross", rerun::Color(0xFFFF00FF)},
        {"TCross", rerun::Color(0x00FF00FF)},
        {"XCross", rerun::Color(0x0000FFFF)},
        {"Person", rerun::Color(0xFF00FFFF)},
        {"Goalpost", rerun::Color(0x00FFFFFF)},
        {"Opponent", rerun::Color(0xFF0000FF)},
    };

    // for logging boundingBoxes
    vector<rerun::Vec2D> mins;
    vector<rerun::Vec2D> sizes;
    vector<rerun::Text> labels;
    vector<rerun::Color> colors;

    // for logging marker points in robot frame
    vector<rerun::Vec2D> points;
    vector<rerun::Vec2D> points_r; // robot frame

    for (int i = 0; i < gameObjects.size(); i++)
    {
        auto obj = gameObjects[i];
        auto label = obj.label;
        labels.push_back(rerun::Text(format("%s x:%.2f y:%.2f c:%.2f", obj.label.c_str(), obj.posToRobot.x, obj.posToRobot.y, obj.confidence)));
        points.push_back(rerun::Vec2D{obj.posToField.x, -obj.posToField.y});
        points_r.push_back(rerun::Vec2D{obj.posToRobot.x, -obj.posToRobot.y});
        mins.push_back(rerun::Vec2D{obj.boundingBox.xmin, obj.boundingBox.ymin});
        sizes.push_back(rerun::Vec2D{obj.boundingBox.xmax - obj.boundingBox.xmin, obj.boundingBox.ymax - obj.boundingBox.ymin});

        auto it = detectColorMap.find(label);
        if (it != detectColorMap.end())
        {
            colors.push_back(detectColorMap[label]);
        }
        else
        {
            colors.push_back(rerun::Color(0xFFFFFFFF));
        }
    }

    double time = msg.header.stamp.sec + static_cast<double>(msg.header.stamp.nanosec) * 1e-9;
    log->setTimeSeconds(time);
    log->log("image/detection_boxes",
             rerun::Boxes2D::from_mins_and_sizes(mins, sizes)
                 .with_labels(labels)
                 .with_colors(colors));

    log->log("field/detection_points",
             rerun::Points2D(points)
                 .with_colors(colors)
             // .with_labels(labels)
    );
    log->log("robotframe/detection_points",
             rerun::Points2D(points_r)
                 .with_colors(colors)
             // .with_labels(labels)
    );
}

void Brain::odometerCallback(const booster_interface::msg::Odometer &msg)
{

    data->robotPoseToOdom.x = msg.x * config->robotOdomFactor;
    data->robotPoseToOdom.y = msg.y * config->robotOdomFactor;
    data->robotPoseToOdom.theta = msg.theta;

    transCoord(
        data->robotPoseToOdom.x, data->robotPoseToOdom.y, data->robotPoseToOdom.theta,
        data->odomToField.x, data->odomToField.y, data->odomToField.theta,
        data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta);

    log->setTimeNow();
    log->log("field/robot",
             rerun::Points2D({{data->robotPoseToField.x, -data->robotPoseToField.y}, {data->robotPoseToField.x + 0.1 * cos(data->robotPoseToField.theta), -data->robotPoseToField.y - 0.1 * sin(data->robotPoseToField.theta)}})
                 .with_radii({0.2, 0.1})
                 .with_colors({0xFF6666FF, 0xFF0000FF}));

    // Draw penalty points on the field
    auto fd = config->fieldDimensions;
    double rightPenaltyX = fd.length / 2 - fd.penaltyDist;
    double leftPenaltyX = -fd.length / 2 + fd.penaltyDist;

    std::vector<rerun::Position2D> penaltyPoints = {
        rerun::Position2D(rightPenaltyX, 0.0f),  // Right penalty point
        rerun::Position2D(leftPenaltyX, 0.0f)    // Left penalty point
    };
    std::vector<rerun::Color> penaltyColors = {
        rerun::Color(255, 0, 0),    // Red for right penalty point
        rerun::Color(0, 0, 255)     // Blue for left penalty point
    };
    std::vector<rerun::components::Text> penaltyLabels = {
        rerun::components::Text("Right Penalty"),
        rerun::components::Text("Left Penalty")
    };

    log->log("field/penalty_points",
        rerun::Points2D(penaltyPoints)
            .with_colors(penaltyColors)
            .with_radii({0.1f, 0.1f})
            .with_labels(penaltyLabels)
            .with_show_labels(true)
    );
}

void Brain::lowStateCallback(const booster_interface::msg::LowState &msg)
{
    data->headYaw = msg.motor_state_serial[0].q;
    data->headPitch = msg.motor_state_serial[1].q;

    log->setTimeNow();

    log->log("low_state_callback/imu/rpy/roll", rerun::Scalar(msg.imu_state.rpy[0]));
    log->log("low_state_callback/imu/rpy/pitch", rerun::Scalar(msg.imu_state.rpy[1]));
    log->log("low_state_callback/imu/rpy/yaw", rerun::Scalar(msg.imu_state.rpy[2]));
    log->log("low_state_callback/imu/acc/x", rerun::Scalar(msg.imu_state.acc[0]));
    log->log("low_state_callback/imu/acc/y", rerun::Scalar(msg.imu_state.acc[1]));
    log->log("low_state_callback/imu/acc/z", rerun::Scalar(msg.imu_state.acc[2]));
    log->log("low_state_callback/imu/gyro/x", rerun::Scalar(msg.imu_state.gyro[0]));
    log->log("low_state_callback/imu/gyro/y", rerun::Scalar(msg.imu_state.gyro[1]));
    log->log("low_state_callback/imu/gyro/z", rerun::Scalar(msg.imu_state.gyro[2]));
}

void Brain::imageCallback(const sensor_msgs::msg::Image &msg)
{
    if (!config->rerunLogEnable)
        return;

    static int counter = 0;
    counter++;
    if (counter % config->rerunLogImgInterval == 0)
    {

        cv::Mat imageBGR(msg.height, msg.width, CV_8UC3, const_cast<uint8_t *>(msg.data.data()));
        cv::Mat imageRGB;
        cv::cvtColor(imageBGR, imageRGB, cv::COLOR_BGR2RGB);

        std::vector<uint8_t> compressed_image;
        std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 10};
        cv::imencode(".jpg", imageRGB, compressed_image, compression_params);

        double time = msg.header.stamp.sec + static_cast<double>(msg.header.stamp.nanosec) * 1e-9;
        log->setTimeSeconds(time);
        log->log("image/img", rerun::EncodedImage::from_bytes(compressed_image));
    }
}

void Brain::depthImageCallback(const sensor_msgs::msg::Image &msg)
{
    if (!config->rerunLogEnable)
        return;

    static int counter = 0;
    counter++;
    if (counter % config->rerunLogImgInterval == 0)
    {
        // 处理深度图像
        cv::Mat depthImage;

        // 检查图像编码格式
        if (msg.encoding == "16UC1" || msg.encoding == "mono16")
        {
            // 16位单通道深度图像
            depthImage = cv::Mat(msg.height, msg.width, CV_16UC1, const_cast<uint8_t *>(msg.data.data()));
        }
        else if (msg.encoding == "32FC1")
        {
            // 32位浮点深度图像
            depthImage = cv::Mat(msg.height, msg.width, CV_32FC1, const_cast<uint8_t *>(msg.data.data()));
        }
        else
        {
            // 不支持的编码格式，直接返回
            return;
        }

        // 将深度图像转换为8位灰度图像用于显示
        cv::Mat depthImage8bit;
        if (depthImage.type() == CV_16UC1)
        {
            // 16位深度图像转换为8位，进行归一化
            double minVal, maxVal;
            cv::minMaxLoc(depthImage, &minVal, &maxVal);
            depthImage.convertTo(depthImage8bit, CV_8UC1, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
            // 翻转灰度值：近的地方白色，远的地方黑色
            depthImage8bit = 255 - depthImage8bit;
        }
        else if (depthImage.type() == CV_32FC1)
        {
            // 32位浮点深度图像转换为8位
            double minVal, maxVal;
            cv::minMaxLoc(depthImage, &minVal, &maxVal);
            depthImage.convertTo(depthImage8bit, CV_8UC1, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
            // 翻转灰度值：近的地方白色，远的地方黑色
            depthImage8bit = 255 - depthImage8bit;
        }

        // 将单通道灰度图像转换为RGB格式
        cv::Mat depthImageRGB;
        cv::cvtColor(depthImage8bit, depthImageRGB, cv::COLOR_GRAY2RGB);

        // 压缩图像
        std::vector<uint8_t> compressed_depth_image;
        std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 10};
        cv::imencode(".jpg", depthImageRGB, compressed_depth_image, compression_params);

        // 发送到rerun
        double time = msg.header.stamp.sec + static_cast<double>(msg.header.stamp.nanosec) * 1e-9;

        //TODO: Change back if you want to show the depth image
        // log->setTimeSeconds(time);
        // log->log("image/depth", rerun::EncodedImage::from_bytes(compressed_depth_image));
    }
}

void Brain::headPoseCallback(const geometry_msgs::msg::Pose &msg)
{

    // --- for test:
    // if (config->rerunLogEnable) {
    if (false)
    {
        auto x = msg.position.x;
        auto y = msg.position.y;
        auto z = msg.position.z;

        auto orientation = msg.orientation;

        auto roll = rad2deg(atan2(2 * (orientation.w * orientation.x + orientation.y * orientation.z), 1 - 2 * (orientation.x * orientation.x + orientation.y * orientation.y)));
        auto pitch = rad2deg(asin(2 * (orientation.w * orientation.y - orientation.z * orientation.x)));
        auto yaw = rad2deg(atan2(2 * (orientation.w * orientation.z + orientation.x * orientation.y), 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)));

        log->setTimeNow();

        log->log("head_to_base/text",
                 rerun::TextLog("x: " + to_string(x) + " y: " + to_string(y) + " z: " + to_string(z) + " roll: " + to_string(roll) + " pitch: " + to_string(pitch) + " yaw: " + to_string(yaw)));
        log->log("head_to_base/x",
                 rerun::Scalar(x));
        log->log("head_to_base/y",
                 rerun::Scalar(y));
        log->log("head_to_base/z",
                 rerun::Scalar(z));
        log->log("head_to_base/roll",
                 rerun::Scalar(roll));
        log->log("head_to_base/pitch",
                 rerun::Scalar(pitch));
        log->log("head_to_base/yaw",
                 rerun::Scalar(yaw));
    }
}

void Brain::recoveryStateCallback(const booster_interface::msg::RawBytesMsg &msg)
{
    // uint8_t state; // IS_READY = 0, IS_FALLING = 1, HAS_FALLEN = 2, IS_GETTING_UP = 3,
    // uint8_t is_recovery_available; // 1 for available, 0 for not available
    // 使用 RobotRecoveryState 结构，将msg里面的msg转换为RobotRecoveryState
    try
    {
        const std::vector<unsigned char>& buffer = msg.msg;
        RobotRecoveryStateData recoveryState;
        memcpy(&recoveryState, buffer.data(), buffer.size());

        vector<RobotRecoveryState> recoveryStateMap = {
            RobotRecoveryState::IS_READY,
            RobotRecoveryState::IS_FALLING,
            RobotRecoveryState::HAS_FALLEN,
            RobotRecoveryState::IS_GETTING_UP
        };
        this->data->recoveryState = recoveryStateMap[static_cast<int>(recoveryState.state)];
        this->data->isRecoveryAvailable = static_cast<bool>(recoveryState.is_recovery_available);
        this->data->currentRobotModeIndex = static_cast<int>(recoveryState.current_planner_index);

        // cout << "recoveryState: " << static_cast<int>(recoveryState.state) << endl;
        // cout << "recovery is available: " << static_cast<int>(recoveryState.is_recovery_available) << endl;
        // cout << "current planner idx: " << static_cast<int>(recoveryState.current_planner_index) << endl;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}


vector<GameObject> Brain::getGameObjects(const vision_interface::msg::Detections &detections)
{
    vector<GameObject> res;

    auto timestamp = detections.header.stamp;

    rclcpp::Time timePoint(timestamp.sec, timestamp.nanosec);

    for (int i = 0; i < detections.detected_objects.size(); i++)
    {
        auto obj = detections.detected_objects[i];
        GameObject gObj;

        gObj.timePoint = timePoint;
        gObj.label = obj.label;

        gObj.boundingBox.xmax = obj.xmax;
        gObj.boundingBox.xmin = obj.xmin;
        gObj.boundingBox.ymax = obj.ymax;
        gObj.boundingBox.ymin = obj.ymin;
        gObj.confidence = obj.confidence;

        if (obj.position.size() > 0 && !(obj.position[0] == 0 && obj.position[1] == 0))
        {
            gObj.posToRobot.x = obj.position[0];
            gObj.posToRobot.y = obj.position[1];
        }
        else
        {
            gObj.posToRobot.x = obj.position_projection[0];
            gObj.posToRobot.y = obj.position_projection[1];
        }

        gObj.range = norm(gObj.posToRobot.x, gObj.posToRobot.y);
        gObj.yawToRobot = atan2(gObj.posToRobot.y, gObj.posToRobot.x);
        gObj.pitchToRobot = atan2(config->robotHeight, gObj.range);

        transCoord(
            gObj.posToRobot.x, gObj.posToRobot.y, 0,
            data->robotPoseToField.x, data->robotPoseToField.y, data->robotPoseToField.theta,
            gObj.posToField.x, gObj.posToField.y, gObj.posToField.z);

        res.push_back(gObj);
    }

    return res;
}

void Brain::detectProcessBalls(const vector<GameObject> &ballObjs)
{
    // Parameters
    const double confidenceValve = 0.35;        // If the confidence is lower than this threshold, it is considered not a ball (note that the target confidence passed in by the detection module is currently all > 0.2).
    const double pitchLimit = deg2rad(0);       // When the pitch of the ball relative to the front of the robot (downward is positive) is lower than this value, it is considered not a ball. (Because the ball won't be in the sky.)
    const int timeCountThreshold = 5;           // Only when the ball is detected in consecutive several frames is it considered a ball. This is only used in the ball-finding strategy.
    const unsigned int detectCntThreshold = 3;  // The maximum count. Only when the target is detected in such a number of frames is it considered that the target is truly identified. (Currently only used for ball detection.)
    const unsigned int diffConfidThreshold = 4; // The threshold for the difference times between the tracked ball and the high-confidence ball. After reaching this threshold, the high-confidence ball will be adopted.

    double bestConfidence = 0;
    double minPixDistance = 1.e4;
    int indexRealBall = -1;  // Which ball is considered to be the real one. -1 indicates that no ball has been detected.
    int indexTraceBall = -1; // Track the ball according to the pixel distance. -1 indicates that no target has been tracked.

    // Find the most likely real ball.
    for (int i = 0; i < ballObjs.size(); i++)
    {
        auto ballObj = ballObjs[i];

        // Judgment: If the confidence is too low, it is considered a false detection.
        if (ballObj.confidence < confidenceValve)
            continue;

        // Prevent the lights in the sky from being recognized as balls.
        if (ballObj.posToRobot.x < -0.5 || ballObj.posToRobot.x > 10.0)
            continue;

        // Find the one with the highest confidence among the remaining balls.
        if (ballObj.confidence > bestConfidence)
        {
            bestConfidence = ballObj.confidence;
            indexRealBall = i;
        }
    }

    if (indexRealBall >= 0)
    {
        data->ballDetected = true;

        data->ball = ballObjs[indexRealBall];

        tree->setEntry<bool>("ball_location_known", true);
    }
    else
    {
        data->ballDetected = false;
        data->ball.boundingBox.xmin = 0;
        data->ball.boundingBox.xmax = 0;
        data->ball.boundingBox.ymin = 0;
        data->ball.boundingBox.ymax = 0;
        data->ball.confidence = 0;
    }

    data->robotBallAngleToField = atan2(data->ball.posToField.y - data->robotPoseToField.y, data->ball.posToField.x - data->robotPoseToField.x);
}

void Brain::detectProcessMarkings(const vector<GameObject> &markingObjs)
{
    const double confidenceValve = 0.1;

    data->markings.clear();

    for (int i = 0; i < markingObjs.size(); i++)
    {
        auto marking = markingObjs[i];

        if (marking.confidence < confidenceValve)
            continue;

        if (marking.posToRobot.x < -0.5 || marking.posToRobot.x > 10.0)
            continue;

        data->markings.push_back(marking);
    }
}
