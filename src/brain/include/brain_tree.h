#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

#include "types.h"

class Brain;

using namespace std;
using namespace BT;

class BrainTree
{
public:
    BrainTree(Brain *argBrain) : brain(argBrain) {}

    void init();

    void tick();

    // get entry on blackboard
    template <typename T>
    inline T getEntry(const string &key)
    {
        T value = T();
        [[maybe_unused]] auto res = tree.rootBlackboard()->get<T>(key, value);
        return value;
    }

    // set entry on blackboard
    template <typename T>
    inline void setEntry(const string &key, const T &value)
    {
        tree.rootBlackboard()->set<T>(key, value);
    }

private:
    Tree tree;
    Brain *brain;

    /**
     * Initialize the entries in the blackboard.
     * Note: For newly added fields, set a default value here.
     */

    void initEntry();
};

class StrikerDecide : public SyncActionNode
{
public:
    StrikerDecide(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("chase_threshold", 1.0, "Perform the chasing action if the distance exceeds this threshold"),
            InputPort<double>("kick_range_threshold", 0.8, "Ball must be within this range (in meters) to enter kick mode"),
            InputPort<double>("dribble_range_threshold", 0.3, "Ball must be within this range and angle threshold to enter chasetotarget mode"),
            InputPort<string>("decision_in", "", "Used to read the last decision"),
            InputPort<string>("position", "offense", "offense | defense, determines the direction to kick the ball"),
            OutputPort<string>("decision_out"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};

class GoalieDecide : public SyncActionNode
{
public:
    GoalieDecide(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("chase_threshold", 1.0, "Perform the chasing action if the distance exceeds this threshold"),
            InputPort<double>("kick_range_threshold", 0.6, "Ball must be within this range (in meters) to enter kick mode"),
            InputPort<double>("dribble_range_threshold", 0.3, "Ball must be within this range and angle threshold to enter chasetotarget mode"),
            InputPort<double>("adjust_angle_tolerance", 0.1, "Consider the adjustment successful if the angle is smaller than this value"),
            InputPort<double>("adjust_y_tolerance", 0.1, "Consider the y-direction adjustment successful if the offset is smaller than this value"),
            InputPort<string>("decision_in", "", "Used to read the last decision"),
            OutputPort<string>("decision_out"),
        };
    }

    BT::NodeStatus tick() override;

private:
    Brain *brain;
};

class CamTrackBall : public SyncActionNode
{
public:
    CamTrackBall(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {};
    }
    NodeStatus tick() override;

private:
    Brain *brain;
};

class CamFindBall : public SyncActionNode
{
public:
    CamFindBall(const string &name, const NodeConfig &config, Brain *_brain);

    NodeStatus tick() override;

private:
    double _cmdSequence[6][2];    // The sequence of actions for finding the ball, in which the robot looks towards these positions in order.
    rclcpp::Time _timeLastCmd;    // The time of the last command execution, used to ensure there is a time interval between commands.
    int _cmdIndex;                // The current step in the cmdSequence that is being executed.
    long _cmdIntervalMSec;        // The time interval (in milliseconds) between executing actions in the sequence.
    long _cmdRestartIntervalMSec; // If the time since the last execution exceeds this value, the sequence will restart from step 0.

    Brain *brain;
};

// The robot performs the action of finding the ball, which needs to be used in conjunction with CamFindBall.
class RobotFindBall : public StatefulActionNode
{
public:
    RobotFindBall(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("vyaw_limit", 1.0, "yaw limit"),
        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    void onHalted() override;

private:
    double _turnDir; // 1.0 left -1.0 right
    Brain *brain;
};

// Chasing the ball: If the ball is behind the robot, it will move around to the back of the ball.
class Chase : public SyncActionNode
{
public:
    Chase(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("vx_limit", 0.4, "Maximum x velocity for chasing the ball"),
            InputPort<double>("vy_limit", 0.4, "Maximum y velocity for chasing the ball"),
            InputPort<double>("vtheta_limit", 0.1, "Maximum angular velocity for real-time direction adjustment while chasing the ball"),
            InputPort<double>("dist", 1.0, "The target distance behind the ball for chasing it"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
    string _state;     // circl_back, chase;
    double _dir = 1.0; // 1.0 circle back from left, -1.0  circle back from right
};

// Chasing the ball with target direction: Like Chase but steers towards a target position
class ChaseToTarget : public SyncActionNode
{
public:
    ChaseToTarget(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) 
    {
        // Initialize dribbling state variables - will be set on first use with proper clock
        _lastDribbleKickTime = rclcpp::Time(0LL, RCL_CLOCK_UNINITIALIZED);
        _dribbleKickStartTime = rclcpp::Time(0LL, RCL_CLOCK_UNINITIALIZED);
    }

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("vx_limit", 0.4, "Maximum x velocity for chasing the ball"),
            InputPort<double>("vy_limit", 0.4, "Maximum y velocity for chasing the ball"),
            InputPort<double>("vtheta_limit", 0.1, "Maximum angular velocity for real-time direction adjustment while chasing the ball"),
            InputPort<double>("dist", 1.0, "The target distance behind the ball for chasing it"),
            InputPort<double>("target_x", 0.0, "Target x position to steer the ball towards"),
            InputPort<double>("target_y", 0.0, "Target y position to steer the ball towards"),
            InputPort<double>("turn_factor", 0.3, "How aggressively to turn towards target (0.0-1.0)"),
            
            // Dribbling parameters
            InputPort<bool>("enable_dribbling", false, "Enable dribbling behavior"),
            InputPort<double>("dribble_distance", 0.25, "Distance to ball when dribbling kicks are triggered"),
            InputPort<double>("dribble_kick_power", 0.3, "Power of dribbling kicks (0.0-1.0)"),
            InputPort<int>("dribble_kick_duration", 200, "Duration of dribbling kick in milliseconds"),
            InputPort<int>("dribble_cooldown", 500, "Cooldown between dribble kicks in milliseconds"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
    string _state;     // chase, circle_back, dribble_kick, dribble_cooldown
    double _dir = 1.0; // 1.0 circle back from left, -1.0  circle back from right
    
    // Dribbling state variables
    rclcpp::Time _lastDribbleKickTime;
    rclcpp::Time _dribbleKickStartTime;
    bool _isDribbleKicking = false;
};

// After approaching the ball, adjust to the appropriate kicking angle for offense or defense.
class Adjust : public SyncActionNode
{
public:
    Adjust(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("turn_threshold", 0.2, "If the angle to the ball exceeds this value, the robot will first turn to face the ball"),
            InputPort<double>("vx_limit", 0.1, "Limit for vx during adjustment, [-limit, limit]"),
            InputPort<double>("vy_limit", 0.1, "Limit for vy during adjustment, [-limit, limit]"),
            InputPort<double>("vtheta_limit", 0.4, "Limit for vtheta during adjustment, [-limit, limit]"),
            InputPort<double>("max_range", 1.5, "When the ball range exceeds this value, move slightly forward"),
            InputPort<double>("min_range", 1.0, "When the ball range is smaller than this value, move slightly backward"),
            InputPort<string>("position", "offense", "offense | defense, determines which direction to kick the ball"),
            InputPort<double>("near_threshold", 0.3, "Threshold for near distance calculations"),
            InputPort<double>("tangential_speed_far", 0.4, "Tangential speed when far from target"),
            InputPort<double>("tangential_speed_near", 0.2, "Tangential speed when near target"),
            InputPort<double>("vtheta_factor", 1.0, "Factor for angular velocity adjustment"),
            InputPort<double>("range", 1.0, "Target range to maintain"),
            InputPort<double>("no_turn_threshold", 0.1, "Threshold below which no turning is applied"),
            InputPort<double>("turn_first_threshold", 0.5, "Threshold above which robot turns first before moving"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};

// Tangential adjustment: robot moves along tangent to the circular path around ball for better forward/backward motion efficiency
class TangentialAdjust : public SyncActionNode
{
public:
    TangentialAdjust(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("turn_threshold", 0.2, "If the angle to the ball exceeds this value, the robot will first turn to face the ball"),
            InputPort<double>("vx_limit", 0.6, "Limit for vx during adjustment, [-limit, limit]"),
            InputPort<double>("vy_limit", 0.3, "Limit for vy during adjustment, [-limit, limit]"),
            InputPort<double>("vtheta_limit", 0.8, "Limit for vtheta during adjustment, [-limit, limit]"),
            InputPort<double>("max_range", 1.5, "When the ball range exceeds this value, move slightly forward"),
            InputPort<double>("min_range", 1.0, "When the ball range is smaller than this value, move slightly backward"),
            InputPort<string>("position", "offense", "offense | defense, determines which direction to kick the ball"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};

class Kick : public StatefulActionNode
{
public:
    Kick(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<int>("min_msec_kick", 500, "The minimum duration (in milliseconds) for executing a kick action"),
            InputPort<int>("msec_stand", 500, "The number of milliseconds after issuing a stop command"),
            InputPort<double>("vx_limit", 1.2, "vx limit"),
            InputPort<double>("vy_limit", 0.4, "vy limit"),
        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    // callback to execute if the action was aborted by another node
    void onHalted() override;

private:
    Brain *brain;
    rclcpp::Time _startTime;
    int _msecKick = 1000;
};

// A full sweep of the field of view involves first tilting the head upwards in one direction,
// and then lowering it to sweep in another direction, completing one full circle.
class CamScanField : public SyncActionNode
{
public:
    CamScanField(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("low_pitch", 0.4, "The minimum pitch when looking upwards"),
            InputPort<double>("high_pitch", 0.2, "The minimum pitch when looking upwards"),
            InputPort<double>("left_yaw", 0.8, "The maximum yaw when looking to the left"),
            InputPort<double>("right_yaw", -0.8, " The minimum yaw when looking to the right"),
            InputPort<int>("msec_cycle", 4000, "How many milliseconds it takes to complete one full rotation"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};

// SelfLocate: Uses particle filtering to correct the current position, compensating for odometry drift.
class SelfLocate : public SyncActionNode
{
public:
    SelfLocate(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;

    static PortsList providedPorts()
    {
        return {
            // enter_field: Used when entering the field, at this point, the robot is definitely in its own half, and the direction can be further narrowed based on the position of its own goal.
            // trust_direction: Used in normal conditions, where the odom information is generally accurate (the robot has not fallen over).
            // trust_position: Used after the robot has fallen. At this point, x and y are reliable, but the direction is not. (Note: if near the midfield line, the position should also be considered unreliable due to the symmetry of the field).
            // trust_nothing: An extreme case where neither x nor y is reliable, and the robot needs to identify its direction using landmarks.
            // face_forward: The direction facing the opponent's goal, primarily used for testing.
            InputPort<string>("mode", "enter_field", "must be one of [enter_field, trust_direction, trust_position, trust_nothing, face_forward]"),
        };
    };

private:
    Brain *brain;
};

// Move to a Pose in the Field coordinate system, including the final target orientation.
// It is recommended to use this together with CamScanField and SelfLocate for a more accurate final position.
class MoveToPoseOnField : public SyncActionNode
{
public:
    MoveToPoseOnField(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("x", 0, "Target x-coordinate in the Field coordinate system"),
            InputPort<double>("y", 0, "Target y-coordinate in the Field coordinate system"),
            InputPort<double>("theta", 0, "Final orientation of the target in the Field coordinate system"),
            InputPort<double>("long_range_threshold", 1.5, "When the distance to the target point exceeds this value, prioritize moving towards it rather than fine-tuning position and orientation"),
            InputPort<double>("turn_threshold", 0.4, "For long distances, if the angle to the target point exceeds this threshold, turn towards the target point first"),
            InputPort<double>("vx_limit", 1.0, "x limit"),
            InputPort<double>("vy_limit", 0.5, "y limit"),
            InputPort<double>("vtheta_limit", 0.4, "theta limit"),
            InputPort<double>("x_tolerance", 0.2, "X tolerance"),
            InputPort<double>("y_tolerance", 0.2, "y tolerance"),
            InputPort<double>("theta_tolerance", 0.1, "theta tolerance"),
        };
    }

    BT::NodeStatus tick() override;

private:
    Brain *brain;
};

// Conditional stand up
class CheckAndStandUp : public SyncActionNode
{
public:
CheckAndStandUp(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts() {
        return {};
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};

// Rotation for relocation after standing up
class RotateForRelocate : public StatefulActionNode
{
public:
    RotateForRelocate(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("vyaw_limit", 1.0, "Upper limit of turning speed"),
            InputPort<int>("max_msec_locate", 5000, "Maximum relocation time"),
        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    void onHalted() override;

private:
    Brain *brain;
    rclcpp::Time _lastSuccessfulLocalizeTime;
    rclcpp::Time _startTime;
};


/**
 * @brief Set the robot's velocity.
 *
 * @param x, y, theta double, the robot's velocity in the x and y directions (m/s) and angular velocity (rad/s) for counterclockwise rotation.
 * Default values are 0. If all values are 0, it is equivalent to issuing a command to make the robot stand still.
 */

class SetVelocity : public SyncActionNode
{
public:
    SetVelocity(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;
    static PortsList providedPorts()
    {
        return {
            InputPort<double>("x", 0, "Default x is 0"),
            InputPort<double>("y", 0, "Default y is 0"),
            InputPort<double>("theta", 0, "Default  theta is 0"),
        };
    }

private:
    Brain *brain;
};

class WaveHand : public SyncActionNode
{
public:
    WaveHand(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain)
    {
    }

    NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<string>("action", "start", "start | stop"),
        };
    }

private:
    Brain *brain;
};

class GoBackInField : public StatefulActionNode
{
public:
    GoBackInField(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("valve", 0.5, "How far from boundary to stop when returning to field"),
        };
    }

    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

private:
    Brain *brain;
    bool _isMovingBack;
    double _targetDirection;
    rclcpp::Time _startTime;
    static constexpr int _timeoutMs = 10000; // 10 second timeout
};

class TurnOnSpot : public StatefulActionNode
{
public:
    TurnOnSpot(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("rad", 0, "How many radians to turn, positive for left"),
            InputPort<bool>("towards_ball", false, "When true, ignore rad sign and turn towards last seen ball direction")
        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    void onHalted() override {};

private:
    double _lastAngle; // Angle from previous tick
    double _angle; // How many radians to turn
    double _cumAngle; // Total angle turned
    double _msecLimit = 5000;  // Maximum execution time in milliseconds (prevent deadlock)
    rclcpp::Time _timeStart; // Time when entering the node
    Brain *brain;
};

// Position-based turn node that turns by a relative angle (no visual feedback)
class TurnByAngle : public StatefulActionNode
{
public:
    TurnByAngle(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("rad", 0, "How many radians to turn, positive for left"),
            InputPort<double>("angular_velocity", 1.0, "Angular velocity in rad/s for turning"),
            InputPort<double>("tolerance", 0.05, "Angle tolerance in radians for completion"),
            InputPort<int>("timeout_ms", 5000, "Maximum execution time in milliseconds")
        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    void onHalted() override;

private:
    double _startAngle;       // Starting angle from odometry
    double _targetAngle;      // Target angle to reach
    double _turnAngle;        // How much to turn (input)
    double _angularVelocity;  // Angular velocity to use
    double _tolerance;        // Completion tolerance
    int _timeoutMs;           // Timeout in milliseconds
    rclcpp::Time _startTime;  // Start time
    Brain *brain;
};

// Position-based turn node that turns to an absolute angle in field coordinates
class TurnToAngle : public StatefulActionNode
{
public:
    TurnToAngle(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("target_angle", 0, "Target angle in field coordinates (radians)"),
            InputPort<double>("angular_velocity", 1.0, "Angular velocity in rad/s for turning"),
            InputPort<double>("tolerance", 0.05, "Angle tolerance in radians for completion"),
            InputPort<int>("timeout_ms", 5000, "Maximum execution time in milliseconds")
        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    void onHalted() override;

private:
    double _targetAngle;      // Target angle in field coordinates
    double _angularVelocity;  // Angular velocity to use
    double _tolerance;        // Completion tolerance
    int _timeoutMs;           // Timeout in milliseconds
    rclcpp::Time _startTime;  // Start time
    Brain *brain;
};


// ------------------------------- FOR GOALKEEPER -------------------------------

// GoalKeeper Position: Maintain goalkeeper position at goal center with 90-degree orientation
class GoalKeeperPosition : public StatefulActionNode
{
public:
    GoalKeeperPosition(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("base_x", -6.5, "Base X position for goalkeeper (goal center)"),
            InputPort<double>("base_y", 0.0, "Base Y position for goalkeeper"),
            InputPort<double>("base_theta", 1.57, "Base orientation (90 degrees)"),
            InputPort<double>("y_adjustment_factor", 0.5, "Factor for Y adjustment based on ball position"),
            InputPort<double>("max_y_offset", 1.0, "Maximum Y offset from base position"),
            InputPort<double>("vx_limit", 0.3, "X velocity limit"),
            InputPort<double>("vy_limit", 0.3, "Y velocity limit"),
            InputPort<double>("vtheta_limit", 0.5, "Angular velocity limit"),
            InputPort<double>("position_tolerance", 0.2, "Position tolerance for considering target reached"),
            InputPort<double>("angle_tolerance", 0.15, "Angle tolerance for considering target reached"),
        };
    }

    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

private:
    Brain *brain;
    double _targetX, _targetY, _targetTheta;
    double _baseX, _baseY, _baseTheta;
    double _yAdjustmentFactor, _maxYOffset;
    double _vxLimit, _vyLimit, _vthetaLimit;
    double _positionTolerance, _angleTolerance;
};

// GoalKeeper Intercept: Actively intercept the ball by moving forward when it's close
class GoalKeeperIntercept : public StatefulActionNode
{
public:
    GoalKeeperIntercept(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("intercept_distance", 2.0, "Distance to start intercepting"),
            InputPort<double>("prediction_time", 1.0, "Time to predict ball movement"),
            InputPort<double>("vx_limit", 0.6, "X velocity limit for interception"),
            InputPort<double>("vy_limit", 0.4, "Y velocity limit for interception"),
            InputPort<double>("vtheta_limit", 1.0, "Angular velocity limit"),
        };
    }

    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

private:
    Brain *brain;
    double _interceptX, _interceptY;
    bool _hasValidIntercept;
    rclcpp::Time _startTime;

    // Three-phase interception
    enum Phase { TURN_TO_BALL, MOVE_TO_BALL, MOVE_FORWARD_TO_INTERCEPT };
    Phase _phase;
    double _targetAngle;  // Angle to face the ball
};

// GoalKeeper Track and Adjust: Track ball and adjust position accordingly
class GoalKeeperTrackAndAdjust : public StatefulActionNode
{
public:
    GoalKeeperTrackAndAdjust(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("base_x", -6.5, "Base X position for goalkeeper"),
            InputPort<double>("adjustment_speed", 0.2, "Speed of position adjustment"),
            InputPort<double>("ball_y_factor", 0.3, "Factor for Y adjustment based on ball Y position"),
            InputPort<double>("max_adjustment", 0.8, "Maximum adjustment from base position"),
            InputPort<double>("vx_limit", 0.2, "X velocity limit"),
            InputPort<double>("vy_limit", 0.2, "Y velocity limit"),
            InputPort<double>("vtheta_limit", 0.3, "Angular velocity limit"),
            InputPort<double>("position_tolerance", 0.15, "Position tolerance for smooth adjustment"),
        };
    }

    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

private:
    Brain *brain;
    double _baseX, _adjustmentSpeed, _ballYFactor, _maxAdjustment;
    double _vxLimit, _vyLimit, _vthetaLimit, _positionTolerance;
};

// GoalKeeper Y-Axis Defense: Always orient along Y-axis, predict ball direction, and react accordingly
class GoalKeeperYAxisDefense : public StatefulActionNode
{
public:
    GoalKeeperYAxisDefense(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("base_x", -6.5, "Base X position for goalkeeper"),
            InputPort<double>("base_y", 0.0, "Base Y position for goalkeeper"),
            InputPort<double>("base_theta", 1.57, "Base orientation (Y-axis aligned)"),
            InputPort<double>("prediction_distance", 1.0, "Distance to start ball direction prediction"),
            InputPort<double>("reaction_speed", 0.5, "Speed of forward/backward reaction"),
            InputPort<double>("vx_limit", 0.6, "X velocity limit for forward/backward movement"),
            InputPort<double>("vy_limit", 0.4, "Y velocity limit for lateral adjustment"),
            InputPort<double>("vtheta_limit", 1.0, "Angular velocity limit"),
        };
    }

    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

private:
    Brain *brain;
    double _baseX, _baseY, _baseTheta;
    double _predictionDistance, _reactionSpeed;
    double _vxLimit, _vyLimit, _vthetaLimit;
    
    // Ball direction prediction
    enum BallDirection { BALL_UNKNOWN, BALL_MOVING_FORWARD, BALL_MOVING_BACKWARD, BALL_STATIONARY };
    BallDirection _ballDirection;
    double _lastBallX, _lastBallY;
    rclcpp::Time _lastBallTime;
    
    // Phases of defense
    enum DefensePhase { ORIENT_Y_AXIS, PREDICT_AND_REACT, INTERCEPT };
    DefensePhase _phase;
    
    BallDirection predictBallDirection();
    void executeReaction();
};

// ------------------------------- FOR DEMO -------------------------------

// This node is for demonstrating chasing the ball and is not used during actual gameplay.
// The difference from Chase is that Simple Chase just moves towards the ball without circling around to the ball's back,
// and therefore does not require field, localization, or video to run.
class SimpleChase : public SyncActionNode
{
public:
    SimpleChase(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("stop_dist", 1.0, "The distance at which the robot will stop moving towards the ball"),
            InputPort<double>("stop_angle", 0.1, "The angle at which the robot will stop turning towards the ball"),
            InputPort<double>("vy_limit", 0.2, "Limit the velocity in the Y direction to prevent instability while walking"),
            InputPort<double>("vx_limit", 0.6, "Limit the velocity in the X direction to prevent instability while walking"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};

// Go to the ball position seen by teammate
class GoToTeammateBall : public StatefulActionNode
{
public:
    GoToTeammateBall(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("long_range_threshold", 1.5, "When distance to target exceeds this value, prioritize moving towards it rather than fine-tuning position and orientation"),
            InputPort<double>("turn_threshold", 0.4, "For long distances, if angle to target exceeds this threshold, turn towards target first"),
            InputPort<double>("vx_limit", 0.6, "X direction velocity limit"),
            InputPort<double>("vy_limit", 0.4, "Y direction velocity limit"),
            InputPort<double>("vtheta_limit", 1.0, "Angular velocity limit"),
            InputPort<double>("x_tolerance", 0.5, "X coordinate tolerance"),
            InputPort<double>("y_tolerance", 0.5, "Y coordinate tolerance"),
            InputPort<double>("theta_tolerance", 0.2, "Angle tolerance"),
        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    void onHalted() override;

private:
    Brain *brain;
    double _targetX, _targetY, _targetTheta;
    double _longRangeThreshold, _turnThreshold, _vxLimit, _vyLimit, _vthetaLimit;
    double _xTolerance, _yTolerance, _thetaTolerance;
    int _selectedTeammateId;
    bool _hasValidTarget;
};

// Follow the robot with ball possession
class FollowTeammate : public StatefulActionNode
{
public:
    FollowTeammate(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("follow_distance", 2.0, "Distance to maintain from the teammate"),
            InputPort<double>("long_range_threshold", 3.0, "When distance to target exceeds this value, prioritize moving towards it"),
            InputPort<double>("turn_threshold", 0.4, "For long distances, if angle to target exceeds this threshold, turn towards target first"),
            InputPort<double>("vx_limit", 0.4, "X direction velocity limit"),
            InputPort<double>("vy_limit", 0.3, "Y direction velocity limit"),
            InputPort<double>("vtheta_limit", 0.8, "Angular velocity limit"),
            InputPort<double>("position_tolerance", 0.3, "Position tolerance for considering target reached"),
            InputPort<double>("angle_tolerance", 0.2, "Angle tolerance for considering target reached"),
        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    void onHalted() override;

private:
    Brain *brain;
    double _targetX, _targetY, _targetTheta;
    double _followDistance, _longRangeThreshold, _turnThreshold;
    double _vxLimit, _vyLimit, _vthetaLimit;
    double _positionTolerance, _angleTolerance;
    int _possessionPlayerId;
    bool _hasValidTarget;

    // Helper functions
    std::pair<double, double> calculateFollowPosition(double teammateX, double teammateY, double ballX, double ballY, double followDistance);
    double calculateBallViewingAngle(double robotX, double robotY, double ballX, double ballY);
};

// ------------------------------- FOR DEBUG -------------------------------
class PrintMsg : public SyncActionNode
{
public:
    PrintMsg(const std::string &name, const NodeConfig &config, Brain *_brain)
        : SyncActionNode(name, config)
    {
    }

    NodeStatus tick() override;

    static PortsList providedPorts()
    {
        return {InputPort<std::string>("msg")};
    }

private:
    Brain *brain;
};
