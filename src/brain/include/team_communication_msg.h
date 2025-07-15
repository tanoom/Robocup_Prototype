#pragma once

#include "types.h"

#define VALIDATION_COMMUNICATION 31201
#define VALIDATION_DISCOVERY 41202
struct TeamCommunicationMsg
{
    int validation = VALIDATION_COMMUNICATION; // validate msg, to determine if it's sent by us.
    int communicationId;
    int teamId;
    int playerId;
    // Robot position information in field coordinate system
    float robotPoseX;     // Robot X coordinate (field coordinate system)
    float robotPoseY;     // Robot Y coordinate (field coordinate system)
    float robotPoseTheta; // Robot orientation angle (field coordinate system)
    // Ball information
    bool ballDetected;    // Whether ball is detected
    float ballPosX;       // Ball X coordinate (field coordinate system)
    float ballPosY;       // Ball Y coordinate (field coordinate system)
    // Collaboration information
    float ballCost;       // Cost function value for this robot to reach the ball
    bool hasPossession;   // Whether this robot currently has ball possession
    int masterPlayerId;   // Player ID of the master robot (-1 if unknown)
    int possessionPlayerId; // Player ID of robot that should possess the ball (-1 if unknown)
    // Dynamic role assignment (master robot decides roles based on possession and position)
    int dynamicRole;      // 0=striker, 1=goal_keeper, 2=striker_follower (-1 if unknown)
    int goalKeeperPlayerId; // Player ID assigned as goal keeper (-1 if unknown)
    int strikerPlayerId;    // Player ID assigned as main striker (-1 if unknown)
    int followerPlayerId;   // Player ID assigned as follower striker (-1 if unknown)
    // TODO: You can add more information you want to send to teammates
    int testInfo;
};

struct TeamDiscoveryMsg
{
    int validation = VALIDATION_DISCOVERY; // validate msg, to determine if it's sent by us.
    int communicationId;
    int teamId;
    int playerId;
};
