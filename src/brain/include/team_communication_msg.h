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
