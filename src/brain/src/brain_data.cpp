#include "brain_data.h"
#include "utils/math.h"

vector<FieldMarker> BrainData::getMarkers()
{
    vector<FieldMarker> res;
    
    // Process field markings (crosses and penalty points)
    for (size_t i = 0; i < markings.size(); i++)
    {
        auto label = markings[i].label;
        auto x = markings[i].posToRobot.x;
        auto y = markings[i].posToRobot.y;
        auto confidence = markings[i].confidence;

        char markerType = ' ';
        if (label == "LCross")
            markerType = 'L';
        else if (label == "TCross")
            markerType = 'T';
        else if (label == "XCross")
            markerType = 'X';
        else if (label == "PenaltyPoint")
            markerType = 'P';

        if (markerType != ' ')
            res.push_back(FieldMarker{markerType, x, y, confidence});
    }
    
    // Process goalposts
    for (size_t i = 0; i < goalposts.size(); i++)
    {
        auto goalpost = goalposts[i];
        auto x = goalpost.posToRobot.x;
        auto y = goalpost.posToRobot.y;
        auto confidence = goalpost.confidence;

        // Add goalpost as 'G' type marker
        res.push_back(FieldMarker{'G', x, y, confidence});
    }
    
    return res;
}

Pose2D BrainData::robot2field(const Pose2D &poseToRobot)
{
    Pose2D poseToField;
    transCoord(
        poseToRobot.x, poseToRobot.y, poseToRobot.theta,
        robotPoseToField.x, robotPoseToField.y, robotPoseToField.theta,
        poseToField.x, poseToField.y, poseToField.theta);
    poseToField.theta = toPInPI(poseToField.theta);
    return poseToField;
}

Pose2D BrainData::field2robot(const Pose2D &poseToField)
{
    Pose2D poseToRobot;
    double xfr, yfr, thetafr; // fr = field to robot
    yfr = sin(robotPoseToField.theta) * robotPoseToField.x - cos(robotPoseToField.theta) * robotPoseToField.y;
    xfr = -cos(robotPoseToField.theta) * robotPoseToField.x - sin(robotPoseToField.theta) * robotPoseToField.y;
    thetafr = -robotPoseToField.theta;
    transCoord(
        poseToField.x, poseToField.y, poseToField.theta,
        xfr, yfr, thetafr,
        poseToRobot.x, poseToRobot.y, poseToRobot.theta);
    return poseToRobot;
}