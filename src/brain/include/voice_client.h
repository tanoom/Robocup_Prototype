#pragma once

#include <string>
#include <map>
#include <cstdlib>

class Brain; // Forward declaration

using namespace std;

/**
 * Voice client class for robot voice feedback functionality
 * Provides simple TTS (Text-to-Speech) functionality
 */
class VoiceClient
{
public:
    VoiceClient(Brain *argBrain);
    
    void init();

    /**
     * @brief Play specified text
     * @param text Text to be spoken
     * @return 0 indicates success
     */
    int speak(const string& text);

    /**
     * @brief Play direction command
     * @param direction Direction: "right", "left", "forward", "backward"
     * @return 0 indicates success
     */
    int speakDirection(const string& direction);

    /**
     * @brief Play action status
     * @param action Action: "start", "stop", "kick", "chase", "find"
     * @return 0 indicates success
     */
    int speakAction(const string& action);

    /**
     * @brief Play ball status
     * @param status Status: "found", "lost", "tracking"
     * @return 0 indicates success
     */
    int speakBallStatus(const string& status);

    /**
     * @brief Play decision information
     * @param decision Decision: "chase", "kick", "adjust", "find"
     * @return 0 indicates success
     */
    int speakDecision(const string& decision);

    /**
     * @brief Play numerical information
     * @param value Numerical value
     * @param unit Unit
     * @return 0 indicates success
     */
    int speakValue(double value, const string& unit = "");

    /**
     * @brief Set voice parameters
     * @param speed Speech speed (WPM)
     * @param amplitude Volume
     */
    void setVoiceParameters(int speed = 200, int amplitude = 500);

    /**
     * @brief Stop current speech playback
     */
    void stopSpeaking();

private:
    Brain *brain;
    
    // Voice parameters
    int voice_speed_;
    int voice_amplitude_;
    
    // Predefined phrase mappings
    map<string, string> phrases_;
    
    // Initialize phrase mappings
    void initPhrases();
    
    // Execute espeak command
    int executeEspeak(const string& text);
}; 