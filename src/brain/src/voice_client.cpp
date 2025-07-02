#include "voice_client.h"
#include "brain.h"
#include "utils/print.h"
#include <sstream>
#include <iomanip>

VoiceClient::VoiceClient(Brain *argBrain) : brain(argBrain)
{
    // Set default voice parameters
    voice_speed_ = 150;      // speech speed
    voice_amplitude_ = 400;   // volume
    voice_pitch_ = 45;       // pitch
}

void VoiceClient::init()
{
    initPhrases();
    prtDebug("VoiceClient initialized with English TTS support");
}

void VoiceClient::initPhrases()
{
    // Direction commands
    phrases_["right"] = "right";
    phrases_["left"] = "left";
    phrases_["forward"] = "forward";
    phrases_["backward"] = "backward";
    phrases_["up"] = "up";
    phrases_["down"] = "down";
    
    // Action commands
    phrases_["start"] = "start";
    phrases_["stop"] = "stop";
    phrases_["kick"] = "kick";
    phrases_["chase"] = "chase";
    phrases_["find"] = "find";
    phrases_["adjust"] = "adjust";
    
    // Ball status
    phrases_["found"] = "ball found";
    phrases_["lost"] = "ball lost";
    phrases_["tracking"] = "tracking ball";
    
    // Decision info
    phrases_["chase_decision"] = "decision chase";
    phrases_["kick_decision"] = "decision kick";
    phrases_["adjust_decision"] = "decision adjust";
    phrases_["find_decision"] = "decision find";
    
    // Status info
    phrases_["ready"] = "ready";
    phrases_["mission_complete"] = "mission complete";
    phrases_["goal_scored"] = "goal scored";
    phrases_["system_error"] = "system error";
}

int VoiceClient::speak(const string& text)
{
    if (text.empty()) {
        prtDebug("Speech text is empty");
        return -1;
    }
    
    return executeEspeak(text);
}

int VoiceClient::speakDirection(const string& direction)
{
    auto it = phrases_.find(direction);
    if (it != phrases_.end()) {
        prtDebug("Speaking direction: " + direction + " -> " + it->second);
        return executeEspeak(it->second);
    } else {
        prtDebug("Unknown direction, speaking in English: " + direction);
        return executeEspeak(direction);
    }
}

int VoiceClient::speakAction(const string& action)
{
    auto it = phrases_.find(action);
    if (it != phrases_.end()) {
        prtDebug("Speaking action: " + action + " -> " + it->second);
        return executeEspeak(it->second);
    } else {
        prtDebug("Unknown action, speaking in English: " + action);
        return executeEspeak(action);
    }
}

int VoiceClient::speakBallStatus(const string& status)
{
    auto it = phrases_.find(status);
    if (it != phrases_.end()) {
        prtDebug("Speaking ball status: " + status + " -> " + it->second);
        return executeEspeak(it->second);
    } else {
        prtDebug("Unknown ball status, speaking in English: " + status);
        return executeEspeak(status);
    }
}

int VoiceClient::speakDecision(const string& decision)
{
    string key = decision + "_decision";
    auto it = phrases_.find(key);
    if (it != phrases_.end()) {
        prtDebug("Speaking decision: " + decision + " -> " + it->second);
        return executeEspeak(it->second);
    } else {
        // Try direct lookup
        it = phrases_.find(decision);
        if (it != phrases_.end()) {
            prtDebug("Speaking decision: " + decision + " -> " + it->second);
            return executeEspeak(it->second);
        } else {
            prtDebug("Unknown decision, speaking in English: " + decision);
            return executeEspeak(decision);
        }
    }
}

int VoiceClient::speakValue(double value, const string& unit)
{
    stringstream ss;
    ss << fixed << setprecision(1) << value;
    if (!unit.empty()) {
        ss << unit;
    }
    
    string text = ss.str();
    prtDebug("Speaking value: " + text);
    return executeEspeak(text);
}

void VoiceClient::setVoiceParameters(int speed, int amplitude, int pitch)
{
    voice_speed_ = max(80, min(500, speed));         // Limit to reasonable range
    voice_amplitude_ = max(0, min(200, amplitude));   // Limit to reasonable range
    voice_pitch_ = max(0, min(99, pitch));           // Limit to reasonable range
    
    prtDebug("Voice parameters updated: speed=" + to_string(voice_speed_) + 
             ", amplitude=" + to_string(voice_amplitude_) + 
             ", pitch=" + to_string(voice_pitch_));
}

void VoiceClient::stopSpeaking()
{
    // Stop all espeak processes
    system("killall espeak > /dev/null 2>&1");
    prtDebug("All speech stopped");
}

int VoiceClient::executeEspeak(const string& text)
{
    stringstream command;
    command << "espeak ";
    
    // Set voice (always English now)
    command << "-v en ";
    
    // Set parameters
    command << "-s " << voice_speed_ << " ";     // speed
    command << "-a " << voice_amplitude_ << " "; // volume
    command << "-p " << voice_pitch_ << " ";     // pitch
    
    // Add text and background execution
    command << "\"" << text << "\" &";
    
    string cmd = command.str();
    prtDebug("Executing: " + cmd);
    
    int result = system(cmd.c_str());
    return (result == 0) ? 0 : -1;
} 