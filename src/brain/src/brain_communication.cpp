#include "brain.h"
#include "brain_communication.h"
#include <cmath>
#include <sstream>

BrainCommunication::BrainCommunication(Brain *argBrain) : brain(argBrain)
{
}

BrainCommunication::~BrainCommunication()
{
    clearupGameControllerBroadcast();
    clearupDiscoveryBroadcast();
    clearupDiscoveryReceiver();
    clearupCommunicationUnicast();
    clearupCommunicationReceiver();
}


void BrainCommunication::initUDPBroadcast()
{
    bool enableCom = false;
    brain->get_parameter("enable_com", enableCom);
    if (enableCom)
    {
        cout << RED_CODE << "Communication enabled." << RESET_CODE << endl;
        _discovery_udp_port = 20000 + brain->config->teamId;
        _unicast_udp_port = 30000 + brain->config->teamId;

        initGameControllerBroadcast();
        initDiscoveryBroadcast();
        initDiscoveryReceiver();
        initCommunicationUnicast();
        initCommunicationReceiver();
    }
    else
    {
        cout << RED_CODE << "Communication disabled." << RESET_CODE << endl;
    }
}
    

void BrainCommunication::initGameControllerBroadcast()
{
    try
    {
        _gc_send_socket = socket(AF_INET, SOCK_DGRAM, 0);
        if (_gc_send_socket < 0)
        {
        cout << RED_CODE << format("gc socket failed: %s", strerror(errno))
            << RESET_CODE << endl;
        throw std::runtime_error(strerror(errno));
        }
        // broadcast
        int gc_broadcast_enable = 1;
        if (setsockopt(_gc_send_socket, SOL_SOCKET, SO_BROADCAST, &gc_broadcast_enable, sizeof(gc_broadcast_enable)) < 0)
        {
        cout << RED_CODE << format("Failed to set gc SO_BROADCAST: %s", strerror(errno))
            << RESET_CODE << endl;
        throw std::runtime_error(strerror(errno));
        }
        // target address
        _gcsaddr.sin_family = AF_INET;
        _gcsaddr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
        _gcsaddr.sin_port = htons(GAMECONTROLLER_RETURN_PORT);

        _broadcast_gamecontrol_flag = true;
        _gamecontrol_broadcast_thread = std::thread([this](){ this->broadcastToGameController(); });
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}

void BrainCommunication::clearupGameControllerBroadcast()
{
    _broadcast_gamecontrol_flag = false;
    if (_gc_send_socket >= 0)
    {
        close(_gc_send_socket);
        _gc_send_socket = -1;
        cout << RED_CODE << format("GameControl send socket has been closed.")
            << RESET_CODE << endl;
    }
    if (_gamecontrol_broadcast_thread.joinable())
    {
        _gamecontrol_broadcast_thread.join();
    }
}

void BrainCommunication::initDiscoveryBroadcast()
{
    try
    {
        _discovery_send_socket = socket(AF_INET, SOCK_DGRAM, 0);
        if (_discovery_send_socket < 0)
        {
            cout << RED_CODE << format("socket failed: %s", strerror(errno))
                << RESET_CODE << endl;
            throw std::runtime_error(strerror(errno));
        }

        // 设置广播选项
        int broadcast = 1;
        if (setsockopt(_discovery_send_socket, SOL_SOCKET, SO_BROADCAST, 
                    &broadcast, sizeof(broadcast)) < 0)
        {
            cout << RED_CODE << format("Failed to set SO_BROADCAST: %s", strerror(errno))
                << RESET_CODE << endl;
            throw std::runtime_error(strerror(errno));
        }

        // 配置广播地址
        _saddr.sin_family = AF_INET;
        _saddr.sin_addr.s_addr = INADDR_BROADCAST;  // 255.255.255.255
        _saddr.sin_port = htons(_discovery_udp_port);

        _broadcast_discovery_flag = true;
        _discovery_broadcast_thread = std::thread([this](){ this->broadcastDiscovery(); });
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    
}

void BrainCommunication::clearupDiscoveryBroadcast()
{
    _broadcast_discovery_flag = false;
    if (_discovery_send_socket >= 0)
    {
        close(_discovery_send_socket);
        _discovery_send_socket = -1;
        cout << RED_CODE << format("Discovery send socket has been closed.")
            << RESET_CODE << endl;
    }

    if (_discovery_broadcast_thread.joinable())
    {
        _discovery_broadcast_thread.join();
    }
}

void BrainCommunication::initDiscoveryReceiver()
{
    try
    {
        _discovery_recv_socket = socket(AF_INET, SOCK_DGRAM, 0);
        if (_discovery_recv_socket < 0)
        {
            cout << RED_CODE << format("socket failed: %s", strerror(errno))
                << RESET_CODE << endl;
            throw std::runtime_error(strerror(errno));
        }

        // 允许地址重用
        int reuse = 1;
        if (setsockopt(_discovery_recv_socket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0)
        {
            cout << RED_CODE << format("Failed to set SO_REUSEADDR: %s", strerror(errno))
                << RESET_CODE << endl;
            throw std::runtime_error(strerror(errno));
        }

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);  // listen on all interfaces
        addr.sin_port = htons(_discovery_udp_port);
        
        if (bind(_discovery_recv_socket, (sockaddr *)&addr, sizeof(addr)) < 0)
        {
            cout << RED_CODE << format("bind failed: %s (port=%d)", strerror(errno), _discovery_udp_port)
                << RESET_CODE << endl;
            throw std::runtime_error(strerror(errno));
        }

        cout << GREEN_CODE << format("Listening for UDP broadcast on port %d", _discovery_udp_port)
            << RESET_CODE << endl;

        _receive_discovery_flag = true;
        _discovery_recv_thread = std::thread([this](){ this->spinDiscoveryReceiver(); });
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}

void BrainCommunication::clearupDiscoveryReceiver()
{
    _receive_discovery_flag = false;
    if (_discovery_recv_socket >= 0)
    {
        close(_discovery_recv_socket);
        _discovery_recv_socket = -1;
        cout << RED_CODE << format("Communication receive socket has been closed.")
            << RESET_CODE << endl;
    }
    if (_discovery_recv_thread.joinable())
    {
        _discovery_recv_thread.join();
    }
}

void BrainCommunication::broadcastToGameController() {
    while (_broadcast_gamecontrol_flag)
    {
        // cout << RED_CODE << format("broadcastToGameController header=%s version=%d teamId=%d, playerId=%d", gc_return_data.header, gc_return_data.version, brain->config->teamId, brain->config->playerId)
        //     << RESET_CODE << endl;
        gc_return_data.team = brain->config->teamId;
        gc_return_data.player = brain->config->playerId + 1; // player number starts with 1
        gc_return_data.message = GAMECONTROLLER_RETURN_MSG_ALIVE;

        int ret = sendto(_gc_send_socket, &gc_return_data, sizeof(gc_return_data), 0, (sockaddr *)&_gcsaddr, sizeof(_gcsaddr));
        if (ret < 0)
        {
            cout << RED_CODE << format("gc sendto failed: %s", strerror(errno))
                << RESET_CODE << endl;
        }
        this_thread::sleep_for(chrono::milliseconds(BROADCAST_GAME_CONTROL_INTERVAL_MS));
    }
}

void BrainCommunication::broadcastDiscovery() {
    while (_broadcast_discovery_flag)
    {
        TeamDiscoveryMsg msg;

        msg.communicationId = _discovery_msg_id++;
        msg.teamId = brain->config->teamId;
        msg.playerId = brain->config->playerId;

        int ret = sendto(_discovery_send_socket, &msg, sizeof(msg), 0, (sockaddr *)&_saddr, sizeof(_saddr));
        if (ret < 0)
        {
            cout << RED_CODE << format("sendto failed: %s", strerror(errno))
                << RESET_CODE << endl;
        }
        this_thread::sleep_for(chrono::milliseconds(BROADCAST_DISCOVERY_INTERVAL_MS));
    } 
}

void BrainCommunication::spinDiscoveryReceiver() {    
    sockaddr_in addr{};
    socklen_t addr_len = sizeof(addr);

    TeamDiscoveryMsg msg;

    while (_receive_discovery_flag) {

        ssize_t len = recvfrom(_discovery_recv_socket, &msg, sizeof(msg), 0, (sockaddr *)&addr, &addr_len);

        if (len < 0)
        {
            cout << RED_CODE << format("receiving UDP message failed: %s", strerror(errno))
                << RESET_CODE << endl;
            continue;
        }

        if (msg.validation != VALIDATION_DISCOVERY) return; // fail to pass validation

        if (msg.teamId != brain->config->teamId) { // ignore other teams' messages
            continue;
        }

        if (msg.playerId == brain->config->playerId) {  // ignore self messages
            cout << YELLOW_CODE <<  format(
                "discoveryID: %d, teamId:%d, playerId: %d, address: %s:%d",
                msg.communicationId, msg.teamId, msg.playerId, inet_ntoa(addr.sin_addr), ntohs(addr.sin_port))
                << RESET_CODE << endl;
            continue;
        } else {
            // teammate discovered
            cout << GREEN_CODE <<  format(
                "discoveryID: %d, teamId:%d, playerId: %d, address: %s:%d",
                msg.communicationId, msg.teamId, msg.playerId, inet_ntoa(addr.sin_addr), ntohs(addr.sin_port))
                << RESET_CODE << endl;
            
            auto time_now = brain->get_clock()->now();
            std::lock_guard<std::mutex> lock(_teammate_addresses_mutex);
            _teammate_addresses[addr.sin_addr.s_addr] = {
                addr.sin_addr.s_addr,
                msg.playerId,
                time_now,
                0.0f,   // robotPoseX
                0.0f,   // robotPoseY  
                0.0f,   // robotPoseTheta
                false,  // hasValidPose
                false,  // ballDetected
                0.0f,   // ballPosX
                0.0f,   // ballPosY
                false,  // hasValidBallInfo
                0.0f,   // ballCost
                false,  // hasPossession
                -1,     // masterPlayerId
                -1,     // possessionPlayerId
                false,  // hasValidCollaborationInfo
                -1,     // dynamicRole
                -1,     // goalKeeperPlayerId
                -1,     // strikerPlayerId
                -1      // followerPlayerId
            };
        }
    }
}

void BrainCommunication::cleanupExpiredTeammates() {
    std::lock_guard<std::mutex> lock(_teammate_addresses_mutex);    
    for (auto it = _teammate_addresses.begin(); it != _teammate_addresses.end();) {
        auto timeDiff = this->brain->get_clock()->now().nanoseconds() - it->second.lastUpdate.nanoseconds();
        if (timeDiff > TEAMMATE_TIMEOUT_MS * 1e6) {
            cout << YELLOW_CODE << format("Teammate id %d timed out", it->second.playerId) 
                 << RESET_CODE << endl;
            it = _teammate_addresses.erase(it);
        } else {
            ++it;
        }
    }
}

void BrainCommunication::initCommunicationUnicast() {
    try
    {
        _unicast_socket = socket(AF_INET, SOCK_DGRAM, 0);
        if (_unicast_socket < 0) {
            cout << RED_CODE << format("socket failed: %s", strerror(errno))
                << RESET_CODE << endl;
            throw std::runtime_error("Failed to create unicast socket");
        }

        _unicast_saddr.sin_family = AF_INET;
        _unicast_saddr.sin_port = htons(_unicast_udp_port);

        _unicast_communication_flag = true;
        _unicast_thread = std::thread([this](){ this->unicastCommunication(); });
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    
}

void BrainCommunication::unicastCommunication() {
    while (_unicast_communication_flag) {
        cleanupExpiredTeammates();
        TeamCommunicationMsg msg;
        msg.validation = VALIDATION_COMMUNICATION;
        msg.communicationId = _team_communication_msg_id++;
        msg.teamId = brain->config->teamId;
        msg.playerId = brain->config->playerId;
        
        // 填充机器人在场地坐标系中的位置信息
        msg.robotPoseX = static_cast<float>(brain->data->robotPoseToField.x);
        msg.robotPoseY = static_cast<float>(brain->data->robotPoseToField.y);
        msg.robotPoseTheta = static_cast<float>(brain->data->robotPoseToField.theta);
        
        // 填充球的信息
        msg.ballDetected = brain->data->ballDetected;
        if (msg.ballDetected) {
            msg.ballPosX = static_cast<float>(brain->data->ball.posToField.x);
            msg.ballPosY = static_cast<float>(brain->data->ball.posToField.y);
        } else {
            msg.ballPosX = 0.0f;
            msg.ballPosY = 0.0f;
        }
        
        // 填充协作信息
        msg.ballCost = static_cast<float>(brain->data->ballCost);
        msg.hasPossession = brain->data->hasBallPossession;
        msg.masterPlayerId = (brain->config->collaborationRole == "master") ? brain->config->playerId : -1;
        msg.possessionPlayerId = brain->data->possessionPlayerId;
        
        // 填充动态角色分配信息
        msg.dynamicRole = brain->data->dynamicRole;
        msg.goalKeeperPlayerId = brain->data->goalKeeperPlayerId;
        msg.strikerPlayerId = brain->data->strikerPlayerId;
        msg.followerPlayerId = brain->data->followerPlayerId;
        
        // Debug: 输出协作信息状态
        static int debug_counter = 0;
        if (debug_counter % 100 == 0) { // 每100次输出一次，避免spam
            cout << YELLOW_CODE << format(
                "DEBUG发送协作信息: role='%s', playerId=%d, masterPlayerId=%d, possessionPlayerId=%d, ballCost=%.2f, 动态角色=%d, 角色分配(主攻=%d,守门=%d,跟随=%d)",
                brain->config->collaborationRole.c_str(), brain->config->playerId,
                msg.masterPlayerId, msg.possessionPlayerId, msg.ballCost, msg.dynamicRole,
                msg.strikerPlayerId, msg.goalKeeperPlayerId, msg.followerPlayerId) << RESET_CODE << endl;
        }
        debug_counter++;
        
        // TODO: add more information you want to send to teammates
        msg.testInfo = 1234567; 

        std::lock_guard<std::mutex> lock(_teammate_addresses_mutex);
        for (auto it = _teammate_addresses.begin(); it != _teammate_addresses.end(); ++it) {
            auto ip = it->second.ip;

            // cout << GREEN_CODE << format("unicastCommunication to %s", inet_ntoa(*(in_addr *)&ip))
            //     << RESET_CODE << endl;
            
            _unicast_saddr.sin_addr.s_addr = ip;
            int ret = sendto(_unicast_socket, &msg, sizeof(msg), 0, (sockaddr *)&_unicast_saddr, sizeof(_unicast_saddr));
            if (ret < 0) {
                cout << RED_CODE << format("sendto failed: %s", strerror(errno))
                    << RESET_CODE << endl;
            }
        }
        this_thread::sleep_for(chrono::milliseconds(UNICAST_INTERVAL_MS));
    }
}

void BrainCommunication::clearupCommunicationUnicast() {
    _unicast_communication_flag = false;
    if (_unicast_socket >= 0) {
        close(_unicast_socket);
        _unicast_socket = -1;
        cout << RED_CODE << format("Communication send socket has been closed.")
            << RESET_CODE << endl;
    }

    if (_unicast_thread.joinable()) {
        _unicast_thread.join();
    }
}

void BrainCommunication::initCommunicationReceiver() {
    try
    {
        _communication_recv_socket = socket(AF_INET, SOCK_DGRAM, 0);
        if (_communication_recv_socket < 0) {
            cout << RED_CODE << format("socket failed: %s", strerror(errno))
                << RESET_CODE << endl;
            throw std::runtime_error(strerror(errno));
        }

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(_unicast_udp_port);
        
        if (bind(_communication_recv_socket, (sockaddr *)&addr, sizeof(addr)) < 0) {
            cout << RED_CODE << format("bind failed: %s (port=%d)", strerror(errno), _unicast_udp_port)
                << RESET_CODE << endl;
            throw std::runtime_error(strerror(errno));
        }

        _receive_communication_flag = true;
        _communication_recv_thread = std::thread([this](){ this->spinCommunicationReceiver(); });
    
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}

void BrainCommunication::spinCommunicationReceiver() {
    sockaddr_in addr{};
    socklen_t addr_len = sizeof(addr);

    TeamCommunicationMsg msg;

    while (_receive_communication_flag) {

        ssize_t len = recvfrom(_communication_recv_socket, &msg, sizeof(msg), 0, (sockaddr *)&addr, &addr_len);

        if (len < 0) {
            cout << RED_CODE << format("receiving UDP message failed: %s", strerror(errno))
                << RESET_CODE << endl;
            continue;
        }

        if (msg.validation != VALIDATION_COMMUNICATION) return; // fail to pass validation

        if (msg.teamId != brain->config->teamId) { // ignore other teams' messages
            continue;
        }

        if (msg.playerId == brain->config->playerId) {  // ignore self messages
            cout << CYAN_CODE <<  format(
                "communicationId: %d, playerId: %d, testInfo: %d",
                msg.communicationId, msg.playerId, msg.testInfo)
                << RESET_CODE << endl;
            continue;
        } 

        // 处理接收到的队友位置信息和球信息
        {
            std::lock_guard<std::mutex> lock(_teammate_addresses_mutex);
            auto it = _teammate_addresses.find(addr.sin_addr.s_addr);
            if (it != _teammate_addresses.end()) {
                // 更新队友位置信息
                it->second.robotPoseX = msg.robotPoseX;
                it->second.robotPoseY = msg.robotPoseY;
                it->second.robotPoseTheta = msg.robotPoseTheta;
                it->second.hasValidPose = true;
                
                // 更新队友的球信息
                it->second.ballDetected = msg.ballDetected;
                it->second.ballPosX = msg.ballPosX;
                it->second.ballPosY = msg.ballPosY;
                it->second.hasValidBallInfo = true;
                
                // 更新队友的协作信息
                it->second.ballCost = msg.ballCost;
                it->second.hasPossession = msg.hasPossession;
                it->second.masterPlayerId = msg.masterPlayerId;
                it->second.possessionPlayerId = msg.possessionPlayerId;
                it->second.hasValidCollaborationInfo = true;
                
                // 更新队友的动态角色分配信息
                it->second.dynamicRole = msg.dynamicRole;
                it->second.goalKeeperPlayerId = msg.goalKeeperPlayerId;
                it->second.strikerPlayerId = msg.strikerPlayerId;
                it->second.followerPlayerId = msg.followerPlayerId;
                
                it->second.lastUpdate = brain->get_clock()->now();
                
                cout << GREEN_CODE << format(
                    "收到队友 %d 信息: 位置(%.2f, %.2f, %.2f°) 球(%s", 
                    msg.playerId, msg.robotPoseX, msg.robotPoseY, 
                    msg.robotPoseTheta * 180.0 / M_PI,
                    msg.ballDetected ? "已发现" : "未发现");
                
                if (msg.ballDetected) {
                    cout << format(" 球位置: %.2f, %.2f", msg.ballPosX, msg.ballPosY);
                }
                
                cout << format(" 协作(cost: %.2f, possession: %s, master: %d, assigned: %d)", 
                    msg.ballCost, msg.hasPossession ? "是" : "否", 
                    msg.masterPlayerId, msg.possessionPlayerId);
                
                cout << ")" << RESET_CODE << endl;
            }
        }
    }
}

void BrainCommunication::clearupCommunicationReceiver() {
    _receive_communication_flag = false;
    if (_communication_recv_socket >= 0) {
        close(_communication_recv_socket);
        _communication_recv_socket = -1;
        cout << RED_CODE << format("Communication receive socket has been closed.")
            << RESET_CODE << endl;
    }
    if (_communication_recv_thread.joinable()) {
        _communication_recv_thread.join();
    }
}

std::vector<BrainCommunication::TeammateInfo> BrainCommunication::getTeammatePositions() {
    std::lock_guard<std::mutex> lock(_teammate_addresses_mutex);
    std::vector<TeammateInfo> teammates;
    
    for (const auto& pair : _teammate_addresses) {
        if (pair.second.hasValidPose) {
            teammates.push_back(pair.second);
        }
    }
    
    return teammates;
}

std::vector<BrainCommunication::TeammateInfo> BrainCommunication::getTeammateBallInfo() {
    // std::lock_guard<std::mutex> lock(_teammate_addresses_mutex);
    std::vector<TeammateInfo> teammates;
    
    for (const auto& pair : _teammate_addresses) {
        if (pair.second.hasValidBallInfo) {
            teammates.push_back(pair.second);
        }
    }
    
    return teammates;
}

std::vector<BrainCommunication::TeammateInfo> BrainCommunication::getTeammateCollaborationInfo() {
    // std::lock_guard<std::mutex> lock(_teammate_addresses_mutex);
    std::vector<TeammateInfo> teammates;
    
    for (const auto& pair : _teammate_addresses) {
        if (pair.second.hasValidCollaborationInfo) {
            teammates.push_back(pair.second);
        }
    }
    
    return teammates;
}

// Dashboard implementation
void BrainCommunication::initDashboard() {
    // Get dashboard configuration from environment or default
    const char* dashboard_ip = std::getenv("DASHBOARD_IP");
    const char* dashboard_port = std::getenv("DASHBOARD_PORT");
    
    if (!dashboard_ip) {
        dashboard_ip = "192.168.4.77";  // Default Mac IP
    }
    if (!dashboard_port) {
        dashboard_port = "8080";  // Default port
    }
    
    // Create UDP socket for dashboard
    _dashboard_socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (_dashboard_socket < 0) {
        cout << YELLOW_CODE << "Dashboard socket creation failed, continuing without dashboard" << RESET_CODE << endl;
        return;
    }
    
    // Configure dashboard address
    memset(&_dashboard_addr, 0, sizeof(_dashboard_addr));
    _dashboard_addr.sin_family = AF_INET;
    _dashboard_addr.sin_port = htons(std::atoi(dashboard_port));
    
    if (inet_pton(AF_INET, dashboard_ip, &_dashboard_addr.sin_addr) <= 0) {
        cout << YELLOW_CODE << "Invalid dashboard IP: " << dashboard_ip << ", continuing without dashboard" << RESET_CODE << endl;
        close(_dashboard_socket);
        _dashboard_socket = -1;
        return;
    }
    
    _dashboard_enabled = true;
    cout << GREEN_CODE << format("Dashboard initialized: %s:%s", dashboard_ip, dashboard_port) << RESET_CODE << endl;
}

void BrainCommunication::sendDashboardData() {
    if (!_dashboard_enabled || _dashboard_socket < 0) return;
    
    try {
        // Create simple JSON string manually
        std::ostringstream json;
        json << "{";
        
        // Robot identification
        json << "\"robot_id\":" << brain->config->playerId << ",";
        json << "\"robot_name\":\"robot" << (brain->config->playerId + 1) << "\",";
        json << "\"team_id\":" << brain->config->teamId << ",";
        json << "\"timestamp\":" << brain->get_clock()->now().seconds() << ",";
        
        // Game state
        json << "\"game\":{";
        json << "\"state\":\"" << brain->tree->getEntry<string>("gc_game_state") << "\",";
        json << "\"kickoff_side\":" << (brain->tree->getEntry<bool>("gc_is_kickoff_side") ? "true" : "false") << ",";
        json << "\"score\":" << brain->data->lastScore;
        json << "},";
        
        // Robot pose
        json << "\"robot\":{";
        json << "\"pose\":{";
        json << "\"x\":" << brain->data->robotPoseToField.x << ",";
        json << "\"y\":" << brain->data->robotPoseToField.y << ",";
        json << "\"theta\":" << brain->data->robotPoseToField.theta;
        json << "},";
        
        // Ball information
        json << "\"ball\":{";
        json << "\"detected\":" << (brain->data->ballDetected ? "true" : "false");
        if (brain->data->ballDetected) {
            json << ",\"x\":" << brain->data->ball.posToField.x;
            json << ",\"y\":" << brain->data->ball.posToField.y;
            json << ",\"range\":" << brain->data->ball.range;
        }
        json << "}";
        json << "},";
        
        // Collaboration
        json << "\"collaboration\":{";
        json << "\"role\":\"" << brain->config->collaborationRole << "\",";
        json << "\"dynamic_role\":" << brain->data->dynamicRole << ",";
        json << "\"has_possession\":" << (brain->tree->getEntry<bool>("has_ball_possession") ? "true" : "false") << ",";
        json << "\"possession_player\":" << brain->data->possessionPlayerId << ",";
        json << "\"ball_cost\":" << brain->data->ballCost;
        json << "},";
        
        // Behavior
        json << "\"behavior\":{";
        json << "\"decision\":\"" << brain->tree->getEntry<string>("decision") << "\",";
        json << "\"ball_location_known\":" << (brain->tree->getEntry<bool>("ball_location_known") ? "true" : "false");
        json << "},";
        
        // Performance
        double avgLoopTime = brain->totalLoopTime / std::max(brain->loopCount, 1);
        json << "\"performance\":{";
        json << "\"avg_loop_time\":" << avgLoopTime << ",";
        json << "\"max_loop_time\":" << brain->maxLoopTime;
        json << "},";
        
        // Head tracking
        json << "\"head\":{";
        json << "\"pitch\":" << brain->data->headPitch << ",";
        json << "\"yaw\":" << brain->data->headYaw;
        json << "},";
        
        // Recovery state
        json << "\"recovery\":{";
        json << "\"state\":" << static_cast<int>(brain->data->recoveryState) << ",";
        json << "\"available\":" << (brain->data->isRecoveryAvailable ? "true" : "false");
        json << "},";
        
        // Team data
        auto teammates = getTeammateCollaborationInfo();
        json << "\"team_count\":" << teammates.size();
        
        json << "}";
        
        // Send data (non-blocking)
        std::string jsonStr = json.str();
        sendto(_dashboard_socket, jsonStr.c_str(), jsonStr.length(), MSG_DONTWAIT,
               (struct sockaddr*)&_dashboard_addr, sizeof(_dashboard_addr));
        
    } catch (...) {
        // Ignore dashboard errors silently
    }
}
