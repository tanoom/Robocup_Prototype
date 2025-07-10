#!/usr/bin/env python3.8
# -*- coding: utf-8 -*-

"""
Robot Configuration Management Script
Automatically update all related files based on robot_profiles.yaml configuration
"""

import os
import sys
import yaml
import json
import shutil
from pathlib import Path

class RobotConfigManager:
    def __init__(self, project_root=None):
        """
        Initialize configuration manager
        
        Args:
            project_root: Project root directory, auto-detect if None
        """
        if project_root is None:
            # Auto-detect project root directory (script is now in root)
            current_dir = Path(__file__).parent
            self.project_root = current_dir
        else:
            self.project_root = Path(project_root)
        
        self.profiles_file = self.project_root / "robot_profiles.yaml"
        self.config_data = None
        
        # Configuration file paths
        self.config_paths = {
            'sftp': self.project_root / ".vscode" / "sftp.json",
            'brain': self.project_root / "src" / "brain" / "config" / "brain.yaml",
            'fastdds': self.project_root / "configs" / "fastdds.xml",
            'vision': self.project_root / "src" / "vision" / "config" / "vision.yaml"
        }
    
    def load_profiles(self):
        """Load robot configuration file"""
        try:
            with open(self.profiles_file, 'r', encoding='utf-8') as f:
                self.config_data = yaml.safe_load(f)
            print(f"✓ Successfully loaded configuration file: {self.profiles_file}")
            return True
        except FileNotFoundError:
            print(f"✗ Configuration file not found: {self.profiles_file}")
            return False
        except yaml.YAMLError as e:
            print(f"✗ Configuration file format error: {e}")
            return False
    
    def get_current_robot_config(self):
        """Get current robot configuration"""
        if not self.config_data:
            return None
        
        current_robot = self.config_data.get('current_robot')
        if not current_robot:
            print("✗ Current robot configuration not found")
            return None
        
        robot_config = self.config_data.get('robots', {}).get(current_robot)
        if not robot_config:
            print(f"✗ Robot configuration not found: {current_robot}")
            return None
        
        return current_robot, robot_config
    
    def backup_file(self, file_path):
        """Backup file (disabled)"""
        # Backup functionality disabled per user request
        pass
    
    def update_sftp_config(self, sftp_config):
        """Update SFTP configuration"""
        sftp_path = self.config_paths['sftp']
        
        # Ensure directory exists
        sftp_path.parent.mkdir(parents=True, exist_ok=True)
        
        # Backup original file
        self.backup_file(sftp_path)
        
        # Build new SFTP configuration
        new_config = {
            "name": f"booster_{sftp_config['host'].split('.')[-1]}",
            "host": sftp_config['host'],
            "protocol": "sftp",
            "port": sftp_config['port'],
            "username": sftp_config['username'],
            "password": sftp_config['password'],
            "remotePath": sftp_config['remotePath'],
            "uploadOnSave": True,
            "useTempFile": False,
            "openSsh": True,
            "preserveTime": True,
            "fileMode": "0644",
            "dirMode": "0755"
        }
        
        # Write new configuration
        with open(sftp_path, 'w', encoding='utf-8') as f:
            json.dump(new_config, f, indent=4, ensure_ascii=False)
        
        print(f"  ✓ Updated SFTP configuration: {sftp_config['host']}")
    
    def update_brain_config(self, brain_config):
        """Update Brain configuration"""
        brain_path = self.config_paths['brain']
        
        # Backup original file
        self.backup_file(brain_path)
        
        # Build brain.yaml content with strict ordering as per user requirements
        yaml_content = f"""brain_node:
  ros__parameters:
    game:
      team_id: {brain_config['game']['team_id']}
      player_id: {brain_config['game']['player_id']}
      field_type: "{brain_config['game']['field_type']}"
      player_role: "{brain_config['game']['player_role']}"
      player_start_pos: "{brain_config['game']['player_start_pos']}"
      collaboration_role: "{brain_config['game']['collaboration_role']}"
      
    robot:
      robot_height: 1.0
      odom_factor: 1.2
      vx_factor: 0.90
      yaw_offset: 0.10
    
    enable_com: true

    rerunLog:
      enable: true
      server_addr: "{brain_config['rerunLog']['server_addr']}"
      img_interval: {brain_config['rerunLog']['img_interval']}
"""
        
        # Write new configuration
        with open(brain_path, 'w', encoding='utf-8') as f:
            f.write(yaml_content)
        
        print(f"  ✓ Updated Brain configuration: player_id={brain_config['game']['player_id']}, role={brain_config['game']['player_role']}, collaboration_role={brain_config['game']['collaboration_role']}")
    
    def update_fastdds_config(self, network_config):
        """Update FastDDS network configuration"""
        fastdds_path = self.config_paths['fastdds']
        
        # Backup original file
        self.backup_file(fastdds_path)
        
        # Build interface address list
        interface_lines = []
        for interface in network_config['interfaces']:
            interface_lines.append(f"                <address>{interface}</address>")
        
        # Build new XML configuration
        xml_content = f"""<?xml version="1.0" encoding="UTF-8" ?>

<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles" >
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>UdpTransport</transport_id>
            <type>UDPv4</type>
            <interfaceWhiteList>
{chr(10).join(interface_lines)}

            </interfaceWhiteList>
        </transport_descriptor>
    </transport_descriptors>

    <participant profile_name="udp_transport_profile" is_default_profile="true">
        <rtps>
            <userTransports>
                <transport_id>UdpTransport</transport_id>
            </userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
        </rtps>
    </participant>
</profiles>
"""
        
        # Write new configuration
        with open(fastdds_path, 'w', encoding='utf-8') as f:
            f.write(xml_content)
        
        print(f"  ✓ Updated FastDDS configuration: {len(network_config['interfaces'])} network interfaces")
    
    def update_vision_config(self, vision_config=None):
        """Update Vision configuration with strict formatting"""
        vision_path = self.config_paths['vision']
        
        # Backup original file
        self.backup_file(vision_path)
        
        # Build vision.yaml content with strict formatting as per user requirements
        vision_content = """show_res: false
camera:
  type: "realsense" # realsense or zed
  intrin:
    fx: 643.898
    fy: 643.216
    cx: 649.038
    cy: 357.21
    distortion_coeffs: [-0.0553056,0.065975,-0.000994232,2.98548e-05,-0.0216579]
    distortion_model: 2 # 0: none, 1: opencv format, 2: for rs d455 only
  extrin:
    - [ 0.05659255, 0.03014561, 0.99794215, 0.04217854]
    - [-0.99839673, 0.00283376, 0.05653273, 0.01405556]
    - [-0.00112372,-0.9995415 , 0.03025765,-0.01538294]
    - [ 0.        , 0.        , 0.        , 1.        ]
  pitch_compensation: 0.0 # in degree
  yaw_compensation: 0.0
  z_compensation: 0.0
detection_model:
  model_path: "./src/vision/model/best_orin.engine"
  confidence_threshold: 0.2
use_depth: true
ball_pose_estimator:
  use_depth: false
  radius: 1.109
  down_sample_leaf_size: 0.01
  cluster_distance_threshold: 0.01
  fitting_distance_threshold: 0.01
human_like_pose_estimator:
  use_depth: false
  down_sample_leaf_size: 0.01
  fitting_distance_threshold: 0.01
  statistic_outlier_multiplier: 0.01
"""
        
        # Write new configuration
        with open(vision_path, 'w', encoding='utf-8') as f:
            f.write(vision_content)
        
        print(f"  ✓ Updated Vision configuration")
    
    def apply_configuration(self, robot_name=None):
        """Apply configuration"""
        if not self.load_profiles():
            return False
        
        if robot_name:
            # If robot name is specified, update current_robot first
            if robot_name in self.config_data.get('robots', {}):
                self.config_data['current_robot'] = robot_name
                with open(self.profiles_file, 'w', encoding='utf-8') as f:
                    yaml.dump(self.config_data, f, default_flow_style=False, allow_unicode=True, indent=2)
                print(f"✓ Switched to robot configuration: {robot_name}")
            else:
                print(f"✗ Robot configuration not found: {robot_name}")
                return False
        
        # Get current robot configuration
        result = self.get_current_robot_config()
        if not result:
            return False
        
        current_robot, robot_config = result
        
        print(f"\n🤖 Applying robot configuration: {current_robot}")
        print(f"   Description: {robot_config.get('description', 'N/A')}")
        print(f"   Starting to update configuration files...")
        
        try:
            # Update each configuration file
            self.update_sftp_config(robot_config['sftp'])
            self.update_brain_config(robot_config['brain'])
            self.update_fastdds_config(robot_config['network'])
            self.update_vision_config()
            
            print(f"\n🎉 Configuration update completed!")
            print(f"   Current robot: {robot_config['name']}")
            print(f"   SFTP host: {robot_config['sftp']['host']}")
            print(f"   Player ID: {robot_config['brain']['game']['player_id']}")
            print(f"   Role: {robot_config['brain']['game']['player_role']}")
            print(f"   Starting position: {robot_config['brain']['game']['player_start_pos']}")
            print(f"   Collaboration role: {robot_config['brain']['game']['collaboration_role']}")
            
            return True
            
        except Exception as e:
            print(f"✗ Configuration update failed: {e}")
            return False
    
    def list_robots(self):
        """List all available robot configurations"""
        if not self.load_profiles():
            return
        
        current_robot = self.config_data.get('current_robot')
        robots = self.config_data.get('robots', {})
        
        print("🤖 Available robot configurations:")
        for robot_id, robot_config in robots.items():
            status = "✓ Current" if robot_id == current_robot else "  "
            print(f"{status} {robot_id}: {robot_config.get('name', 'N/A')} - {robot_config.get('description', 'N/A')}")

def main():
    """Main function"""
    manager = RobotConfigManager()
    
    if len(sys.argv) > 1:
        command = sys.argv[1]
        
        if command == "list":
            manager.list_robots()
        elif command == "apply":
            if len(sys.argv) > 2:
                robot_name = sys.argv[2]
                manager.apply_configuration(robot_name)
            else:
                manager.apply_configuration()
        else:
            print("Usage: python config_manager.py [list|apply] [robot_name]")
    else:
        # Interactive mode
        manager.list_robots()
        print("\nPlease select a robot configuration to apply:")
        robot_name = input("Enter robot ID (or press Enter to apply current configuration): ").strip()
        
        if robot_name:
            manager.apply_configuration(robot_name)
        else:
            manager.apply_configuration()

if __name__ == "__main__":
    main() 