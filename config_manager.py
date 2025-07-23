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
        
        # Get robot profile file path
        robot_profiles = self.config_data.get('robot_profiles', {})
        robot_profile_path = robot_profiles.get(current_robot)
        if not robot_profile_path:
            print(f"✗ Robot profile path not found: {current_robot}")
            return None
        
        # Load robot configuration from separate file
        robot_config_file = self.project_root / robot_profile_path
        try:
            with open(robot_config_file, 'r', encoding='utf-8') as f:
                robot_config = yaml.safe_load(f)
            if not robot_config:
                print(f"✗ Robot configuration file is empty: {robot_config_file}")
                return None
        except FileNotFoundError:
            print(f"✗ Robot configuration file not found: {robot_config_file}")
            return None
        except yaml.YAMLError as e:
            print(f"✗ Robot configuration file format error: {e}")
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
        
        # Add ignore configuration if present
        if 'ignore' in sftp_config:
            new_config["ignore"] = sftp_config['ignore']
        
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

    dashboard:
      ip: "{brain_config.get('dashboard', {}).get('ip', '192.168.5.75')}"
      port: {brain_config.get('dashboard', {}).get('port', 8080)}

    rerunLog:
      enable: true
      server_addr: "{brain_config['rerunLog']['server_addr']}"
      img_interval: {brain_config['rerunLog']['img_interval']}
"""
        
        # Write new configuration
        with open(brain_path, 'w', encoding='utf-8') as f:
            f.write(yaml_content)
        
        dashboard_config = brain_config.get('dashboard', {})
        dashboard_ip = dashboard_config.get('ip', '192.168.5.75')
        dashboard_port = dashboard_config.get('port', 8080)
        print(f"  ✓ Updated Brain configuration: player_id={brain_config['game']['player_id']}, role={brain_config['game']['player_role']}, collaboration_role={brain_config['game']['collaboration_role']}, dashboard={dashboard_ip}:{dashboard_port}")
    
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
    
    def update_vision_config(self, robot_config=None):
        """Update Vision configuration with robot-specific or template configuration"""
        vision_path = self.config_paths['vision']
        
        # Backup original file
        self.backup_file(vision_path)
        
        # Check if robot has specific vision config
        vision_config = None
        if robot_config and 'vision' in robot_config:
            vision_config = robot_config['vision']
            print(f"  ✓ Using robot-specific vision configuration")
        else:
            # Load vision template
            vision_template_path = self.config_data.get('vision_template')
            if vision_template_path:
                template_file = self.project_root / vision_template_path
                try:
                    with open(template_file, 'r', encoding='utf-8') as f:
                        vision_config = yaml.safe_load(f)
                    print(f"  ✓ Using vision template configuration")
                except (FileNotFoundError, yaml.YAMLError) as e:
                    print(f"  ✗ Failed to load vision template: {e}")
                    return
        
        if not vision_config:
            print(f"  ✗ No vision configuration available")
            return
        
        # Write new configuration with strict ordering
        self._write_vision_config(vision_path, vision_config)
        
        print(f"  ✓ Updated Vision configuration")
    
    def _write_vision_config(self, file_path, vision_config):
        """Write vision configuration with strict field ordering"""
        with open(file_path, 'w', encoding='utf-8') as f:
            # Write fields in the specified order
            if 'show_res' in vision_config:
                f.write(f"show_res: {str(vision_config['show_res']).lower()}\n")
            
            if 'camera' in vision_config:
                f.write("camera:\n")
                camera = vision_config['camera']
                
                if 'type' in camera:
                    f.write(f"  type: {camera['type']}\n")
                
                if 'intrin' in camera:
                    f.write("  intrin:\n")
                    intrin = camera['intrin']
                    for key in ['fx', 'fy', 'cx', 'cy', 'distortion_model']:
                        if key in intrin:
                            f.write(f"    {key}: {intrin[key]}\n")
                    if 'distortion_coeffs' in intrin:
                        f.write("    distortion_coeffs:\n")
                        for coeff in intrin['distortion_coeffs']:
                            f.write(f"    - {coeff}\n")
                
                if 'extrin' in camera:
                    f.write("  extrin:\n")
                    for row in camera['extrin']:
                        f.write("  -\n")
                        for val in row:
                            f.write(f"    - {val}\n")
                
                for key in ['pitch_compensation', 'yaw_compensation', 'z_compensation']:
                    if key in camera:
                        f.write(f"  {key}: {camera[key]}\n")
            
            if 'detection_model' in vision_config:
                f.write("detection_model:\n")
                detection = vision_config['detection_model']
                if 'model_path' in detection:
                    f.write(f"  model_path: {detection['model_path']}\n")
                if 'confidence_threshold' in detection:
                    f.write(f"  confidence_threshold: {detection['confidence_threshold']}\n")
            
            if 'use_depth' in vision_config:
                f.write(f"use_depth: {str(vision_config['use_depth']).lower()}\n")
            
            if 'ball_pose_estimator' in vision_config:
                f.write("ball_pose_estimator:\n")
                ball = vision_config['ball_pose_estimator']
                for key in ['use_depth', 'radius', 'down_sample_leaf_size', 'cluster_distance_threshold', 'fitting_distance_threshold']:
                    if key in ball:
                        f.write(f"  {key}: {ball[key]}\n")
            
            if 'human_like_pose_estimator' in vision_config:
                f.write("human_like_pose_estimator:\n")
                human = vision_config['human_like_pose_estimator']
                for key in ['use_depth', 'down_sample_leaf_size', 'fitting_distance_threshold', 'statistic_outlier_multiplier']:
                    if key in human:
                        f.write(f"  {key}: {human[key]}\n")
            
            if 'calibration' in vision_config:
                f.write("calibration:\n")
                calibration = vision_config['calibration']
                if 'handeye' in calibration:
                    f.write("  handeye:\n")
                    handeye = calibration['handeye']
                    if 'calibration_time' in handeye:
                        f.write(f"    calibration_time: {handeye['calibration_time']}\n")
                    if 'reprojection_error' in handeye:
                        f.write(f"    reprojection_error: {handeye['reprojection_error']}\n")
    
    def apply_configuration(self, robot_name=None):
        """Apply configuration"""
        if not self.load_profiles():
            return False
        
        if robot_name:
            # If robot name is specified, update current_robot first
            robot_profiles = self.config_data.get('robot_profiles', {})
            if robot_name in robot_profiles:
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
            self.update_vision_config(robot_config)
            
            print(f"\n🎉 Configuration update completed!")
            print(f"   Current robot: {robot_config['name']}")
            print(f"   SFTP host: {robot_config['sftp']['host']}")
            print(f"   Player ID: {robot_config['brain']['game']['player_id']}")
            print(f"   Role: {robot_config['brain']['game']['player_role']}")
            print(f"   Starting position: {robot_config['brain']['game']['player_start_pos']}")
            print(f"   Collaboration role: {robot_config['brain']['game']['collaboration_role']}")
            dashboard_config = robot_config['brain'].get('dashboard', {})
            print(f"   Dashboard: {dashboard_config.get('ip', '192.168.5.75')}:{dashboard_config.get('port', 8080)}")
            
            return True
            
        except Exception as e:
            print(f"✗ Configuration update failed: {e}")
            return False
    
    def list_robots(self):
        """List all available robot configurations"""
        if not self.load_profiles():
            return
        
        current_robot = self.config_data.get('current_robot')
        robot_profiles = self.config_data.get('robot_profiles', {})
        
        print("🤖 Available robot configurations:")
        for robot_id, robot_profile_path in robot_profiles.items():
            status = "✓ Current" if robot_id == current_robot else "  "
            
            # Load robot configuration from separate file to get name and description
            robot_config_file = self.project_root / robot_profile_path
            try:
                with open(robot_config_file, 'r', encoding='utf-8') as f:
                    robot_config = yaml.safe_load(f)
                name = robot_config.get('name', 'N/A')
                description = robot_config.get('description', 'N/A')
            except (FileNotFoundError, yaml.YAMLError):
                name = 'N/A'
                description = 'Configuration file error'
            
            print(f"{status} {robot_id}: {name} - {description}")

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