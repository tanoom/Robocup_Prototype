#!/bin/bash
# Robot Configuration Quick Switch Script

set -e

# Color definitions
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Project root directory
PROJECT_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
CONFIG_MANAGER="$PROJECT_ROOT/config_manager.py"

# Check Python environment
check_python() {
    # Try Python 3.8 first (PyYAML is installed there)
    if command -v python3.8 &> /dev/null; then
        PYTHON_CMD="python3.8"
    elif command -v python3 &> /dev/null; then
        PYTHON_CMD="python3"
    elif command -v python &> /dev/null; then
        PYTHON_CMD="python"
    else
        echo -e "${RED}Error: Python environment not found${NC}"
        exit 1
    fi
}

# Check dependencies
check_dependencies() {
    echo -e "${BLUE}Checking dependencies...${NC}"
    
    # Check PyYAML
    if ! $PYTHON_CMD -c "import yaml" &> /dev/null; then
        echo -e "${YELLOW}Warning: PyYAML not found, installing...${NC}"
        pip install PyYAML
    fi
    
    # Check configuration manager
    if [ ! -f "$CONFIG_MANAGER" ]; then
        echo -e "${RED}Error: Configuration manager not found: $CONFIG_MANAGER${NC}"
        exit 1
    fi
    
    # Check configuration file
    if [ ! -f "$PROJECT_ROOT/robot_profiles.yaml" ]; then
        echo -e "${RED}Error: Configuration file not found: $PROJECT_ROOT/robot_profiles.yaml${NC}"
        exit 1
    fi
}

# Show help information
show_help() {
    echo -e "${GREEN}Robot Configuration Quick Switch Script${NC}"
    echo ""
    echo "Usage:"
    echo "  ./switch_robot.sh [options] [robot_id]"
    echo ""
    echo "Options:"
    echo "  -h, --help     Show help information"
    echo "  -l, --list     List all available robot configurations"
    echo "  -c, --current  Show current robot configuration"
    echo ""
    echo "Examples:"
    echo "  ./switch_robot.sh                    # Interactive selection"
    echo "  ./switch_robot.sh robot1             # Switch to robot1"
    echo "  ./switch_robot.sh --list             # List all configurations"
    echo "  ./switch_robot.sh --current          # Show current configuration"
    echo ""
    echo "Available robot configurations:"
    echo "  robot1 - Robot #1 (Striker, left starting position)"
    echo "  robot2 - Robot #2 (Striker, right starting position)"
    echo "  robot3 - Robot #3 (Goalkeeper, left starting position)"
    echo "  robot4 - Robot #4 (Goalkeeper, right starting position)"
}

# Show current configuration
show_current() {
    echo -e "${BLUE}Current robot configuration:${NC}"
    $PYTHON_CMD "$CONFIG_MANAGER" list | grep "✓ Current" || echo -e "${YELLOW}Current configuration not found${NC}"
}

# List all configurations
list_robots() {
    echo -e "${BLUE}List all robot configurations:${NC}"
    $PYTHON_CMD "$CONFIG_MANAGER" list
}

# Switch configuration
switch_robot() {
    local robot_id="$1"
    
    if [ -z "$robot_id" ]; then
        echo -e "${BLUE}Interactive robot configuration selection:${NC}"
        $PYTHON_CMD "$CONFIG_MANAGER"
    else
        echo -e "${BLUE}Switching to robot configuration: $robot_id${NC}"
        $PYTHON_CMD "$CONFIG_MANAGER" apply "$robot_id"
    fi
}

# Main function
main() {
    check_python
    check_dependencies
    
    case "$1" in
        -h|--help)
            show_help
            ;;
        -l|--list)
            list_robots
            ;;
        -c|--current)
            show_current
            ;;
        "")
            # Interactive mode
            switch_robot
            ;;
        *)
            # Direct switch to specified robot
            switch_robot "$1"
            ;;
    esac
}

# Welcome message
echo -e "${GREEN}🤖 Robot Configuration Management System${NC}"
echo -e "${BLUE}========================================${NC}"

# Execute main function
main "$@" 