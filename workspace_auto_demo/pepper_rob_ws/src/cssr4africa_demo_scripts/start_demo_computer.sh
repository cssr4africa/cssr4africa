#!/bin/bash

################################################################################
# CSSR System - Computer Startup Script (Computer as Remote Node)
#
# This script launches the CSSR system nodes on the main computer. The computer connects to the Jetson's ROS MASTER.
#
# It handles:
# - ROS environment setup
# - Network configuration for distributed ROS (connecting to Jetson's roscore)
# - Robot hardware interface launch
# - All system nodes launch (except face detection on Jetson)
#
# Usage:
#   ./start_demo_computer.sh [JETSON_IP] [COMPUTER_IP] [ROBOT_IP] [LAUNCH_CONTROLLER]
#
# Arguments:
#   JETSON_IP           - IP address of Jetson running roscore (default: 172.29.111.248)
#   COMPUTER_IP         - IP address of this computer as remote node (default: 172.29.111.245)
#   ROBOT_IP            - IP address of Pepper robot (default: 172.29.111.230)
#   LAUNCH_CONTROLLER   - Launch behavior controller (true/false, default: true)
#
# Prerequisites:
# - ROS Noetic installed
# - Workspace built and sourced
# - Virtual environments created for Python nodes
# - Pepper robot connected and accessible
# - Jetson nodes launched (roscore + camera + face detection)
#
################################################################################

set -e  # Exit on error

# ============================================================================
# Configuration
# ============================================================================

# Default IP addresses
DEFAULT_JETSON_IP="172.29.111.248"
DEFAULT_COMPUTER_IP="172.29.111.245"
DEFAULT_ROBOT_IP="172.29.111.230"
DEFAULT_LAUNCH_CONTROLLER="true"
DEFAULT_ROBOT_PORT="9559"
DEFAULT_NETWORK_INTERFACE="wlp0s20f3"

# Initialize with defaults
JETSON_IP="$DEFAULT_JETSON_IP"
COMPUTER_IP="$DEFAULT_COMPUTER_IP"
ROBOT_IP="$DEFAULT_ROBOT_IP"
LAUNCH_CONTROLLER="$DEFAULT_LAUNCH_CONTROLLER"
ROBOT_PORT="$DEFAULT_ROBOT_PORT"
NETWORK_INTERFACE="$DEFAULT_NETWORK_INTERFACE"

# ============================================================================
# Help Function
# ============================================================================

show_help() {
    cat << EOF
Usage: $0 [OPTIONS]

Launch CSSR System nodes on Computer (connects to Jetson's ROS Master)

OPTIONS:
    --jetson-ip=IP           IP address of Jetson running roscore (default: $DEFAULT_JETSON_IP)
    --computer-ip=IP         IP address of this computer (default: $DEFAULT_COMPUTER_IP)
    --robot-ip=IP            IP address of Pepper robot (default: $DEFAULT_ROBOT_IP)
    --launch-controller=BOOL Launch behavior controller (true/false, default: $DEFAULT_LAUNCH_CONTROLLER)
    --robot-port=PORT        Pepper robot port (default: $DEFAULT_ROBOT_PORT)
    --network-interface=IF   Network interface name (default: $DEFAULT_NETWORK_INTERFACE)
    -h, --help               Show this help message

EXAMPLES:
    # Use all defaults
    $0

    # Specify only Jetson IP
    $0 --jetson-ip=192.168.1.248

    # Specify multiple parameters
    $0 --jetson-ip=192.168.1.248 --computer-ip=192.168.1.245 --robot-ip=192.168.1.230

    # Launch without behavior controller
    $0 --launch-controller=false

    # Backward compatible positional arguments
    $0 JETSON_IP COMPUTER_IP ROBOT_IP LAUNCH_CONTROLLER

EOF
    exit 0
}

# ============================================================================
# Parse Arguments
# ============================================================================

# Check for help flag
if [[ "$1" == "-h" ]] || [[ "$1" == "--help" ]]; then
    show_help
fi

# Check if using positional arguments (backward compatibility)
if [[ $# -gt 0 ]] && [[ ! "$1" =~ ^-- ]]; then
    # Positional arguments mode
    JETSON_IP="${1:-$DEFAULT_JETSON_IP}"
    COMPUTER_IP="${2:-$DEFAULT_COMPUTER_IP}"
    ROBOT_IP="${3:-$DEFAULT_ROBOT_IP}"
    LAUNCH_CONTROLLER="${4:-$DEFAULT_LAUNCH_CONTROLLER}"
else
    # Named arguments mode
    for arg in "$@"; do
        case $arg in
            --jetson-ip=*)
                JETSON_IP="${arg#*=}"
                shift
                ;;
            --computer-ip=*)
                COMPUTER_IP="${arg#*=}"
                shift
                ;;
            --robot-ip=*)
                ROBOT_IP="${arg#*=}"
                shift
                ;;
            --launch-controller=*)
                LAUNCH_CONTROLLER="${arg#*=}"
                shift
                ;;
            --robot-port=*)
                ROBOT_PORT="${arg#*=}"
                shift
                ;;
            --network-interface=*)
                NETWORK_INTERFACE="${arg#*=}"
                shift
                ;;
            *)
                echo "Unknown option: $arg"
                echo "Use --help for usage information"
                exit 1
                ;;
        esac
    done
fi

# ROS Workspace path - ADJUST THIS TO YOUR WORKSPACE
ROS_WORKSPACE="${ROS_WORKSPACE:-$HOME/workspace_auto_demo/pepper_rob_ws}"

# Launch file path
LAUNCH_FILE="$(rospack find cssr_system 2>/dev/null || echo "$ROS_WORKSPACE/src/cssr4africa/cssr_system")/launch/cssrSystemLaunchMissionComputer.launch"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# ============================================================================
# Helper Functions
# ============================================================================

print_header() {
    echo -e "\n${CYAN}========================================${NC}"
    echo -e "${CYAN}$1${NC}"
    echo -e "${CYAN}========================================${NC}\n"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ ERROR: $1${NC}"
}

print_warning() {
    echo -e "${YELLOW} NOTE: $1${NC}"
}

print_info() {
    echo -e "${BLUE} $1${NC}"
}

# ============================================================================
# Pre-flight Checks
# ============================================================================

print_header "CSSR System - Computer Nodes Startup"

print_info "Jetson IP (ROS Master): $JETSON_IP"
print_info "Computer IP: $COMPUTER_IP"
print_info "Robot IP: $ROBOT_IP"
print_info "Launch Behavior Controller: $LAUNCH_CONTROLLER"
print_info "ROS Workspace: $ROS_WORKSPACE"
echo
print_warning "NOTE: This computer will connect to Jetson's roscore"
print_info "Make sure the Jetson (ROS Master) is already running"

# Check if ROS workspace exists
if [ ! -d "$ROS_WORKSPACE" ]; then
    print_error "ROS workspace not found at: $ROS_WORKSPACE"
    print_info "Please set the ROS_WORKSPACE environment variable or edit this script"
    exit 1
fi
print_success "ROS workspace found"

# ============================================================================
# ROS Environment Setup
# ============================================================================

print_header "Setting up ROS Environment"

# Source ROS Noetic
if [ -f "/opt/ros/noetic/setup.bash" ]; then
    source /opt/ros/noetic/setup.bash
    print_success "Sourced ROS Noetic"
else
    print_error "ROS Noetic not found. Is it installed?"
    exit 1
fi

# Source workspace
if [ -f "$ROS_WORKSPACE/devel/setup.bash" ]; then
    source "$ROS_WORKSPACE/devel/setup.bash"
    print_success "Sourced workspace: $ROS_WORKSPACE"
else
    print_error "Workspace not built. Please run 'catkin_make' in $ROS_WORKSPACE"
    exit 1
fi

# ============================================================================
# ROS Network Configuration (Connect to Jetson's roscore)
# ============================================================================

print_header "Configuring ROS Network (Connecting to Jetson's roscore)"

# Set ROS Master URI to Jetson
export ROS_MASTER_URI="http://$JETSON_IP:11311"
print_success "ROS_MASTER_URI set to: $ROS_MASTER_URI"

# Set ROS IP to this computer's IP
export ROS_IP="$COMPUTER_IP"
print_success "ROS_IP set to: $ROS_IP"

# Check if Jetson's roscore is accessible
print_info "Checking connection to Jetson's roscore..."
if timeout 5 rostopic list &> /dev/null; then
    print_success "Connected to roscore on Jetson ($JETSON_IP)"
else
    print_error "Cannot connect to roscore on Jetson ($JETSON_IP)"
    print_info "Make sure:"
    print_info "  1. Jetson is running (start_demo_jetson.sh)"
    print_info "  2. roscore is running on Jetson"
    print_info "  3. Jetson IP ($JETSON_IP) is correct"
    print_info "  4. Network connection is working"
    print_info "  5. Firewall allows ROS communication (port 11311)"
    print_info "  6. Both devices are on the same network"
    exit 1
fi

# ============================================================================
# Check Robot Connection
# ============================================================================

print_header "Checking Pepper Robot Connection"

print_info "Attempting to ping robot at $ROBOT_IP..."
if ping -c 2 -W 2 "$ROBOT_IP" &> /dev/null; then
    print_success "Robot is reachable at $ROBOT_IP"
else
    print_warning "Cannot ping robot at $ROBOT_IP"
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# ============================================================================
# Check Jetson Face Detection
# ============================================================================

print_header "Checking Jetson Nodes Status"

print_info "Checking if face detection is available from Jetson..."
if timeout 5 rostopic list 2>/dev/null | grep -q "/faceDetection"; then
    print_success "Face detection node detected (Jetson nodes are running)"
else
    print_warning "Face detection node not detected"
    print_info "Make sure you have:"
    print_info "  1. Started the Jetson nodes using start_demo_jetson.sh"
    print_info "  2. Configured ROS networking properly"
    print_info "  3. Camera and face detection are running on Jetson"
    echo
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# ============================================================================
# Check Virtual Environments
# ============================================================================

print_header "Checking Virtual Environments"

VENV_BASE="$HOME/integrated/dependencies"

check_venv() {
    local venv_name=$1
    local venv_path="$VENV_BASE/$venv_name"

    if [ -d "$venv_path" ]; then
        if [ -f "$venv_path/bin/activate" ]; then
            print_success "Found: $venv_name"
            return 0
        fi
    fi
    print_warning "Missing: $venv_name at $venv_path"
    return 1
}

check_venv "sound_detection"
check_venv "speech_event"
check_venv "text_to_speech"

echo

# ============================================================================
# Launch CSSR System on Computer
# ============================================================================

print_header "Launching CSSR System Nodes"

print_info "Launch sequence:"
print_info "  0. Robot Hardware Interface (actuators + sensors)"
print_info "  1. Robot Localization"
print_info "  2. Robot Navigation"
print_info "  3. Sound Detection (with virtual env)"
print_info "  4. Speech Event (with virtual env)"
print_info "  5. Text to Speech (with virtual env)"
print_info "  6. Overt Attention"
print_info "  7. Gesture Execution"
if [ "$LAUNCH_CONTROLLER" = "true" ]; then
    print_info "  8. Behavior Controller (Mission Control)"
fi
echo

print_warning "This will take approximately 60 seconds for all nodes to start"
print_info "Nodes will start sequentially with delays to ensure dependencies are met"
echo

print_info "Press Ctrl+C to stop all nodes"
echo

# Launch the system with all parameters
roslaunch cssr_system cssrSystemLaunchMissionComputer.launch \
    jetson_ip:="$JETSON_IP" \
    computer_ip:="$COMPUTER_IP" \
    robot_ip:="$ROBOT_IP" \
    network_interface:="$NETWORK_INTERFACE" \
    launch_controller:="$LAUNCH_CONTROLLER" \
    launch_actuators:=true \
    launch_sensors:=true \
    launch_audio_nodes:=true

# This line is reached when roslaunch is terminated
print_header "Shutdown Complete"
print_info "All nodes on this computer have been stopped"
print_info "Note: roscore is still running on Jetson if you haven't stopped it"
