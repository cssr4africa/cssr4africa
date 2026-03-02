# CSSR System - Launcher/GUI Guide

This guide covers two ways to launch the CSSR system.

## Table of Contents

1. [Option 1: Unified Script (Single Command)](#option-1-unified-script)
2. [Option 2: GUI Launcher](#option-2-gui-launcher)
3. [Comparison](#comparison)
4. [Setup Instructions](#setup-instructions)

---

## Option 1: Unified Script

A single bash script that launches nodes on both Jetson and Computer.


### Prerequisites

**You must set up passwordless SSH between machines (if not already done):**

```bash
# On the controlling machine (where you'll run the script)
ssh-keygen -t rsa -b 4096

# Copy key to remote machine
# If running from Computer:
ssh-copy-id roboticslab@172.29.111.240  # Jetson (verify ip address and hostname)

# If running from Jetson:
ssh-copy-id cssr4africa1@172.29.111.237  # Computer (verify ip address and hostname)

# Test SSH (should NOT ask for password)
# From Computer:
ssh roboticslab@172.29.111.240 "echo 'SSH works'"
# From Jetson:
ssh cssr4africa1@172.29.111.237 "echo 'SSH works'"
```

### Usage

```bash
# Navigate to scripts directory
cd ~/workspace_auto_demo/pepper_rob_ws/src/cssr4africa_demo_script

# Make script executable (first time only)
chmod +x start_cssr_demo.sh

# Run with all defaults (from Computer)
./start_cssr_demo.sh

# Run with custom IPs
./start_cssr_demo.sh --jetson-ip=192.168.1.240 --computer-ip=192.168.1.237

# Run from Jetson instead
./start_cssr_demo.sh --control-from=jetson

# Without behavior controller
./start_cssr_demo.sh --launch-controller=false

# Show help
./start_cssr_demo.sh --help
```

### How It Works

**When running from Computer:**

1. Script executes on Computer
2. SSH into Jetson → Start roscore + camera + face detection
3. Wait 15 seconds for Jetson to initialize
4. Start Computer nodes locally

**When running from Jetson:**

1. Script executes on Jetson
2. Start roscore + camera + face detection locally
3. Wait 15 seconds
4. SSH into Computer → Start Computer nodes

### Stopping the System

Press `Ctrl+C` to stop all nodes. The script will clean up processes on both machines.

---

## Option 2: GUI Launcher

A graphical application interface with buttons and status indicators.

### Features

 **Big START button** - One click to launch everything

 **Real-time status indicators** - See which nodes are running (turns green when confirmed active)

 **Live log window** - Monitor system output

 **Stop button** - Cleanly shutdown system

### Installation

```bash
# Install required Python package (if not already installed)
pip3 install tk

# Navigate to scripts directory
cd ~/workspace_auto_demo/pepper_rob_ws/src/cssr4africa_demo_script

# Make script executable
chmod +x cssr_launcher_gui.py
```

### Running the GUI

**Method 1: Desktop Icon Shortcut**

```bash
# Copy desktop launcher to Desktop
cp CSSR_Launcher.desktop ~/Desktop/

# Make it executable
chmod +x ~/Desktop/CSSR_Launcher.desktop

# Double-click the icon on your Desktop to launch!
```

**Method 2: Add to Applications Menu**

```bash
# Copy to applications folder
mkdir -p ~/.local/share/applications
cp CSSR_Launcher.desktop ~/.local/share/applications/

# Now you can launch from your system's application menu
```

**Method 3: From Terminal**

```bash
cd ~/workspace_auto_demo/pepper_rob_ws/src/cssr4africa_demo_script
python3 cssr_launcher_gui.py
```

### Using the GUI

#### Configuration Panel

Fill in the IP addresses:
- **Jetson IP**: IP address of Jetson (default: 172.29.111.240)
- **Computer IP**: IP address of Computer (default: 172.29.111.237)
- **Robot IP**: IP address of Pepper robot (default: 172.29.111.230)
- **Launch Behavior Controller**: Check/uncheck to enable/disable
- **Control from**: Select "computer" or "jetson"

#### Starting the System

1. **Check your configuration** in the Configuration Panel
2. **Click the green "START SYSTEM" button**
3. **Watch the status indicators** turn green as nodes start
4. **Monitor the log window** for detailed output

#### Status Indicators

Colors:
- **Gray (●)** - Stopped/Not running
- **Orange (●)** - Starting
- **Green (●)** - Running
- **Red (●)** - Error

#### Stopping the System

Click the red **"STOP SYSTEM"** button to cleanly shutdown all nodes.

---

## Comparison

| Feature | Unified Script | GUI Launcher |
|---------|---------------|--------------|
| **Ease of Use** | Command-line | Point-and-click |
| **Visual Status** | Terminal output only | Color-coded indicators |
| **Network Config** | Command-line args | GUI form fields |
| **Log Viewing** | Terminal scrollback | Dedicated log window |
| **Remote Control** | Yes (via SSH) | No (local only) |

---

## Setup Instructions

### For Unified Script

1. **Set up SSH keys** (one-time setup):

   ```bash
   # On Computer
   ssh-keygen -t rsa -b 4096
   ssh-copy-id roboticslab@172.29.111.240

   # Test
   ssh roboticslab@172.29.111.240 "echo 'Success'"
   ```

2. **Make script executable**:

   ```bash
   chmod +x start_cssr_demo.sh
   ```

3. **Run it**:

   ```bash
   ./start_cssr_demo.sh
   ```

### For GUI Launcher

1. **Install dependencies**:

   ```bash
   pip3 install tk
   ```

2. **Make script executable**:

   ```bash
   chmod +x cssr_launcher_gui.py
   ```

3. **Create desktop shortcut**:

   ```bash
   # Edit CSSR_Launcher.desktop and update the Exec path
   # Then:
   cp CSSR_Launcher.desktop ~/Desktop/
   chmod +x ~/Desktop/CSSR_Launcher.desktop
   ```

4. **Launch**:

   Double-click the desktop icon or run:
   ```bash
   python3 cssr_launcher_gui.py
   ```

---

## Troubleshooting

### Unified Script Issues

**Problem:** "Permission denied" when SSH'ing

**Solution:** Set up SSH keys properly:
```bash
ssh-keygen -t rsa -b 4096
ssh-copy-id user@remote-ip
```

**Problem:** "Connection refused" to remote machine

**Solution:**
- Check if remote machine is on
- Verify IP addresses are correct
- Check network connectivity: `ping remote-ip`

### GUI Launcher Issues

**Problem:** "ModuleNotFoundError: No module named 'tkinter'"

**Solution:**
```bash
# Ubuntu/Debian
sudo apt-get install python3-tk

# Or use pip
pip3 install tk
```

**Problem:** GUI shows but START button doesn't work / "Script not found" error

**Solution:**
- **IMPORTANT**: `start_cssr_demo.sh` must be in the SAME folder as `cssr_launcher_gui.py`
- The GUI will automatically try to make the script executable
- Check the log window - it shows the exact path being searched and lists all files in the directory
- If script is found but not executable, manually run: `chmod +x start_cssr_demo.sh`
- If script is not found, verify both files are in the same directory:
  ```bash
  # Find both files
  find ~ -name "cssr_launcher_gui.py" -o -name "start_cssr_demo.sh"

  # They should be in the same directory
  # Move the script if needed:
  mv start_cssr_demo.sh /path/to/same/folder/as/gui/
  ```

**Problem:** Desktop icon doesn't launch GUI

**Solution:**
- Edit `CSSR_Launcher.desktop`
- Update the `Exec=` line with the correct full path
- Make it executable: `chmod +x CSSR_Launcher.desktop`

---

## Advanced Usage

### Customizing Default IPs in GUI

Edit `cssr_launcher_gui.py` and change these lines (~29-31):

```python
self.config = {
    'jetson_ip': tk.StringVar(value="YOUR_JETSON_IP"),
    'computer_ip': tk.StringVar(value="YOUR_COMPUTER_IP"),
    'robot_ip': tk.StringVar(value="YOUR_ROBOT_IP"),
    ...
}
```

### Running GUI Remotely (X11 Forwarding)

```bash
# SSH with X11 forwarding
ssh -X user@remote-machine

# Launch GUI
python3 cssr_launcher_gui.py
```

---

## Screenshots (GUI)

### Main Window

```
┌──────────────────────────────────────────────────────┐
│  🤖 CSSR System Launcher                             │
├──────────────────────────────────────────────────────┤
│  Configuration                                       │
│  Jetson IP:    [172.29.111.240]  Computer IP: [...]  │
│  Robot IP:     [172.29.111.230]  ☑ Launch Controller │
│  Control from: [computer ▼]                          │
├──────────────────────────────────────────────────────┤
│  System Status                                       │
│  ROS Master: ● Camera: ● Face Detection: ●           │
│  Robot Interface: ● Navigation: ● Controller: ●      │
├──────────────────────────────────────────────────────┤
│  ┌──────────────┐  ┌──────────────┐                 │
│  │ ▶ START      │  │ ■ STOP       │                 │
│  │   SYSTEM     │  │   SYSTEM     │                 │
│  └──────────────┘  └──────────────┘                 │
├──────────────────────────────────────────────────────┤
│  System Log                                          │
│  ┌────────────────────────────────────────────────┐ │
│  │ [12:34:56] [INFO] CSSR System Launcher init... │ │
│  │ [12:35:01] [INFO] Checking Jetson connection.. │ │
│  │ [12:35:02] [SUCCESS] ✓ Jetson is reachable     │ │
│  │ [12:35:05] [INFO] Starting system...           │ │
│  └────────────────────────────────────────────────┘ │
├──────────────────────────────────────────────────────┤
│  CSSR4Africa Robot System | Ready              ●    │
└──────────────────────────────────────────────────────┘
```

---
## Support

For issues or questions:
- Create an issue on GitHub
- Contact: Ibrahim Jimoh - <a href="mailto:ioj@alumni.cmu.edu">ioj@alumni.cmu.edu</a><br>
- Visit: <a href="http://www.cssr4africa.org">www.cssr4africa.org</a>
