# OutNav

OutNav is an autonomous outdoor navigation framework designed for mobile robots operating in complex outdoor environments. It provides a modular and scalable navigation stack capable of real-time localization, mapping, obstacle detection, waypoint navigation, motion control, and sensor fusion for reliable autonomous mobility. 
<img width="109" height="39" alt="image" src="https://github.com/user-attachments/assets/149a60ba-23b9-4c6c-add2-2146f9acfeb9" />

----

## Table of Contents

- [Launching the App](#launching-the-app)
- [Main Interface Overview](#main-interface-overview)
- [GPS Tracking Panel](#gps-tracking-panel)
- [Interactive Map](#interactive-map)
- [Drawing Waypoints & Routes](#drawing-waypoints--routes)
- [Road Routing (OSRM)](#road-routing-osrm)
- [Mission Planning](#mission-planning)
- [Camera Feed](#camera-feed)
- [Emergency Stop](#emergency-stop)
- [Audio Alerts](#audio-alerts)
- [Settings & Configuration](#settings--configuration)
- [Robot Mode ](#Robot-Mode)
- [Troubleshooting](#troubleshooting)

---

### Use  GPS-Localization-Filter-with-EMA-EKF-Ros2 Repsoitory For Localization/Pose
    https://github.com/saitejavarma801/GPS-Localization-Filter-with-EMA-EKF-Ros2.git
---
## Launching the App

### Laptop

Open a terminal, navigate to the OutNav folder, and run:

```bash
source /opt/ros/humble/setup.bash
cd outnav
./run.sh
```

The app window will open. Give it a few seconds — it initializes the ROS2 node and loads the map before the UI becomes fully interactive.

<img width="354" height="65" alt="image" src="https://github.com/user-attachments/assets/2279aa83-6572-4f80-9ac5-20a446428527" />


### What happens at startup

1. ROS2 context is initialized by `app.py`
2. The main PyQt5 window loads (`main_window.py`)
3. The Leaflet map is rendered inside the embedded browser panel
4. GPS, camera, and LiDAR subscriber nodes start listening
5. Audio system initializes — you will hear a startup chime if audio is enabled

---

## Main Interface Overview

The OutNav window is divided into several panels. Here is a quick map of the layout:

```
┌──────────────────────────────────────────────────────┐
│                    Top Bar                           │
│        [ GPS Status ]    [ E-STOP ]                  │
├─────────────────────────┬────────────────────────────┤
│                         │                            │
│      Interactive        │     Camera Feed            │
│         Map             │                            │
│                         ├────────────────────────────┤
│                         │   Mission Control Panel    │
│                         │                            │
└─────────────────────────┴────────────────────────────┘
```
<img width="1850" height="1053" alt="image" src="https://github.com/user-attachments/assets/b964aea3-0244-47b1-a942-e0b8a66e1379" />



| Panel | What it does |
|---|---|
| Top bar | Shows GPS lock status and hosts the Emergency Stop button |
| Map | The main navigation workspace — draw waypoints, view live position |
| Camera Feed | Streams live video from the robot's onboard camera |
| Mission Control | Create, load, and execute missions; monitor progress |

---

## GPS Tracking Panel

The GPS panel is always visible in the top section of the interface. It updates in real time as the robot moves.

<!-- IMAGE: Close-up screenshot of the GPS status panel showing all fields -->



### What each field means

| Field | Description |
|---|---|
| **Latitude / Longitude** | The robot's current GPS coordinates in decimal degrees |
| **Satellites** | Number of satellites currently locked. Higher is better — aim for 6 or more for reliable navigation |
| **HDOP** | Horizontal Dilution of Precision. A lower value means higher accuracy. HDOP below 1.5 is excellent; above 5 is poor |
| **Heading** | Direction the robot is facing in degrees (0° = North, 90° = East, 180° = South, 270° = West) |
| **Status indicator** | Green dot = GPS stable and locked. Red dot = GPS signal lost |

### GPS audio alerts

OutNav plays audio cues automatically based on GPS health:

- **GPS Stable** — plays when a good fix is acquired after startup or after a signal loss
- **GPS Lost** — plays when the satellite lock is dropped. The robot will stop autonomous movement until GPS is restored

 <img width="423" height="67" alt="image" src="https://github.com/user-attachments/assets/d70f82ca-9c6a-45a1-bc6d-3a6ff4db31a7" />



> ⚠️ **Always wait for GPS Stable before starting a mission.** Starting a mission with a poor fix can cause the robot to navigate to incorrect positions.


**Check the settings page for the minimum Satellites to Start **

---

## Interactive Map

The map is the central workspace of OutNav. It is powered by Leaflet.js and runs inside an embedded browser (QWebEngine). Everything on the map communicates back to the ROS2 node in real time via the QWebChannel bridge.

 <img width="1850" height="1053" alt="Screenshot from 2025-12-16 11-41-17" src="https://github.com/user-attachments/assets/20400831-caa0-4e4d-af81-2dd3b424526f" />



### Map controls

| Action | How to do it |
|---|---|
| Pan | Click and drag |
| Zoom in / out | Scroll wheel, or the +/− buttons in the top-left corner |


### Live robot position

A marker on the map shows the robot's current GPS position. It updates continuously as `/fix` messages arrive. A heading arrow on the marker shows the direction the robot is facing (sourced from `/gps/heading`).


### Live path trail

As the robot moves, OutNav draws the path it has travelled directly on the map. This lets you review the route taken during a mission. The trail is cleared each time a new mission starts.

<img width="1219" height="696" alt="image" src="https://github.com/user-attachments/assets/1fce10dc-0c79-4562-96ca-af2a4cc5df45" />


---

## Drawing Waypoints & Routes

Waypoints are positions on the map that you want the robot to visit. You draw them directly on the map before starting a mission.

<img width="1850" height="1053" alt="image" src="https://github.com/user-attachments/assets/f29ef2df-03d3-4b8b-8bb9-84f0123752b2" />


### Adding a waypoint

1. Click the Action Button on the left Panel.
2. Choose the Waypoints Mode (Point,Circle,Square, Rectangel and Boundary).
3. Set the Path mode (Inside/ Perimeter),Grid Spacing and Grid Angle.
4. Draw the Waypoints on the map and the outnav automatically generates the waypoint Inside or around the Perimeter based on the Grid Spacing and the Angle.

<img width="1850" height="1053" alt="image" src="https://github.com/user-attachments/assets/efef0aad-fc75-445f-9724-2f8f1c1c4fad" />



### Reordering waypoints

Click and drag any waypoint marker to move it to a new position on the map. The route line between waypoints updates automatically.

### Removing a waypoint

Right-click a waypoint marker and select **Remove**, or click the marker and press the **Delete** button in the waypoint panel.

### Clearing all waypoints

Click **Clear Route** in the mission toolbar to remove all waypoints and start fresh.


> 📌 **Tip:** Plan your route with the real-world environment in mind. Check for obstacles, slopes, and GPS-shadowed areas (tall buildings, tree cover) before sending.

---
### Road Routing (OSRM)

OutNav supports automatic road-following navigation using the OSRM (Open Source Routing Machine) public API. Instead of navigating in a straight line between waypoints, the robot follows real roads — useful when operating on or near public paths, tracks, or mapped terrain.

<img width="1850" height="1053" alt="image" src="https://github.com/user-attachments/assets/12845684-ac57-4451-bd4e-f1110cac4741" />

How it works

When you request an OSRM route, OutNav contacts the public OSRM server (router.project-osrm.org), which calculates the fastest road path between your home position and the chosen destination. The full road geometry is returned as a series of GPS coordinates, drawn on the map as a blue polyline, and sent to the robot over ROS2.

Step-by-step
1. Save a destination using the Places panel

Open the Places panel on the map toolbar
Drop a pin anywhere on the map at your target location
Give it a name and click Save
Saved places appear as named markers on the map and in the Places list
<img width="333" height="861" alt="image" src="https://github.com/user-attachments/assets/971038b2-368f-4d81-8920-8768f93c7f50" />

2. Set the navigation mode to Roads

Find the Mode dropdown in the navigation toolbar
Change the MOde from Waypoints to Roads (auto route)
<img width="333" height="300" alt="image" src="https://github.com/user-attachments/assets/0bf2ebe5-d507-49cc-b93b-166acc0bd809" />


3. Click Go

Select your saved destination from the Places list
Click Go
OutNav sends a routing request to router.project-osrm.org with your current home coordinates as the origin and the saved place as the destination
Within a second or two the route is calculated and drawn on the map as a blue polyline

4. Route is sent to the robot

The full road path (all intermediate GPS points along the route, not just start and end) is published to the robot over ROS2
The robot follows the road geometry step by step rather than cutting across in a straight line

### Direct vs Roads mode — when to use each
ModeBest forDirect (straight line)Open fields, off-road terrain, agricultural land with no obstacles between waypointsRoads (auto route)Urban areas, mapped paths and tracks, situations where the robot must stay on roads

⚠️ Internet connection required. OSRM routing uses the public router.project-osrm.org server. The operator PC must have internet access when you click Go. The robot itself does not need internet — only the operator UI does, to fetch the route.


OSRM routes roads, not terrain. If your destination is in an area with no mapped roads nearby, OSRM will route to the nearest road point it can reach. Always review the blue polyline on the map before sending the robot.

---

## Mission Planning

A mission is a complete automated task for the robot — it combines a route with a program that runs at each waypoint.

<img width="1850" height="1053" alt="image" src="https://github.com/user-attachments/assets/5d28b6e9-3217-403c-92a4-b7c7070e7825" />



### Creating a mission

1. Draw your waypoints on the map (see above)
2. Open the **Mission Panel** on the right side of the screen
3. Give the mission a name
4. Add  python Program by clicking Add Programs
5. Click **Save Mission** to write it to the `missions/programs/` folder for later use
6. Click **Start Mission** to execute immediately
7. The Robot Performs the Python Program assigned at each Waypoint and then Moves to the Next Waypoint.

<img width="445" height="782" alt="image" src="https://github.com/user-attachments/assets/f8e4250a-1b7b-4e5f-97ea-3990f8f51a2e" />



### Loading a saved mission

1. Click **Load Mission** in the Mission Panel
2. Browse the list of saved missions
3. Select a mission — its waypoints will reload onto the map automatically
4. Click **Start Mission** to execute

<img width="473" height="61" alt="image" src="https://github.com/user-attachments/assets/ca9c975b-45b1-4292-8ea5-80c21163ba5c" />


### Mission progress

While a mission runs, the progress bar in the Mission Panel fills as the robot completes each waypoint. OutNav also plays audio milestones:

| Progress | Audio cue |
|---|---|
| 25% | "Robot Reached 25 % of the Waypoints" |
| 50% | "Robot Reached 50 % of the Waypoints" |
| 90% | "Robot Reached 90 % of the Waypoints" |
| 100% | "Robot Reached 100 % of the Waypoints" |


Progress messages are also published by the robot on `/mission_progress` and displayed in the status log at the bottom of the Mission Panel.

### Stopping a mission

Click **Stop Mission** at any time to halt the robot and abort the current mission. The robot will stop in place. This publishes `true` to `/stop_mission`.

> ⚠️ **Stop Mission** brings the robot to a controlled stop. For an immediate hardware-level stop, use the **Emergency Stop** button instead.

---

## Camera Feed

The camera panel streams live compressed video from the robot's onboard camera over the ROS2 topic `/image_raw/compressed`.

Enable the 2nd Camera From the Settings Page by clicking Enable Second Camera HUD and select the USB CAMERA 2. Choose the Port of the 2nd Camera.

<img width="471" height="347" alt="image" src="https://github.com/user-attachments/assets/80212797-dc90-42ca-a8ff-1b880594884b" />


### What you see

The feed displays what the robot's camera sees in real time. This is useful for:

- Visually confirming the robot's environment before starting a mission
- Monitoring for unexpected obstacles during navigation
- Verifying waypoint arrival at specific locations

### If the feed is black or not showing

- Confirm the camera node is running on the robot (`camera_worker.py`)
- Check that the robot and operator PC share the same `ROS_DOMAIN_ID`
- Verify the `/image_raw/compressed` topic is being published: `ros2 topic hz /image_raw/compressed`
- Check network bandwidth — compressed image streams require a stable connection between robot and PC
---

## Emergency Stop

The Emergency Stop (E-Stop) button is located prominently in the top bar of the interface. It is always visible regardless of which panel is in focus.

<!-- IMAGE: Screenshot of the E-Stop button in its normal (armed) state — large, red, clearly labeled -->

### How to trigger it

Click the red **E-STOP** button once. OutNav immediately publishes `true` to the `/emergency_stop` ROS2 topic. The robot's hardware layer receives this and cuts motor power instantly.

### What happens after an E-Stop

- The robot halts immediately (not a gradual stop — all motion ceases)
- Any active mission is aborted
- An audio alert plays: "Emergency stop activated"
- The button changes appearance to confirm the stop was sent

<!-- IMAGE: Screenshot of the E-Stop button after activation, showing the confirmed/pressed state -->

### Resuming after an E-Stop

1. Physically inspect the robot and confirm it is safe to resume
2. Click **Reset E-Stop** (the button label changes after activation)
3. Reload your mission and restart from the beginning or from a specific waypoint

> 🔴 **The E-Stop is for immediate hazards.** Use Stop Mission for planned halts. Do not use E-Stop as a routine way to end missions — it can cause abrupt mechanical stress.

---

## Audio Alerts

OutNav uses a set of MP3 voice alerts stored in the `audio files/` folder to keep the operator informed without needing to watch the screen constantly.

<!-- IMAGE: Screenshot of the audio settings toggle in the config panel -->

### Full list of audio alerts

| Event | Alert played |
|---|---|
| App startup | Startup chime |
| GPS lock acquired | "GPS stable" |
| GPS signal lost | "GPS lost" |
| Mission started | "Mission started" |
| Mission 25% complete | "Robot Reached 25 % of the Waypoints" |
| Mission 50% complete | "Robot Reached 50 % of the Waypoints" |
| Mission 90% complete | "Robot Reached 90 % of the Waypoints" |
| Mission complete | "Mission complete" |
| Mission stopped | "Mission stopped" |
| Obstacle detected (LiDAR) | "Obstacle detected" |
| Return to home initiated | "Returning to home" |
| Emergency stop activated | "Emergency stop activated" |

### Enabling or disabling audio

Audio can be toggled in the **Settings** panel. Set `"audio_enabled": false` in the config file to disable all alerts persistently.

---

## Settings & Configuration

OutNav stores its configuration in a JSON file that persists across restarts. You can edit settings either through the UI Settings panel or directly in the file.



### Settings panel

Open **Settings** from the top menu. Fields available:

| Setting | Description |
|---|---|
| Robot IP | The IP address of the robot on your local network |
| ROS Domain ID | Must match on both the operator PC and the robot |
| Camera Topic | ROS2 topic for the camera stream (default: `/image_raw/compressed`) |
| GPS Serial Port | Serial port for the NMEA GPS receiver (e.g. `/dev/ttyUSB0`) |
| GPS Baud Rate | Serial baud rate for the GPS receiver (default: `9600`) |
| Audio Enabled | Toggle all voice alerts on or off |


<img width="964" height="863" alt="image" src="https://github.com/user-attachments/assets/dae155f2-4cf2-4d04-be58-2ec20f4676d7" />

---

## Robot Mode 

Robot mode runs OutNav on the robot's onboard computer.

### Starting headless mode

```bash
cd outnav
./run.sh
```
---

## Troubleshooting

### Robot position not updating on the map

- Confirm the robot is publishing to `/fix`: `ros2 topic echo /fix`
- Check that `ROS_DOMAIN_ID` is the same on both machines
- Verify the robot and operator PC are on the same network segment

### Camera feed not showing

- Run `ros2 topic hz /image_raw/compressed` on the operator PC to check if messages are arriving
- Confirm `camera_worker.py` is running on the robot
- Check USB camera connection on the robot

### Mission not starting

- Ensure waypoints are placed and the route has been sent via **Send Route**
- Confirm GPS is stable (green indicator) before starting
- Check the ROS2 topic: `ros2 topic echo /start_mission`

### No audio alerts

- Check `"audio_enabled": true` in `config.json`
- Verify the `audio files/` folder contains the MP3 files
- Check system audio output is not muted

### GPS shows 0 satellites

- Check the GPS antenna has a clear view of the sky
- Confirm `gps_serial_node.py` is running and the serial port is correct
- Test the serial port directly: `cat /dev/ttyUSB0` — you should see NMEA sentences scrolling

### App crashes on startup

- Confirm ROS2 is sourced: `echo $ROS_DISTRO` should print `humble`
- Verify all Python dependencies are installed: `pip show PyQt5 PyQtWebEngine opencv-python tf-transformations`
- Check the log file in `outnav/logs/` for the error traceback


---

## License

**OutNav Proprietary Source License**
Copyright &copy; 2025 Sai Teja Varma. All rights reserved.

### 1. Ownership

This software, including its source code, documentation, algorithms, and all associated files (collectively, "OutNav"), is the sole and exclusive intellectual property of Sai Teja Varma ("the Author"). No ownership rights are transferred to any party through access to or use of this software.

### 2. Permitted Use

Subject to the terms and conditions of this License, the Author grants any individual or organisation a limited, non-exclusive, non-transferable, royalty-free right to:

- Install and operate OutNav for personal, academic, or internal non-commercial purposes;
- Access and study the source code for educational or evaluative purposes;
- Fork this repository and propose contributions exclusively through official pull requests directed to the original repository.

### 3. Restrictions

Unless expressly authorised in writing by the Author, the following actions are strictly prohibited:

- Redistributing, sublicensing, selling, or otherwise transferring OutNav or any derivative thereof to any third party;
- Modifying, adapting, translating, or creating derivative works based on OutNav for purposes other than contributing back to the original project;
- Incorporating OutNav or any portion of its source code into any commercial product, service, or offering;
- Removing, altering, or obscuring any copyright, trademark, or proprietary notice contained within OutNav.

### 4. Contributions

Contributions to OutNav are welcomed and encouraged. By submitting a pull request, patch, or any other form of contribution, the contributor:

- Affirms that the contribution is their original work and that they hold the right to submit it;
- Irrevocably assigns all intellectual property rights in the contribution to Sai Teja Varma;
- Agrees that the contribution shall be governed by the same terms as this License.

All accepted contributors will be acknowledged within the project.

### 5. Disclaimer of Warranties

OutNav is provided **"as is"**, without warranty of any kind, express or implied, including but not limited to warranties of merchantability, fitness for a particular purpose, or non-infringement. The Author makes no representations or guarantees regarding the reliability, accuracy, or suitability of this software for any specific use case.

### 6. Limitation of Liability

In no event shall the Author be liable for any direct, indirect, incidental, special, exemplary, or consequential damages — including but not limited to loss of data, loss of revenue, or business interruption — arising out of or in connection with the use or inability to use OutNav, even if advised of the possibility of such damages.

### 7. Governing Law

This License shall be governed by and construed in accordance with applicable intellectual property law. Any disputes arising under this License shall be subject to the exclusive jurisdiction of the courts competent for the Author's place of residence.

---

## Contact

For licensing inquiries, collaboration proposals, or permission requests, please contact the Author directly.

**Sai Teja Varma**
📧 [saitejavarma801@gmail.com](mailto:saitejavarma801@gmail.com)



