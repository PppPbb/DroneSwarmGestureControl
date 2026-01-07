A gesture-controlled drone swarm simulation system​ based on UE4+AirSim, enabling real-time control of virtual drone formations through natural hand gestures.

settings.json show basic UAVswarm settings

HandGestureRecog.py for hand gesture recognization on mac, based on Mediapipe, python3.9
swarm_bridge.py for swarm control on windows, UE4.27 + python3.7 + Airsim1.8.1 + VS2022 (CityPark.sln)
Communication between macOS and Windows is implemented using OSC (Open Sound Control) protocol over UDP.
  <img width="20720" height="11864" alt="工作流" src="https://github.com/user-attachments/assets/936ecf5a-f66d-4384-8d53-1449073b5bb6" />

requirements and environments are not uploaded

<img width="2488" height="1437" alt="图片" src="https://github.com/user-attachments/assets/789024bb-38b1-4ee3-a8f0-4a7543101b54" />


# Supported Gesture Functions
| Category       | Function                               |
|----------------|----------------------------------------|
| Flight Control | Hover / Cruise Mode Switching          |
| Swarm Control  | Formation Transformation               |
| Speed Control  | Multi-level Velocity Adjustment        |
| Selection      | Single-Drone Deployment                |
| Safety         | Control Lock / Unlock                  |

# Swarm Formation Modes:
- **Triangle**: UAV2 as leader, UAV1/UAV3 follow at rear-left and rear-right
- **Line**: UAVs arranged longitudinally behind the leader
- **Row**: UAVs arranged laterally, side-by-side
UAV2 as leader, UAV1 and UAV3 as follower

# Hand Gesture Mapping
This system adopts a dual-hand gesture interaction paradigm for real-time drone swarm control.
- The **left hand** is primarily used for high-level commands, such as flight mode switching, formation reconfiguration, and system state control.
- The **right hand** is responsible for continuous motion control, including orientation and velocity modulation.
The current gesture set is **under active development and iterative refinement**.

# Each control message contains:
[ mode, speed, formation, drone_id, roll, pitch, yaw, throttle ]
- mode: 0 = Hover, 1 = Cruise
- speed: speed level (1–5)
- formation: 1 = triangle, 2 = line, 3 = row
- pitch: vertical motion
- yaw: turning command


