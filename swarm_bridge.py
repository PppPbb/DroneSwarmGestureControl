"""
File Name: swarm_bridge.py
Runs on: Windows (PC connected to AirSim)
Function: Receives commands from Mac, controls AirSim swarm, prevents heartbeat disconnection
"""
import airsim
import time
import numpy as np
import threading
import math
from pythonosc import dispatcher
from pythonosc import osc_server

# ================= Configuration Area =================
LISTEN_IP = "0.0.0.0"  # Allows LAN connections
LISTEN_PORT = 8000  # Must match the setting on the Mac side


# ================= Global Control Variables =================
class ControlState:
    def __init__(self):
        self.vx = 0.0  # Forward/backward velocity
        self.vz = 0.0  # Vertical velocity (New: Controlled by Pitch)
        self.yaw_rate = 0.0  # Yaw rate (Turning speed)
        self.formation = "triangle"
        self.flight_mode = 0  # 0=HOVER, 1=CRUISE
        self.speed_mult = 1  # Speed level 1-5


state = ControlState()


# ================= 1. OSC Message Handling (Adapts to new tactical logic) =================
def handle_osc_command(unused_addr, *args):
    """
    Receives data sent from Mac:
    [mode, speed, formation, drone_id, roll, pitch, yaw, throttle]
    """
    if len(args) < 8: return

    # 1. Unpack
    mode = int(args[0])
    speed_level = args[1]
    form_id = args[2]
    # args[3] is drone_id (temporarily ignored, defaults to swarm control)

    val_pitch = args[5]  # Now represents [Ascend/Descend command]
    val_yaw = args[6]  # Represents [Yaw/Turning command]

    # 2. Update state
    state.flight_mode = mode
    state.speed_mult = speed_level

    # --- Power Mapping (Mixer) ---

    # A. Forward Velocity (VX) - Determined solely by [Mode] (Cruise control)
    if state.flight_mode == 1:  # CRUISE
        # Speed formula: Base speed 0.8m/s * speed level
        state.vx = 1 * state.speed_mult
    else:
        state.vx = 0.0

    # B. Vertical Velocity (VZ) - Determined by [Right hand Pitch]
    # Mac sends: Negative = Pitch up (Ascend), Positive = Pitch down (Descend)
    # AirSim: Negative = Up, Positive = Down
    # Direct mapping, coefficient 0.08 (if used
