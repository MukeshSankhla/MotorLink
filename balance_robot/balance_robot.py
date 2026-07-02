#!/usr/bin/env python3
"""
balance_robot.py

Python script to control a self-balancing robot. 
It interfaces with:
1. An ESP32 via USB Serial, which streams IMU data (MPU-6050 pitch angle).
2. Two DFRobot M0601 motors (ID 0x01 Right, ID 0x02 Left) via an RS485 USB adapter.

Features:
- Runs a PID loop at the rate of the IMU stream (~100Hz).
- Starts a background Web Dashboard at http://localhost:8000.
- Web interface supports:
  - Live telemetry plotting and numeric gauges (Pitch, RPMs).
  - Virtual D-Pad and Keyboard (Arrow keys & WASD) remote control.
  - Live PID parameter tuning and limits adjustment.
  - Emergency safety disable and reset controls.
"""

import serial
import time
import sys
import argparse
import signal
import threading
import json
import urllib.parse
from http.server import BaseHTTPRequestHandler, HTTPServer

# --- Global Web Dashboard State ---
web_state = {
    "target_offset": 0.0,
    "turn_command": 0.0,
    "drive_command": 0.0,
    "pitch": 0.0,
    "target_rpm": 0.0,
    "right_speed": 0.0,
    "left_speed": 0.0,
    "kp": 1.2,
    "ki": 0.0,
    "kd": 0.03,
    "limit": 50,
    "slew_limit": 150.0,
    "safety_triggered": False,
    "reset_pid": False
}

# --- Web Dashboard HTML Template ---
HTML_PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>M0601 Balancing Robot Dashboard</title>
    <style>
        :root {
            --bg-color: #0f172a;
            --card-bg: rgba(30, 41, 59, 0.7);
            --border-color: rgba(255, 255, 255, 0.1);
            --text-color: #f8fafc;
            --accent-cyan: #06b6d4;
            --accent-pink: #d946ef;
            --success-green: #10b981;
            --error-red: #ef4444;
        }

        body {
            background-color: var(--bg-color);
            color: var(--text-color);
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
            margin: 0;
            padding: 20px;
            display: flex;
            flex-direction: column;
            align-items: center;
            min-height: 100vh;
        }

        header {
            text-align: center;
            margin-bottom: 25px;
        }

        h1 {
            font-size: 2.2rem;
            margin: 0;
            background: linear-gradient(135deg, var(--accent-cyan), var(--accent-pink));
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
            font-weight: 800;
        }

        .subtitle {
            color: #94a3b8;
            margin-top: 5px;
            font-size: 0.95rem;
        }

        .container {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 20px;
            max-width: 950px;
            width: 100%;
        }

        @media (max-width: 768px) {
            .container {
                grid-template-columns: 1fr;
            }
        }

        .card {
            background: var(--card-bg);
            border: 1px solid var(--border-color);
            border-radius: 16px;
            padding: 22px;
            backdrop-filter: blur(12px);
            box-shadow: 0 8px 32px 0 rgba(0, 0, 0, 0.3);
        }

        .card h2 {
            font-size: 1.35rem;
            margin-top: 0;
            margin-bottom: 18px;
            border-bottom: 1px solid var(--border-color);
            padding-bottom: 8px;
            color: var(--accent-cyan);
        }

        /* Telemetry Grid */
        .telemetry-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 15px;
        }

        .tele-box {
            background: rgba(15, 23, 42, 0.5);
            border: 1px solid var(--border-color);
            border-radius: 12px;
            padding: 12px;
            text-align: center;
        }

        .tele-box.full-width {
            grid-column: span 2;
        }

        .tele-label {
            font-size: 0.8rem;
            color: #94a3b8;
            text-transform: uppercase;
            letter-spacing: 0.05em;
        }

        .tele-val {
            font-size: 2rem;
            font-weight: 700;
            margin-top: 5px;
        }

        #pitch-val {
            color: var(--success-green);
        }

        /* Safety Banner */
        .status-banner {
            border-radius: 12px;
            padding: 12px;
            text-align: center;
            font-weight: 700;
            margin-bottom: 20px;
            display: flex;
            justify-content: center;
            align-items: center;
            gap: 8px;
        }

        .status-active {
            background: rgba(16, 185, 129, 0.15);
            border: 1px solid var(--success-green);
            color: var(--success-green);
        }

        .status-fault {
            background: rgba(239, 68, 68, 0.15);
            border: 1px solid var(--error-red);
            color: var(--error-red);
        }

        /* D-Pad Controls */
        .dpad-container {
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: 10px;
            margin-top: 10px;
        }

        .dpad-row {
            display: flex;
            gap: 10px;
        }

        .dpad-btn {
            width: 75px;
            height: 75px;
            border-radius: 16px;
            border: 1px solid var(--border-color);
            background: rgba(30, 41, 59, 0.6);
            color: var(--text-color);
            font-size: 1.5rem;
            cursor: pointer;
            display: flex;
            justify-content: center;
            align-items: center;
            user-select: none;
            transition: all 0.15s;
        }

        .dpad-btn:active, .dpad-btn.active {
            background: var(--accent-cyan);
            box-shadow: 0 0 15px var(--accent-cyan);
            color: #0f172a;
            transform: scale(0.95);
        }

        .dpad-btn.stop {
            background: rgba(239, 68, 68, 0.2);
            color: var(--error-red);
            border-color: var(--error-red);
        }

        .dpad-btn.stop:active, .dpad-btn.stop.active {
            background: var(--error-red);
            box-shadow: 0 0 15px var(--error-red);
            color: white;
        }

        .dpad-empty {
            width: 75px;
            height: 75px;
        }

        /* Forms & Inputs */
        .form-group {
            margin-bottom: 12px;
        }

        .form-group label {
            display: block;
            margin-bottom: 4px;
            font-size: 0.85rem;
            color: #94a3b8;
        }

        .form-group input {
            width: 100%;
            background: rgba(15, 23, 42, 0.6);
            border: 1px solid var(--border-color);
            border-radius: 8px;
            padding: 8px 10px;
            color: var(--text-color);
            font-size: 0.95rem;
            box-sizing: border-box;
        }

        .submit-btn {
            width: 100%;
            background: linear-gradient(135deg, var(--accent-cyan), var(--accent-pink));
            border: none;
            border-radius: 8px;
            padding: 10px;
            color: white;
            font-size: 0.95rem;
            font-weight: 700;
            cursor: pointer;
            transition: opacity 0.2s;
            margin-top: 5px;
        }

        .submit-btn:hover {
            opacity: 0.9;
        }

        .action-btn {
            background: rgba(255, 255, 255, 0.1);
            border: 1px solid var(--border-color);
            color: var(--text-color);
            border-radius: 6px;
            padding: 6px 12px;
            cursor: pointer;
            font-size: 0.85rem;
            font-weight: 700;
            transition: all 0.2s;
        }

        .action-btn:hover {
            background: rgba(255, 255, 255, 0.2);
        }

        .action-btn.reset-fault {
            border-color: var(--success-green);
            color: var(--success-green);
        }

        .action-btn.reset-fault:hover {
            background: rgba(16, 185, 129, 0.2);
        }

        .kb-instructions {
            text-align: center;
            font-size: 0.8rem;
            color: #64748b;
            margin-top: 15px;
        }
    </style>
</head>
<body>
    <header>
        <h1>M0601 Self-Balancing Robot</h1>
        <div class="subtitle">Real-time PID Web Dashboard</div>
    </header>

    <div class="container">
        <!-- Telemetry Card -->
        <div class="card">
            <h2>Telemetry Status</h2>
            
            <div id="status-banner" class="status-banner status-active">
                <span id="status-dot">🟢</span> <span id="status-text">Active & Balancing</span>
            </div>

            <div class="telemetry-grid">
                <div class="tele-box full-width">
                    <div class="tele-label">Pitch Angle</div>
                    <div id="pitch-val" class="tele-val">0.00°</div>
                </div>
                <div class="tele-box">
                    <div class="tele-label">Target speed</div>
                    <div id="target-rpm-val" class="tele-val">0.0 RPM</div>
                </div>
                <div class="tele-box">
                    <div class="tele-label">Safety Status</div>
                    <div id="safety-action-container" style="margin-top: 8px;">
                        <span id="safety-label" style="color:var(--success-green); font-weight:700;">OK</span>
                    </div>
                </div>
                <div class="tele-box">
                    <div class="tele-label">Right Motor speed</div>
                    <div id="right-rpm-val" class="tele-val">0.0 RPM</div>
                </div>
                <div class="tele-box">
                    <div class="tele-label">Left Motor speed</div>
                    <div id="left-rpm-val" class="tele-val">0.0 RPM</div>
                </div>
            </div>
        </div>

        <!-- D-Pad Controls -->
        <div class="card">
            <h2>Remote Control</h2>
            <div class="dpad-container">
                <div class="dpad-row">
                    <div class="dpad-empty"></div>
                    <div class="dpad-btn" id="btn-forward" data-dir="forward">▲</div>
                    <div class="dpad-empty"></div>
                </div>
                <div class="dpad-row">
                    <div class="dpad-btn" id="btn-left" data-dir="left">◀</div>
                    <div class="dpad-btn stop" id="btn-stop" data-dir="stop">■</div>
                    <div class="dpad-btn" id="btn-right" data-dir="right">▶</div>
                </div>
                <div class="dpad-row">
                    <div class="dpad-empty"></div>
                    <div class="dpad-btn" id="btn-backward" data-dir="backward">▼</div>
                    <div class="dpad-empty"></div>
                </div>
            </div>
            <div class="kb-instructions">
                Use **Arrow Keys** (▲ ▼ ◀ ▶) or keyboard **W A S D** to drive. Spacebar to stop.
            </div>
        </div>

        <!-- PID Tuning Config -->
        <div class="card">
            <h2>PID Configuration</h2>
            <form id="pid-form">
                <div style="display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 10px;">
                    <div class="form-group">
                        <label for="kp-input">Kp (Proportional)</label>
                        <input type="number" id="kp-input" step="0.1" min="0">
                    </div>
                    <div class="form-group">
                        <label for="ki-input">Ki (Integral)</label>
                        <input type="number" id="ki-input" step="0.01" min="0">
                    </div>
                    <div class="form-group">
                        <label for="kd-input">Kd (Derivative)</label>
                        <input type="number" id="kd-input" step="0.01" min="0">
                    </div>
                </div>
                <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 10px;">
                    <div class="form-group">
                        <label for="limit-input">Max RPM Limit</label>
                        <input type="number" id="limit-input" step="5" min="10" max="330">
                    </div>
                    <div class="form-group">
                        <label for="slew-input">Slew Limit (RPM/s)</label>
                        <input type="number" id="slew-input" step="10" min="50">
                    </div>
                </div>
                <button type="submit" class="submit-btn">Apply Tuning Gains</button>
            </form>
        </div>
    </div>

    <script>
        const API_TELEMETRY = "/api/telemetry";
        const API_MOVE = "/api/move";
        const API_SET_PID = "/api/set_pid";
        const API_RESET_SAFETY = "/api/reset_safety";

        let currentDirection = "stop";

        // 10Hz Telemetry Polling Loop
        setInterval(async () => {
            try {
                const res = await fetch(API_TELEMETRY);
                const data = await res.json();
                
                document.getElementById("pitch-val").innerText = data.pitch.toFixed(2) + "°";
                document.getElementById("target-rpm-val").innerText = data.target_rpm.toFixed(1) + " RPM";
                document.getElementById("right-rpm-val").innerText = data.right_speed.toFixed(1) + " RPM";
                document.getElementById("left-rpm-val").innerText = data.left_speed.toFixed(1) + " RPM";

                const pitchColor = Math.abs(data.pitch) > 15 ? "var(--error-red)" : (Math.abs(data.pitch) > 5 ? "var(--accent-pink)" : "var(--success-green)");
                document.getElementById("pitch-val").style.color = pitchColor;

                const banner = document.getElementById("status-banner");
                const text = document.getElementById("status-text");
                const dot = document.getElementById("status-dot");
                const actionContainer = document.getElementById("safety-action-container");

                if (data.safety_triggered) {
                    banner.className = "status-banner status-fault";
                    text.innerText = "SAFETY SHUTDOWN (Tilt limit exceeded!)";
                    dot.innerText = "🔴";
                    actionContainer.innerHTML = '<button type="button" class="action-btn reset-fault" onclick="resetSafety()">Reset Safety Interlock</button>';
                } else {
                    banner.className = "status-banner status-active";
                    text.innerText = "Active & Balancing";
                    dot.innerText = "🟢";
                    actionContainer.innerHTML = '<span style="color:var(--success-green); font-weight:700;">OK</span>';
                }

                // Fill forms if they are not active
                const kp = document.getElementById("kp-input");
                if (document.activeElement !== kp) kp.value = data.kp;
                const ki = document.getElementById("ki-input");
                if (document.activeElement !== ki) ki.value = data.ki;
                const kd = document.getElementById("kd-input");
                if (document.activeElement !== kd) kd.value = data.kd;
                const limit = document.getElementById("limit-input");
                if (document.activeElement !== limit) limit.value = data.limit;
                const slew = document.getElementById("slew-input");
                if (document.activeElement !== slew) slew.value = data.slew_limit;

            } catch (e) {
                console.error("Telemetry error", e);
            }
        }, 100);

        async function sendMove(direction) {
            if (currentDirection === direction) return;
            currentDirection = direction;
            
            document.querySelectorAll(".dpad-btn").forEach(b => b.classList.remove("active"));
            const activeBtn = document.getElementById("btn-" + direction);
            if (activeBtn) activeBtn.classList.add("active");

            try {
                await fetch(API_MOVE + "?direction=" + direction);
            } catch (e) {
                console.error(e);
            }
        }

        async function resetSafety() {
            try {
                await fetch(API_RESET_SAFETY);
            } catch (e) {
                console.error(e);
            }
        }

        document.getElementById("pid-form").addEventListener("submit", async (e) => {
            e.preventDefault();
            const kp = document.getElementById("kp-input").value;
            const ki = document.getElementById("ki-input").value;
            const kd = document.getElementById("kd-input").value;
            const limit = document.getElementById("limit-input").value;
            const slew = document.getElementById("slew-input").value;

            try {
                const res = await fetch(`${API_SET_PID}?kp=${kp}&ki=${ki}&kd=${kd}&limit=${limit}&slew_limit=${slew}`);
                if (res.ok) alert("PID parameters applied successfully!");
            } catch (err) {
                console.error(err);
                alert("Failed to update PID parameters.");
            }
        });

        // Keyboard Drive listeners
        const activeKeys = new Set();
        document.addEventListener("keydown", (e) => {
            const key = e.key.toLowerCase();
            if (activeKeys.has(key)) return;
            activeKeys.add(key);

            if (key === "arrowup" || key === "w") {
                sendMove("forward");
            } else if (key === "arrowdown" || key === "s") {
                sendMove("backward");
            } else if (key === "arrowleft" || key === "a") {
                sendMove("left");
            } else if (key === "arrowright" || key === "d") {
                sendMove("right");
            } else if (key === " ") {
                e.preventDefault();
                sendMove("stop");
            }
        });

        document.addEventListener("keyup", (e) => {
            const key = e.key.toLowerCase();
            activeKeys.delete(key);

            if (["arrowup", "w", "arrowdown", "s", "arrowleft", "a", "arrowright", "d"].includes(key)) {
                let stillMoving = false;
                for (let k of activeKeys) {
                    if (["arrowup", "w", "arrowdown", "s", "arrowleft", "a", "arrowright", "d"].includes(k)) {
                        stillMoving = true;
                        break;
                    }
                }
                if (!stillMoving) {
                    sendMove("stop");
                }
            }
        });

        // Click / Touch D-pad control hooks
        document.querySelectorAll(".dpad-btn").forEach(btn => {
            const dir = btn.getAttribute("data-dir");
            btn.addEventListener("mousedown", () => sendMove(dir));
            btn.addEventListener("touchstart", (e) => {
                e.preventDefault();
                sendMove(dir);
            });

            if (dir !== "stop") {
                btn.addEventListener("mouseup", () => sendMove("stop"));
                btn.addEventListener("mouseleave", () => sendMove("stop"));
                btn.addEventListener("touchend", () => sendMove("stop"));
            }
        });
    </script>
</body>
</html>
"""

# --- HTTP Server Handler ---
class DashboardHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        # Silence HTTP request logs in python console to keep output clean
        return

    def do_GET(self):
        parsed_url = urllib.parse.urlparse(self.path)
        path = parsed_url.path
        query = urllib.parse.parse_qs(parsed_url.query)
        
        if path == "/":
            self.send_response(200)
            self.send_header("Content-type", "text/html")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(HTML_PAGE.encode("utf-8"))
            
        elif path == "/api/telemetry":
            self.send_response(200)
            self.send_header("Content-type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            data = {
                "pitch": web_state["pitch"],
                "target_rpm": web_state["target_rpm"],
                "right_speed": web_state["right_speed"],
                "left_speed": web_state["left_speed"],
                "kp": web_state["kp"],
                "ki": web_state["ki"],
                "kd": web_state["kd"],
                "limit": web_state["limit"],
                "slew_limit": web_state["slew_limit"],
                "safety_triggered": web_state["safety_triggered"]
            }
            self.wfile.write(json.dumps(data).encode("utf-8"))
            
        elif path == "/api/move":
            direction = query.get("direction", ["stop"])[0]
            if direction == "forward":
                web_state["target_offset"] = -3  # Tilt forward (positive target angle)
                web_state["drive_command"] = -30.0  # Speed bias to drive forward
                web_state["turn_command"] = 0.0
            elif direction == "backward":
                web_state["target_offset"] = 3 # Tilt backward (negative target angle)
                web_state["drive_command"] = 30.0 # Speed bias to drive backward
                web_state["turn_command"] = 0.0
            elif direction == "left":
                web_state["turn_command"] = -15.0
                web_state["target_offset"] = 0.0
                web_state["drive_command"] = 0.0
            elif direction == "right":
                web_state["turn_command"] = 15.0
                web_state["target_offset"] = 0.0
                web_state["drive_command"] = 0.0
            else:
                web_state["target_offset"] = 0.0
                web_state["turn_command"] = 0.0
                web_state["drive_command"] = 0.0
                
            self.send_response(200)
            self.send_header("Content-type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(b'{"status":"ok"}')
            
        elif path == "/api/set_pid":
            try:
                if "kp" in query:
                    web_state["kp"] = float(query["kp"][0])
                if "ki" in query:
                    web_state["ki"] = float(query["ki"][0])
                if "kd" in query:
                    web_state["kd"] = float(query["kd"][0])
                if "limit" in query:
                    web_state["limit"] = int(query["limit"][0])
                if "slew_limit" in query:
                    web_state["slew_limit"] = float(query["slew_limit"][0])
            except ValueError:
                pass
                
            self.send_response(200)
            self.send_header("Content-type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(b'{"status":"ok"}')
            
        elif path == "/api/reset_safety":
            web_state["safety_triggered"] = False
            web_state["reset_pid"] = True
            self.send_response(200)
            self.send_header("Content-type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(b'{"status":"ok"}')
        else:
            self.send_response(404)
            self.end_headers()

# --- CRC8 Maxim Calculation ---
def crc8_maxim(data):
    """CRC-8/MAXIM: polynomial x^8 + x^5 + x^4 + 1, reflected 0x8C."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8C if (crc & 0x01) else (crc >> 1)
    return crc

# --- Motor Control Frame Builders ---
def build_frame(motor_id, cmd_byte, data_bytes):
    """Build a standard 10-byte M0601 frame: [ID, CMD, 7 data bytes, CRC]."""
    f = [motor_id, cmd_byte] + list(data_bytes)
    f.append(crc8_maxim(f))
    return bytes(f)

def frame_velocity_mode(motor_id):
    """Set the motor to Velocity Loop mode (mode 0x02)."""
    return bytes([motor_id, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02])

def frame_velocity(motor_id, rpm, accel=1):
    """Build velocity command frame. rpm: -330 to +330."""
    rpm = max(-330, min(330, int(rpm)))
    v = rpm.to_bytes(2, 'big', signed=True)
    return build_frame(motor_id, 0x64, [v[0], v[1], accel, 0x00, 0x00, 0x00, 0x00])

def frame_brake(motor_id):
    """Build electric brake command frame."""
    return build_frame(motor_id, 0x64, [0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00])

# --- PID Controller ---
class PIDController:
    def __init__(self, kp, ki, kd, target=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.target = target
        self.integral = 0.0
        self.last_error = 0.0
        
    def compute(self, current_value, gyro_rate, dt):
        """
        Compute PID output.
        Using direct gyro rate for D term is cleaner and less noisy than numeric differentiation.
        """
        error = current_value - self.target
        self.integral += error * dt
        
        # Anti-windup: clamp the integral term to prevent runaway error build-up
        self.integral = max(-50.0, min(50.0, self.integral))
        
        p_term = self.kp * error
        i_term = self.ki * self.integral
        d_term = self.kd * gyro_rate
        
        output = p_term + i_term + d_term
        return output

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0

def smooth_stop(ser_motor, current_speed_r, current_speed_l):
    """
    Ramps down the motors from their current speed to 0 RPM smoothly,
    then applies the electric brakes to lock the wheels.
    """
    print(f"\n[*] Smooth stop: deceleration ramp starting (R: {current_speed_r:.1f} RPM, L: {current_speed_l:.1f} RPM)")
    
    # Ramping down over 10 steps (20ms per step -> 200ms total deceleration)
    steps = 10
    for i in range(1, steps + 1):
        ratio = 1.0 - (i / steps)
        target_r = int(current_speed_r * ratio)
        target_l = int(current_speed_l * ratio)
        try:
            ser_motor.write(frame_velocity(0x01, target_r))
            time.sleep(0.001)
            ser_motor.write(frame_velocity(0x02, target_l))
            time.sleep(0.02)
        except Exception:
            break
            
    print("[*] Deceleration complete. Applying electric brakes to lock wheels.")
    for _ in range(5):
        try:
            ser_motor.write(frame_brake(0x01))
            time.sleep(0.01)
            ser_motor.write(frame_brake(0x02))
            time.sleep(0.01)
        except Exception:
            pass

# --- Main Program ---
def main():
    parser = argparse.ArgumentParser(description="Self-Balancing Robot Controller")
    parser.add_argument("--esp", type=str, default="COM10", help="ESP32 serial port (default: COM10)")
    parser.add_argument("--motor", type=str, default="COM13", help="Motor RS485 serial port (default: COM13)")
    parser.add_argument("--baud-esp", type=int, default=115200, help="Baud rate for ESP32 (default: 115200)")
    parser.add_argument("--baud-motor", type=int, default=115200, help="Baud rate for RS485 (default: 115200)")
    parser.add_argument("--kp", type=float, default=1.2, help="PID Proportional gain (default: 1.2)")
    parser.add_argument("--ki", type=float, default=0.0, help="PID Integral gain (default: 0.0)")
    parser.add_argument("--kd", type=float, default=0.03, help="PID Derivative gain (default: 0.03)")
    parser.add_argument("--target", type=float, default=0.0, help="Target balancing angle in degrees (offset calibration)")
    parser.add_argument("--limit", type=int, default=50, help="Max wheel speed in RPM (default: 50, limit: 330)")
    parser.add_argument("--safety-angle", type=float, default=35.0, help="Tilt angle threshold for emergency shutdown")
    parser.add_argument("--right-sign", type=int, default=-1, choices=[1, -1], help="Direction sign for Right Motor (default: -1)")
    parser.add_argument("--left-sign", type=int, default=1, choices=[1, -1], help="Direction sign for Left Motor (default: 1)")
    parser.add_argument("--show-raw", action="store_true", help="Print raw accelerometer and gyroscope values from MPU-6050")
    parser.add_argument("--slew-limit", type=float, default=150.0, help="Max wheel speed change rate in RPM/sec (default: 150.0)")
    args = parser.parse_args()

    # Sync argument defaults with the global web_state
    web_state["kp"] = args.kp
    web_state["ki"] = args.ki
    web_state["kd"] = args.kd
    web_state["limit"] = args.limit
    web_state["slew_limit"] = args.slew_limit

    print("=" * 60)
    print("           M0601 Self-Balancing Robot PID Controller")
    print("=" * 60)
    print(f"ESP32 Port:   {args.esp} (Baud: {args.baud_esp})")
    print(f"Motor Port:   {args.motor} (Baud: {args.baud_motor})")
    print(f"PID Params:   Kp={args.kp:.2f}, Ki={args.ki:.3f}, Kd={args.kd:.3f}")
    print(f"Balance Target: {args.target:.2f} deg")
    print(f"Safety Angle: +/- {args.safety_angle:.1f} deg")
    print(f"Slew Limit:   {args.slew_limit:.1f} RPM/sec")
    print(f"Right Wheel:  ID 0x01, Direction Sign: {args.right_sign}")
    print(f"Left Wheel:   ID 0x02, Direction Sign: {args.left_sign}")
    print("=" * 60)

    # Initialize serial ports
    try:
        ser_esp = serial.Serial(args.esp, args.baud_esp, timeout=0.5)
        print(f"[✓] Connected to ESP32 on {args.esp}")
    except serial.SerialException as e:
        print(f"[✗] Failed to open ESP32 serial port: {e}")
        sys.exit(1)

    try:
        ser_motor = serial.Serial(args.motor, args.baud_motor, timeout=0.1)
        print(f"[✓] Connected to RS485 Motor Bus on {args.motor}")
    except serial.SerialException as e:
        print(f"[✗] Failed to open RS485 serial port: {e}")
        ser_esp.close()
        sys.exit(1)

    # Initialize PID controller
    pid = PIDController(args.kp, args.ki, args.kd, args.target)

    # Active running flag
    running = True

    # Signal handler for clean exit on Ctrl+C
    def signal_handler(sig, frame):
        nonlocal running
        print("\n[!] Ctrl+C detected. Shutting down balancing loop...")
        running = False

    signal.signal(signal.SIGINT, signal_handler)

    # Start HTTP Web Server in background thread
    try:
        server = HTTPServer(("0.0.0.0", 8000), DashboardHandler)
        server_thread = threading.Thread(target=server.serve_forever, daemon=True)
        server_thread.start()
        print("[✓] Web Dashboard running at http://localhost:8000")
    except Exception as e:
        print(f"[⚠️] Could not start Web Server: {e}")

    # Set both motors to velocity loop mode on start
    print("[*] Setting motors to Velocity Loop Mode...")
    for _ in range(5):
        ser_motor.write(frame_velocity_mode(0x01))
        time.sleep(0.01)
        ser_motor.write(frame_velocity_mode(0x02))
        time.sleep(0.01)

    # Clear ESP32 input buffer to avoid lag/stale readings
    ser_esp.reset_input_buffer()
    
    last_time = time.time()
    last_dashboard_time = 0
    loop_count = 0
    last_commanded_speed = 0.0
    right_speed = 0.0
    left_speed = 0.0

    try:
        while running:
            # Read telemetry from ESP32
            line = ser_esp.readline()
            if not line:
                print("\n[CRITICAL] ESP32 Telemetry lost! Timing out...")
                break

            current_time = time.time()
            dt = current_time - last_time
            last_time = current_time

            dt = max(0.001, min(0.1, dt))

            try:
                line_str = line.decode('utf-8', errors='ignore').strip()
                
                parts = {}
                for item in line_str.split(','):
                    if ':' in item:
                        k, v = item.split(':')
                        parts[k] = float(v)
                
                pitch = parts.get("PITCH")
                gyro_y = parts.get("GYRO_Y") # Pitch rate (deg/s)
                
                if pitch is None or gyro_y is None:
                    continue
                    
            except (ValueError, IndexError):
                continue

            # Update live pitch in the web state
            web_state["pitch"] = pitch

            # Check if live PID reset requested
            if web_state.get("reset_pid"):
                pid.reset()
                last_commanded_speed = 0.0
                web_state["reset_pid"] = False

            # --- Safety Check: Emergency Stop ---
            if abs(pitch) > args.safety_angle:
                web_state["safety_triggered"] = True

            # Get user movement offsets from web server
            target_offset = web_state["target_offset"]
            turn_command = web_state["turn_command"]
            drive_command = web_state.get("drive_command", 0.0)

            # Update PID gains from Web Dashboard in real time
            pid.kp = web_state["kp"]
            pid.ki = web_state["ki"]
            pid.kd = web_state["kd"]
            active_limit = web_state["limit"]
            active_slew = web_state["slew_limit"]

            if web_state["safety_triggered"]:
                target_rpm = 0.0
                right_speed = 0.0
                left_speed = 0.0
                last_commanded_speed = 0.0
            else:
                # --- PID Control Computation ---
                pid.target = args.target + target_offset
                control_output = pid.compute(pitch, gyro_y, dt)

                # --- Stabilization Priority Control ---
                # If the robot tilts too far from the base balance point, we temporarily reduce
                # or zero out the drive speed bias, giving the PID loop 100% of the motor RPM headroom.
                error_from_neutral = abs(pitch - args.target)
                if error_from_neutral > 20.0:
                    active_drive = 0.0
                elif error_from_neutral > 10.0:
                    # Gradually fade out the drive bias between 10.0 and 20.0 degrees of tilt
                    scale = 1.0 - (error_from_neutral - 10.0) / 10.0
                    active_drive = drive_command * scale
                else:
                    active_drive = drive_command

                # Combine balancing PID output with the active drive speed bias
                target_rpm = control_output + active_drive

                # Limit speeds to prevent runaway
                target_rpm = max(-active_limit, min(active_limit, target_rpm))
                
                # --- Slew Rate Limiter to prevent sudden jumps ---
                max_change = active_slew * dt
                if target_rpm > last_commanded_speed + max_change:
                    target_rpm = last_commanded_speed + max_change
                elif target_rpm < last_commanded_speed - max_change:
                    target_rpm = last_commanded_speed - max_change
                last_commanded_speed = target_rpm

                # Apply motor direction signs and steering differential turn command
                right_speed = args.right_sign * (target_rpm - turn_command)
                left_speed = args.left_sign * (target_rpm + turn_command)

            # Sync speeds and target to the web state for dashboard feedback
            web_state["target_rpm"] = last_commanded_speed
            web_state["right_speed"] = right_speed
            web_state["left_speed"] = left_speed

            # Send velocity commands over RS485 bus
            try:
                ser_motor.write(frame_velocity(0x01, right_speed))
                time.sleep(0.001) 
                ser_motor.write(frame_velocity(0x02, left_speed))
            except serial.SerialException as e:
                print(f"\n[CRITICAL] RS485 communication failure: {e}")
                break

            # Print dashboard every ~0.1 seconds
            loop_count += 1
            if current_time - last_dashboard_time >= 0.1:
                last_dashboard_time = current_time
                if args.show_raw:
                    acc_str = f"Acc: {parts.get('ACC_X',0.0):+5.3f}X {parts.get('ACC_Y',0.0):+5.3f}Y {parts.get('ACC_Z',0.0):+5.3f}Z"
                    gyr_str = f"Gyr: {parts.get('GYRO_X',0.0):+6.1f}X {gyro_y:+6.1f}Y {parts.get('GYR_Z',0.0):+6.1f}Z"
                    print(f"\r[Loop: {loop_count:6d}] Pitch: {pitch:+6.2f}° | {acc_str} | {gyr_str}", end="", flush=True)
                else:
                    status_lbl = "🔴 SAFETY TRIGGERED" if web_state["safety_triggered"] else "🟢 ACTIVE"
                    print(f"\r[Loop: {loop_count:6d}] [{status_lbl}] Pitch: {pitch:+6.2f}° | Target RPM: {last_commanded_speed:+6.1f} | R: {right_speed:+5.1f} | L: {left_speed:+5.1f}", end="", flush=True)

    except Exception as e:
        print(f"\n[ERROR] Unexpected error in control loop: {e}")
        
    finally:
        # Stop motors smoothly and apply brakes
        smooth_stop(ser_motor, right_speed, left_speed)

        # Close serial ports safely
        try:
            ser_esp.close()
            print("[✓] ESP32 Serial Port closed.")
        except Exception:
            pass

        try:
            ser_motor.close()
            print("[✓] RS485 Motor Serial Port closed.")
        except Exception:
            pass

        print("Done. Goodbye!")

if __name__ == "__main__":
    main()
