#!/usr/bin/env python3
"""
EtherCAT Server - HTTP server for servo motor control (port 9982)

ARCHITECTURE: The EtherCAT PDO loop runs in a SEPARATE PROCESS (ec_motor_process.py)
with REALTIME priority, completely isolated from HTTP server GIL contention.
This eliminates jitter caused by Python's GIL sharing between the PDO thread
and HTTP request threads.

Communication:
- Shared memory for real-time status reads (zero latency, no bus access)
- Queue for commands (connect, move, enable, etc.)

Usage:
    python ec_server.py

TouchDesigner DNA controller sends HTTP requests to localhost:9982
"""

import sys
import os
import json
import time
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn

# Add python_soem app directory to path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SOEM_APP_DIR = os.path.join(SCRIPT_DIR, "python_soem-main", "app")
if os.path.exists(SOEM_APP_DIR):
    sys.path.insert(0, SOEM_APP_DIR)

try:
    import pysoem
    PYSOEM_AVAILABLE = True
except ImportError:
    print("WARNING: pysoem not installed. Install with: pip install pysoem")
    PYSOEM_AVAILABLE = False

from ec_motor_process import ECMotorProcess

# ============================================================
# GLOBAL STATE
# ============================================================
motor = None  # ECMotorProcess instance
_log_buffer = []
PORT = 9982


def seq_log(msg):
    """Log a message to buffer and stdout"""
    print(msg)
    _log_buffer.append(msg)
    if len(_log_buffer) > 200:
        _log_buffer.pop(0)


def drain_motor_logs():
    """Drain log messages from motor process into our log buffer"""
    if motor:
        for msg in motor.get_logs():
            _log_buffer.append(msg)
            if len(_log_buffer) > 200:
                _log_buffer.pop(0)


# ============================================================
# HTTP REQUEST HANDLER
# ============================================================

class ECHandler(BaseHTTPRequestHandler):
    """HTTP handler — thin wrapper over ECMotorProcess"""

    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "POST, GET, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):
        drain_motor_logs()

        if self.path == "/status":
            result = motor.get_status() if motor else {"connected": False, "slaves": []}
        elif self.path == "/adapters":
            resp = motor.send_command(ECMotorProcess.CMD_GET_ADAPTERS) if motor else None
            if resp and resp.get('success'):
                result = {"adapters": resp['data']['adapters']}
            else:
                result = {"adapters": []}
        elif self.path == "/ping":
            result = {"alive": True, "connected": bool(motor and motor.shared_connected.value)}
        elif self.path == "/logs":
            drain_motor_logs()
            result = {"logs": _log_buffer[-50:]}
        elif self.path == "/sequence_status":
            result = {"running": bool(motor and motor.shared_seq_running.value)}
        else:
            result = {"error": "Unknown endpoint"}

        self._send_json(result)

    def do_POST(self):
        content_len = int(self.headers.get("Content-Length", 0))
        body = {}
        if content_len > 0:
            raw = self.rfile.read(content_len)
            try:
                body = json.loads(raw)
            except:
                body = {}

        result = self._handle_post(body)
        drain_motor_logs()
        self._send_json(result)

    def _handle_post(self, body):
        if not motor:
            return {"error": "Motor process not running"}

        path = self.path

        # Commands that need a response (blocking)
        if path == "/connect":
            return motor.send_command(ECMotorProcess.CMD_CONNECT, {
                'adapter': body.get("adapter", ""),
                'mode': body.get("mode", "CSP"),
                'velocity': body.get("velocity", 80000),
                'accel': body.get("accel", 6000),
                'decel': body.get("decel", 6000),
                'csp_velocity': body.get("csp_velocity", 100),
                'steps_per_meter': body.get("steps_per_meter", 792914),
                'raw_steps_per_meter': body.get("raw_steps_per_meter", 202985985)
            }, timeout=20)

        elif path == "/disconnect":
            return motor.send_command(ECMotorProcess.CMD_DISCONNECT)

        elif path == "/enable_all":
            return motor.send_command(ECMotorProcess.CMD_ENABLE_ALL, timeout=15)

        elif path == "/disable_all":
            return motor.send_command(ECMotorProcess.CMD_DISABLE_ALL)

        elif path == "/enable":
            return motor.send_command(ECMotorProcess.CMD_ENABLE_DRIVE, {'slave': body.get("slave", 0)})

        elif path == "/reset_fault":
            return motor.send_command(ECMotorProcess.CMD_RESET_FAULTS)

        elif path == "/reset_all_faults":
            return motor.send_command(ECMotorProcess.CMD_RESET_FAULTS)

        elif path == "/home":
            return motor.send_command(ECMotorProcess.CMD_HOME)

        elif path == "/move":
            return motor.send_command(ECMotorProcess.CMD_MOVE, {
                'slave': body.get("slave", 0),
                'position': body.get("position", 0.0)
            })

        elif path == "/move_all":
            return motor.send_command(ECMotorProcess.CMD_MOVE_ALL, {
                'positions': body.get("positions", {})
            })

        elif path == "/stop":
            return motor.send_command(ECMotorProcess.CMD_STOP_ALL)

        elif path == "/set_speed":
            return motor.send_command(ECMotorProcess.CMD_SET_SPEED, {
                'slave': body.get("slave", 0),
                'velocity': body.get("velocity", 80000),
                'accel': body.get("accel"),
                'decel': body.get("decel")
            })

        elif path == "/set_csp_velocity":
            return motor.send_command(ECMotorProcess.CMD_SET_CSP_VELOCITY, {
                'slave': body.get("slave", 0),
                'velocity': body.get("velocity", 100)
            })

        elif path == "/velocity_forward":
            return motor.send_command(ECMotorProcess.CMD_VELOCITY_FORWARD, {
                'slave': body.get("slave", 0),
                'speed': body.get("speed", 80000)
            })

        elif path == "/velocity_backward":
            return motor.send_command(ECMotorProcess.CMD_VELOCITY_BACKWARD, {
                'slave': body.get("slave", 0),
                'speed': body.get("speed", 80000)
            })

        elif path == "/velocity_stop":
            return motor.send_command(ECMotorProcess.CMD_VELOCITY_STOP, {
                'slave': body.get("slave", 0)
            })

        elif path == "/set_home":
            return motor.send_command(ECMotorProcess.CMD_SET_HOME, {
                'slave': body.get("slave", 0)
            })

        elif path == "/set_home_all":
            return motor.send_command(ECMotorProcess.CMD_SET_HOME_ALL)

        elif path == "/run_sequence":
            return motor.send_command(ECMotorProcess.CMD_RUN_SEQUENCE, {
                'steps': body.get("steps", []),
                'loop_count': body.get("loop_count", 1),
                'loop_forever': body.get("loop_forever", False),
                'mode': body.get("mode", "CSP")
            })

        elif path == "/stop_sequence":
            return motor.send_command(ECMotorProcess.CMD_STOP_SEQUENCE)

        elif path == "/sequence_status":
            return {"running": bool(motor.shared_seq_running.value)}

        elif path == "/status":
            return motor.get_status()

        return {"error": f"Unknown endpoint: {path}"}

    def _send_json(self, result):
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(json.dumps(result).encode())

    def log_message(self, format, *args):
        pass  # Suppress default logging


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":
    import multiprocessing as mp
    mp.freeze_support()

    print("=" * 60)
    print(f"  EtherCAT Server starting on port {PORT}")
    print(f"  Architecture: SEPARATE PROCESS for PDO (jitter-free)")
    print(f"  pysoem available: {PYSOEM_AVAILABLE}")
    print(f"  python_soem path: {SOEM_APP_DIR}")
    print("=" * 60)

    # Start motor process
    motor = ECMotorProcess()
    motor.start()
    time.sleep(0.5)

    if PYSOEM_AVAILABLE:
        resp = motor.send_command(ECMotorProcess.CMD_GET_ADAPTERS)
        if resp and resp.get('success'):
            adapters = resp['data']['adapters']
            print(f"\n  Network adapters found: {len(adapters)}")
            for a in adapters:
                print(f"    {a['name']} - {a['desc']}")

    print(f"\n  Endpoints:")
    print(f"    GET  http://localhost:{PORT}/status")
    print(f"    GET  http://localhost:{PORT}/adapters")
    print(f"    POST http://localhost:{PORT}/connect")
    print(f"    POST http://localhost:{PORT}/move_all")
    print(f"    POST http://localhost:{PORT}/run_sequence")
    print("=" * 60)

    class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
        daemon_threads = True

    server = ThreadedHTTPServer(("0.0.0.0", PORT), ECHandler)
    print(f"\n  Server running on http://localhost:{PORT}")
    print("  Press Ctrl+C to stop\n")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n\nShutting down...")
        motor.stop()
        server.server_close()
        print("Server stopped.")
