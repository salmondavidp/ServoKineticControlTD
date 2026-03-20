#!/usr/bin/env python3
"""
EC Motor Process - Runs EtherCAT in a SEPARATE PROCESS for jitter-free PDO timing.

The PDO cycle (1ms in CSP) runs in its own process with REALTIME priority,
completely isolated from the HTTP server's GIL contention.

Communication:
- Shared memory (Array/Value) for real-time status reads (zero latency)
- Queue for commands (connect, move, enable, etc.)
- Queue for responses

Based on motor_process.py from python_soem (Arsmi-17/python_soem)
"""

import multiprocessing as mp
from multiprocessing import Process, Queue, Value, Array
import ctypes
import time
import sys
import os
import threading

# Add python_soem app directory to path
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SOEM_APP_DIR = os.path.join(SCRIPT_DIR, "python_soem-main", "app")
if os.path.exists(SOEM_APP_DIR):
    sys.path.insert(0, SOEM_APP_DIR)


class ECMotorProcess:
    """EtherCAT motor control in a separate process"""

    # Commands
    CMD_CONNECT = 'connect'
    CMD_DISCONNECT = 'disconnect'
    CMD_ENABLE_ALL = 'enable_all'
    CMD_DISABLE_ALL = 'disable_all'
    CMD_RESET_FAULTS = 'reset_faults'
    CMD_ENABLE_DRIVE = 'enable_drive'
    CMD_MOVE = 'move'
    CMD_MOVE_ALL = 'move_all'
    CMD_HOME = 'home'
    CMD_SET_HOME = 'set_home'
    CMD_SET_HOME_ALL = 'set_home_all'
    CMD_STOP = 'stop'
    CMD_STOP_ALL = 'stop_all'
    CMD_SET_SPEED = 'set_speed'
    CMD_SET_CSP_VELOCITY = 'set_csp_velocity'
    CMD_VELOCITY_FORWARD = 'velocity_forward'
    CMD_VELOCITY_BACKWARD = 'velocity_backward'
    CMD_VELOCITY_STOP = 'velocity_stop'
    CMD_RUN_SEQUENCE = 'run_sequence'
    CMD_STOP_SEQUENCE = 'stop_sequence'
    CMD_GET_ADAPTERS = 'get_adapters'
    CMD_QUIT = 'quit'

    MAX_SLAVES = 8

    def __init__(self):
        # IPC Queues
        self.cmd_queue = Queue()
        self.resp_queue = Queue()

        # Shared memory for real-time status (read by HTTP server, written by motor process)
        # Layout: [0-7] positions_m, [8-15] status_words, [16-23] error_codes,
        #         [24] velocity, [25] accel, [26] decel, [27] csp_vel
        self.shared_data = Array(ctypes.c_double, 32)

        self.shared_connected = Value(ctypes.c_int, 0)
        self.shared_num_slaves = Value(ctypes.c_int, 0)
        self.shared_mode = Value(ctypes.c_int, 8)  # 1=PP, 3=PV, 8=CSP
        self.shared_stop = Value(ctypes.c_int, 0)
        self.shared_seq_running = Value(ctypes.c_int, 0)

        # Log buffer shared via queue (motor process sends log lines)
        self.log_queue = Queue()

        # Process handle
        self.process = None
        self._running = Value(ctypes.c_bool, False)

    def start(self):
        """Start the motor process"""
        if self.process and self.process.is_alive():
            return
        self._running.value = True
        self.process = Process(
            target=self._run_process,
            args=(
                self.cmd_queue, self.resp_queue, self.log_queue,
                self.shared_data, self.shared_connected,
                self.shared_num_slaves, self.shared_mode,
                self.shared_stop, self.shared_seq_running,
                self._running
            ),
            daemon=True
        )
        self.process.start()
        print(f"[EC Motor Process] Started (PID: {self.process.pid})")

    def stop(self):
        """Stop the motor process"""
        if self.process and self.process.is_alive():
            self._running.value = False
            self.cmd_queue.put({'cmd': self.CMD_QUIT})
            self.process.join(timeout=3.0)
            if self.process.is_alive():
                self.process.terminate()
        print("[EC Motor Process] Stopped")

    def send_command(self, cmd, data=None, timeout=15.0):
        """Send command and wait for response"""
        if cmd == self.CMD_STOP or cmd == self.CMD_STOP_ALL or cmd == self.CMD_STOP_SEQUENCE:
            self.shared_stop.value = 1

        self.cmd_queue.put({'cmd': cmd, 'data': data})

        # Wait for response
        try:
            resp = self.resp_queue.get(timeout=timeout)
            return resp
        except Exception:
            return {'success': False, 'error': 'Command timeout'}

    def send_command_nowait(self, cmd, data=None):
        """Send command without waiting for response (fire-and-forget)"""
        if cmd == self.CMD_STOP or cmd == self.CMD_STOP_ALL or cmd == self.CMD_STOP_SEQUENCE:
            self.shared_stop.value = 1
        self.cmd_queue.put({'cmd': cmd, 'data': data})

    def get_status(self):
        """Get real-time status from shared memory (NO bus access, instant)"""
        num_slaves = self.shared_num_slaves.value
        slaves = []
        for i in range(min(num_slaves, self.MAX_SLAVES)):
            pos_m = self.shared_data[i]
            status_word = int(self.shared_data[8 + i])
            error_code = int(self.shared_data[16 + i])

            enabled = (status_word & 0x006F) == 0x0027
            fault = bool(status_word & 0x0008)
            target_reached = bool(status_word & 0x0400)

            error_name = None
            if fault and error_code:
                try:
                    from ethercat_controller import EtherCATController
                    error_name = EtherCATController.KNOWN_ERROR_CODES.get(error_code)
                except:
                    pass

            slaves.append({
                'index': i,
                'enabled': enabled,
                'fault': fault,
                'target_reached': target_reached,
                'position_m': round(pos_m, 6),
                'status': f"0x{status_word:04X}",
                'error_code': error_code,
                'error_name': error_name
            })

        mode_map = {1: "PP", 3: "PV", 8: "CSP"}
        return {
            'connected': bool(self.shared_connected.value),
            'slaves': slaves,
            'mode': mode_map.get(self.shared_mode.value, "?")
        }

    def get_logs(self):
        """Drain log queue"""
        logs = []
        while True:
            try:
                logs.append(self.log_queue.get_nowait())
            except Exception:
                break
        return logs

    @staticmethod
    def _run_process(cmd_queue, resp_queue, log_queue,
                     shared_data, shared_connected,
                     shared_num_slaves, shared_mode,
                     shared_stop, shared_seq_running,
                     running):
        """Main motor process — runs EtherCAT with REALTIME priority"""

        # ====== SET REALTIME PRIORITY ======
        try:
            kernel32 = ctypes.windll.kernel32
            winmm = ctypes.windll.winmm
            winmm.timeBeginPeriod(1)

            current_process = kernel32.GetCurrentProcess()
            # Try REALTIME, fall back to HIGH
            if not kernel32.SetPriorityClass(current_process, 0x100):
                kernel32.SetPriorityClass(current_process, 0x80)
            kernel32.SetProcessPriorityBoost(current_process, True)

            # Prevent background throttling
            ES = 0x80000000 | 0x00000001 | 0x00000040
            kernel32.SetThreadExecutionState(ES)

            # Pin to cores 0+1 (PDO on core 0, status/cmd on core 1)
            kernel32.SetProcessAffinityMask(current_process, 3)  # 0b11 = cores 0,1
            proc_log(log_queue, "Process priority: REALTIME, pinned to cores 0+1")
        except Exception as e:
            proc_log(log_queue, f"Priority setup warning: {e}")

        # ====== IMPORTS (in child process) ======
        from ethercat_controller import EtherCATController
        from movement_controller import MovementController

        ec = None
        mc = None
        seq_stop_event = threading.Event()
        seq_running_flag = [False]
        seq_id = [0]
        fault_bit_counts = [0] * 8

        def send_resp(success, message='', data=None):
            resp_queue.put({'success': success, 'message': message, 'data': data, 'error': None if success else message})

        def safe_disconnect():
            """Safely disconnect and clean up EtherCAT"""
            nonlocal ec, mc
            if ec:
                try:
                    ec._pdo_running = False
                    time.sleep(0.05)
                except:
                    pass
                try:
                    ec.master.close()
                except:
                    pass
                try:
                    ec.connected = False
                except:
                    pass
            ec = None
            mc = None
            shared_connected.value = 0
            shared_num_slaves.value = 0

        def do_connect(adapter, mode, velocity, accel, decel, csp_velocity,
                       steps_per_meter, raw_steps_per_meter):
            """Connect to EtherCAT — creates fresh controller each time"""
            nonlocal ec, mc

            # Always disconnect first
            safe_disconnect()
            time.sleep(0.2)

            # Validate adapter
            if not adapter:
                return False, "No adapter specified"

            proc_log(log_queue, f"Connecting: adapter={adapter}, mode={mode}")

            # Create fresh controller
            ec = EtherCATController(interface=adapter)
            mode_map = {"PP": 1, "PV": 3, "CSP": 8}
            ec.mode = mode_map.get(mode.upper(), 8)
            ec._velocity = velocity
            ec._accel = accel
            ec._decel = decel
            ec.ACTUAL_STEPS_PER_METER = steps_per_meter
            ec.RAW_STEPS_PER_METER = raw_steps_per_meter
            ec.SCALE_FACTOR = steps_per_meter / raw_steps_per_meter

            if not ec.connect():
                ec = None
                return False, "Connection failed - no slaves found"

            # Create movement controller
            mc = MovementController(ec)
            mc.ACTUAL_STEPS_PER_METER = steps_per_meter
            mc.RAW_STEPS_PER_METER = raw_steps_per_meter
            mc.SCALE_FACTOR = steps_per_meter / raw_steps_per_meter
            mc._velocity = velocity
            mc._accel = accel
            mc._decel = decel
            mc._csp_max_velocity = csp_velocity
            mc._mode = ec.mode
            mc.initialize()

            for i in range(ec.slaves_count):
                ec._slave_csp_velocity[i] = csp_velocity

            shared_connected.value = 1
            shared_num_slaves.value = ec.slaves_count
            shared_mode.value = ec.mode

            proc_log(log_queue, f"Connected: {ec.slaves_count} slaves, mode={mode}")
            return True, f"Connected: {ec.slaves_count} slaves"

        def update_shared_status():
            """Write current status to shared memory — PDO cached reads ONLY, no SDO"""
            if not ec or not ec.connected:
                return
            for i in range(min(ec.slaves_count, 8)):
                pos_m = ec.read_position_meters(i)
                status_word = ec.read_status(i)
                shared_data[i] = pos_m
                shared_data[8 + i] = float(status_word)

                # Track fault bit — but NEVER do SDO read here (causes jitter)
                if status_word & 0x0008:
                    fault_bit_counts[i] += 1
                    if fault_bit_counts[i] >= 20:
                        if shared_data[16 + i] == 0.0:
                            shared_data[16 + i] = 0xFFFF  # generic fault marker
                else:
                    fault_bit_counts[i] = 0
                    shared_data[16 + i] = 0.0

            shared_mode.value = ec.mode
            shared_data[24] = float(ec._velocity)
            shared_data[25] = float(ec._accel)
            shared_data[26] = float(ec._decel)
            csp_vel = ec._slave_csp_velocity.get(0, ec.DEFAULT_CSP_VELOCITY)
            shared_data[27] = float(csp_vel)

        def run_sequence_thread(steps, loop_count, loop_forever, mode, my_seq_id):
            """Run motion sequence in a thread within the motor process"""
            nonlocal ec, mc
            seq_stop_event.clear()
            shared_seq_running.value = 1
            seq_running_flag[0] = True
            shared_stop.value = 0

            def _wait(secs):
                elapsed = 0
                while elapsed < secs:
                    if seq_stop_event.is_set() or shared_stop.value == 1:
                        return True
                    time.sleep(min(0.1, secs - elapsed))
                    elapsed += 0.1
                return False

            cur_mode = ec.mode if ec else 1
            proc_log(log_queue, f"Sequence start: mode={cur_mode}, {len(steps)} steps")

            try:
                iteration = 0
                while True:
                    if not loop_forever and iteration >= loop_count:
                        break
                    for step_idx, step in enumerate(steps):
                        if seq_stop_event.is_set() or shared_stop.value == 1:
                            proc_log(log_queue, "Sequence stopped by user")
                            return

                        if cur_mode == 3:  # PV mode
                            slave_sel = step.get("slave", "all")
                            action = step.get("action", "forward")
                            speed = int(step.get("speed", 80000))
                            duration = float(step.get("duration", 2))

                            if slave_sel == "all":
                                slaves = list(range(ec.slaves_count)) if ec else []
                            else:
                                slaves = [int(slave_sel)] if ec else []

                            for i in slaves:
                                if action == "forward":
                                    ec.velocity_forward(i, speed)
                                elif action == "backward":
                                    ec.velocity_backward(i, speed)
                                elif action == "stop":
                                    ec.velocity_stop(i)

                            if _wait(duration):
                                return

                            for i in slaves:
                                try:
                                    ec.velocity_stop(i)
                                except:
                                    pass

                        elif cur_mode == 1:  # PP mode
                            vel = step.get("velocity")
                            acc = step.get("accel")
                            dec = step.get("decel")
                            if vel is not None and mc:
                                for i in range(ec.slaves_count):
                                    try:
                                        mc.set_speed(i, int(vel), int(acc) if acc else None, int(dec) if dec else None)
                                    except Exception as e:
                                        proc_log(log_queue, f"PP set_speed error slave {i}: {e}")
                            positions = step.get("positions", {})
                            pos = {int(k): float(v) for k, v in positions.items()}
                            if pos and mc:
                                try:
                                    mc.move_all_to_meters(pos, wait=True)
                                except Exception as e:
                                    proc_log(log_queue, f"PP move error: {e}")
                            hold = float(step.get("hold", 0))
                            if hold > 0 and _wait(hold):
                                return

                        elif cur_mode == 8:  # CSP mode
                            csp_vel = step.get("csp_velocity", 100)
                            slave_sel = step.get("slave", "all")
                            if ec:
                                if slave_sel == "all":
                                    slaves = list(range(ec.slaves_count))
                                else:
                                    slaves = [int(slave_sel)]
                                for i in slaves:
                                    try:
                                        ec._slave_csp_velocity[i] = int(csp_vel)
                                    except:
                                        pass

                            # Move S -> M -> hold -> E
                            for phase, phase_key in [("S", "pos_s"), ("M", "pos_m"), ("E", "pos_e")]:
                                if seq_stop_event.is_set() or shared_stop.value == 1:
                                    return
                                phase_pos = step.get(phase_key, {})
                                if not phase_pos:
                                    if phase == "S":
                                        phase_pos = step.get("positions", {})
                                    else:
                                        continue
                                pos = {int(k): float(v) for k, v in phase_pos.items()}
                                if slave_sel != "all":
                                    sid = int(slave_sel)
                                    if sid in pos:
                                        pos = {sid: pos[sid]}
                                    elif 0 in pos:
                                        pos = {sid: pos[0]}
                                    else:
                                        continue
                                proc_log(log_queue, f"Step {step_idx+1} [{phase}]: CSP vel={csp_vel} -> {pos}")
                                if pos and mc:
                                    try:
                                        mc.move_all_to_meters(pos, wait=True)
                                    except Exception as e:
                                        proc_log(log_queue, f"{phase} move error: {e}")

                                # Hold after M phase
                                if phase == "M":
                                    hold = float(step.get("hold", 0))
                                    if hold > 0 and _wait(hold):
                                        return

                    iteration += 1
                    proc_log(log_queue, f"Sequence loop {iteration} complete")

                # PV: stop all at end
                if cur_mode == 3 and ec:
                    for i in range(ec.slaves_count):
                        try:
                            ec.velocity_stop(i)
                        except:
                            pass
                proc_log(log_queue, f"Sequence completed ({iteration} loops)")

            except Exception as e:
                proc_log(log_queue, f"Sequence error: {e}")
                import traceback
                traceback.print_exc()
            finally:
                if seq_id[0] == my_seq_id:
                    shared_seq_running.value = 0
                    seq_running_flag[0] = False

        # ====== STATUS UPDATE THREAD ======
        def status_updater():
            try:
                import ctypes as ct
                handle = ct.windll.kernel32.GetCurrentThread()
                ct.windll.kernel32.SetThreadPriority(handle, -1)  # BELOW_NORMAL
            except:
                pass
            while running.value:
                try:
                    update_shared_status()
                except:
                    pass
                time.sleep(0.1)  # 100ms update rate

        status_thread = threading.Thread(target=status_updater, daemon=True)
        status_thread.start()

        # ====== MAIN COMMAND LOOP ======
        try:
            import ctypes as ct2
            handle = ct2.windll.kernel32.GetCurrentThread()
            ct2.windll.kernel32.SetThreadPriority(handle, -1)  # BELOW_NORMAL
        except:
            pass

        proc_log(log_queue, "Motor process ready, waiting for commands...")

        while running.value:
            try:
                cmd_data = cmd_queue.get(timeout=0.2)
            except:
                continue

            cmd = cmd_data.get('cmd')
            data = cmd_data.get('data') or {}

            try:
                if cmd == ECMotorProcess.CMD_QUIT:
                    proc_log(log_queue, "Quit command received")
                    break

                elif cmd == ECMotorProcess.CMD_GET_ADAPTERS:
                    import pysoem
                    adapters = pysoem.find_adapters()
                    result = [{"name": a.name, "desc": a.desc.decode() if isinstance(a.desc, bytes) else str(a.desc)} for a in adapters]
                    send_resp(True, f"Found {len(result)} adapters", {'adapters': result})

                elif cmd == ECMotorProcess.CMD_CONNECT:
                    adapter = data.get('adapter', '')
                    mode = data.get('mode', 'CSP')
                    velocity = data.get('velocity', 80000)
                    accel = data.get('accel', 6000)
                    decel = data.get('decel', 6000)
                    csp_velocity = data.get('csp_velocity', 100)
                    steps_per_meter = data.get('steps_per_meter', 792914)
                    raw_steps_per_meter = data.get('raw_steps_per_meter', 202985985)

                    success, msg = do_connect(
                        adapter, mode, velocity, accel, decel, csp_velocity,
                        steps_per_meter, raw_steps_per_meter
                    )
                    if success:
                        send_resp(True, msg, {
                            'slaves': ec.slaves_count, 'mode': mode.upper()
                        })
                    else:
                        send_resp(False, msg)

                elif cmd == ECMotorProcess.CMD_DISCONNECT:
                    safe_disconnect()
                    proc_log(log_queue, "Disconnected")
                    send_resp(True, "Disconnected")

                elif cmd == ECMotorProcess.CMD_ENABLE_ALL:
                    if ec and ec.connected:
                        ec.enable_all()
                        send_resp(True, "All drives enabled")
                    else:
                        send_resp(False, "Not connected")

                elif cmd == ECMotorProcess.CMD_DISABLE_ALL:
                    if ec and ec.connected:
                        ec.disable_all()
                        send_resp(True, "All drives disabled")
                    else:
                        send_resp(False, "Not connected")

                elif cmd == ECMotorProcess.CMD_RESET_FAULTS:
                    if ec and ec.connected:
                        for i in range(ec.slaves_count):
                            ec.reset_fault(i)
                            time.sleep(0.05)
                        send_resp(True, "Faults reset")
                    else:
                        send_resp(False, "Not connected")

                elif cmd == ECMotorProcess.CMD_ENABLE_DRIVE:
                    idx = data.get('slave', 0)
                    if ec and ec.connected:
                        ec.enable_drive(idx)
                        send_resp(True, f"Drive {idx} enabled")
                    else:
                        send_resp(False, "Not connected")

                elif cmd == ECMotorProcess.CMD_MOVE:
                    slave = data.get('slave', 0)
                    position = data.get('position', 0)
                    if mc:
                        mc.move_to_meters(slave, position)
                        send_resp(True, f"Moving slave {slave} to {position}m")
                    else:
                        send_resp(False, "Not connected")

                elif cmd == ECMotorProcess.CMD_MOVE_ALL:
                    positions = data.get('positions', {})
                    if mc:
                        pos = {int(k): float(v) for k, v in positions.items()}
                        mc.move_all_to_meters(pos, wait=False)
                        send_resp(True, f"Moving to positions: {pos}")
                    else:
                        send_resp(False, "Not connected")

                elif cmd == ECMotorProcess.CMD_HOME:
                    if mc:
                        mc.home_all()
                        send_resp(True, "Homing all slaves")
                    else:
                        send_resp(False, "Not connected")

                elif cmd == ECMotorProcess.CMD_SET_HOME:
                    slave = data.get('slave', 0)
                    if ec and ec.connected:
                        ec.set_home_position(slave)
                        send_resp(True, f"Home set for slave {slave}")
                    else:
                        send_resp(False, "Not connected")

                elif cmd == ECMotorProcess.CMD_SET_HOME_ALL:
                    if ec and ec.connected:
                        for i in range(ec.slaves_count):
                            ec.set_home_position(i)
                        send_resp(True, "Home set for all slaves")
                    else:
                        send_resp(False, "Not connected")

                elif cmd == ECMotorProcess.CMD_STOP:
                    if ec and ec.connected:
                        for i in range(ec.slaves_count):
                            ec.stop_motion(i)
                        send_resp(True, "All motion stopped")
                    else:
                        send_resp(False, "Not connected")

                elif cmd == ECMotorProcess.CMD_STOP_ALL:
                    shared_stop.value = 1
                    seq_stop_event.set()
                    if ec and ec.connected:
                        for i in range(ec.slaves_count):
                            try:
                                ec.stop_motion(i)
                            except:
                                pass
                    send_resp(True, "Emergency stop")

                elif cmd == ECMotorProcess.CMD_SET_SPEED:
                    slave = data.get('slave', 0)
                    velocity = data.get('velocity', 80000)
                    accel = data.get('accel')
                    decel = data.get('decel')
                    if mc:
                        mc.set_speed(slave, velocity, accel, decel)
                        send_resp(True, f"Speed set for slave {slave}")
                    else:
                        send_resp(False, "Not connected")

                elif cmd == ECMotorProcess.CMD_SET_CSP_VELOCITY:
                    slave = data.get('slave', 0)
                    velocity = data.get('velocity', 100)
                    if ec and ec.connected:
                        ec._slave_csp_velocity[slave] = velocity
                        send_resp(True, f"CSP velocity set for slave {slave}: {velocity}")
                    else:
                        send_resp(False, "Not connected")

                elif cmd == ECMotorProcess.CMD_VELOCITY_FORWARD:
                    slave = data.get('slave', 0)
                    speed = data.get('speed', 80000)
                    if ec and ec.connected:
                        ec.velocity_forward(slave, speed)
                        send_resp(True, f"Slave {slave} forward at {speed}")
                    else:
                        send_resp(False, "Not connected")

                elif cmd == ECMotorProcess.CMD_VELOCITY_BACKWARD:
                    slave = data.get('slave', 0)
                    speed = data.get('speed', 80000)
                    if ec and ec.connected:
                        ec.velocity_backward(slave, speed)
                        send_resp(True, f"Slave {slave} backward at {speed}")
                    else:
                        send_resp(False, "Not connected")

                elif cmd == ECMotorProcess.CMD_VELOCITY_STOP:
                    slave = data.get('slave', 0)
                    if ec and ec.connected:
                        ec.velocity_stop(slave)
                        send_resp(True, f"Slave {slave} velocity stopped")
                    else:
                        send_resp(False, "Not connected")

                elif cmd == ECMotorProcess.CMD_RUN_SEQUENCE:
                    if not ec or not ec.connected or not mc:
                        send_resp(False, "Not connected")
                        continue

                    # Stop any existing sequence
                    if seq_running_flag[0]:
                        seq_stop_event.set()
                        shared_stop.value = 1
                        time.sleep(0.3)

                    seq_id[0] += 1
                    my_id = seq_id[0]
                    steps = data.get('steps', [])
                    loop_count = data.get('loop_count', 1)
                    loop_forever = data.get('loop_forever', False)
                    mode = data.get('mode', 'CSP')

                    t = threading.Thread(
                        target=run_sequence_thread,
                        args=(steps, loop_count, loop_forever, mode, my_id),
                        daemon=True
                    )
                    t.start()
                    send_resp(True, f"Sequence started: {len(steps)} steps")

                elif cmd == ECMotorProcess.CMD_STOP_SEQUENCE:
                    seq_stop_event.set()
                    shared_stop.value = 1
                    shared_seq_running.value = 0
                    seq_running_flag[0] = False
                    if ec and ec.connected:
                        cur_mode = ec.mode
                        if cur_mode == 3:  # PV
                            for i in range(ec.slaves_count):
                                try:
                                    ec.velocity_stop(i)
                                except:
                                    pass
                            time.sleep(0.2)
                            for i in range(ec.slaves_count):
                                try:
                                    if ec.has_fault(i):
                                        ec.reset_fault(i)
                                        time.sleep(0.1)
                                    if not ec.is_enabled(i):
                                        ec.enable_drive(i)
                                        time.sleep(0.1)
                                except:
                                    pass
                        else:
                            try:
                                ec.stop_all()
                            except:
                                pass
                    proc_log(log_queue, "Sequence stopped + motors stopped")
                    send_resp(True, "Sequence stopped")

                else:
                    send_resp(False, f"Unknown command: {cmd}")

            except Exception as e:
                proc_log(log_queue, f"Command error ({cmd}): {e}")
                import traceback
                traceback.print_exc()
                send_resp(False, str(e))

        # ====== CLEANUP ======
        if ec and ec.connected:
            try:
                ec._pdo_running = False
                time.sleep(0.05)
                ec.master.close()
            except:
                pass

        try:
            winmm = ctypes.windll.winmm
            winmm.timeEndPeriod(1)
        except:
            pass

        proc_log(log_queue, "Motor process exiting")


def proc_log(log_queue, msg):
    """Send log message from motor process to main process"""
    try:
        log_queue.put_nowait(msg)
    except:
        pass
    print(f"[EC Motor] {msg}")
