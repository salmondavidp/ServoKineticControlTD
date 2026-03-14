"""
EtherCAT Engine for DNA Controller - Embedded in TouchDesigner
Stripped-down version of python_soem's ethercat_controller.py
Provides: connect, enable, disable, move, home, stop, status
"""
import pysoem
import struct
import time
import threading

class EtherCATEngine:
    MODE_PP = 1
    MODE_PV = 3
    MODE_CSP = 8

    CONTROL_SHUTDOWN = 0x0006
    CONTROL_SWITCH_ON = 0x0007
    CONTROL_ENABLE_OP = 0x000F
    CONTROL_DISABLE = 0x0000
    CONTROL_FAULT_RESET = 0x0080
    CONTROL_NEW_SETPOINT = 0x0010
    CONTROL_CHANGE_SET_IMMEDIATELY = 0x0020
    CONTROL_HALT = 0x0100

    STATUS_FAULT = 0x0008
    STATUS_TARGET_REACHED = 0x0400
    STATUS_OFFSET = 2
    POSITION_OFFSET = 4
    DEFAULT_CSP_VELOCITY = 100

    def __init__(self):
        self.master = None
        self.interface = None
        self.slaves_count = 0
        self.connected = False
        self.mode = self.MODE_CSP
        self._velocity = 80000
        self._accel = 6000
        self._decel = 6000
        self.ACTUAL_STEPS_PER_METER = 792914
        self.RAW_STEPS_PER_METER = 202985985
        self.SCALE_FACTOR = self.ACTUAL_STEPS_PER_METER / self.RAW_STEPS_PER_METER

        self._pdo_thread = None
        self._pdo_running = False
        self._pdo_lock = threading.Lock()
        self._cached_input = {}
        self._control_word = {}
        self._target_position = {}
        self._trajectory_active = {}
        self._trajectory_target = {}
        self._trajectory_locks = {}
        self._slave_csp_velocity = {}
        self._slave_velocity = {}
        self._slave_accel = {}
        self._slave_decel = {}

    def list_adapters(self):
        adapters = []
        for a in pysoem.find_adapters():
            name = a.name if isinstance(a.name, str) else a.name.decode('utf-8', errors='replace')
            desc = a.desc if isinstance(a.desc, str) else a.desc.decode('utf-8', errors='replace')
            adapters.append({'name': name, 'desc': desc})
        return adapters

    def configure(self, mode_str='CSP', velocity=80000, accel=6000, decel=6000,
                  steps_per_meter=792914, raw_steps=202985985, csp_velocity=100):
        mode_map = {'PP': self.MODE_PP, 'PV': self.MODE_PV, 'CSP': self.MODE_CSP}
        self.mode = mode_map.get(mode_str.upper(), self.MODE_CSP)
        self._velocity = velocity
        self._accel = accel
        self._decel = decel
        self.ACTUAL_STEPS_PER_METER = steps_per_meter
        self.RAW_STEPS_PER_METER = raw_steps
        self.SCALE_FACTOR = self.ACTUAL_STEPS_PER_METER / self.RAW_STEPS_PER_METER
        self._default_csp_velocity = csp_velocity

    def connect(self, interface_name=None):
        try:
            self.master = pysoem.Master()
            if interface_name:
                self.interface = interface_name
            else:
                adapters = pysoem.find_adapters()
                if not adapters:
                    return {'ok': False, 'error': 'No network adapters found'}
                self.interface = adapters[0].name

            self.master.open(self.interface)
            self.slaves_count = self.master.config_init(usetable=False)
            if self.slaves_count <= 0:
                self.master.close()
                return {'ok': False, 'error': 'No slaves found on interface'}

            for i in range(self.slaves_count):
                slave = self.master.slaves[i]
                self._control_word[i] = 0
                self._target_position[i] = 0
                self._cached_input[i] = bytes()
                self._trajectory_active[i] = False
                self._trajectory_target[i] = 0
                self._trajectory_locks[i] = threading.Lock()
                self._slave_csp_velocity[i] = getattr(self, '_default_csp_velocity', self.DEFAULT_CSP_VELOCITY)
                self._slave_velocity[i] = self._velocity
                self._slave_accel[i] = self._accel
                self._slave_decel[i] = self._decel

                slave.sdo_write(0x6060, 0x00, bytes([self.mode]))
                if self.mode == self.MODE_CSP:
                    try: slave.sdo_write(0x2025, 0x00, (50).to_bytes(2, 'little'))
                    except: pass
                    try: slave.sdo_write(0x2026, 0x00, (200).to_bytes(2, 'little'))
                    except: pass
                    try: slave.sdo_write(0x2020, 0x00, (100).to_bytes(2, 'little'))
                    except: pass
                else:
                    try: slave.sdo_write(0x2020, 0x00, (500).to_bytes(2, 'little'))
                    except: pass

                slave.sdo_write(0x6081, 0x00, self._velocity.to_bytes(4, 'little', signed=False))
                slave.sdo_write(0x607F, 0x00, self._velocity.to_bytes(4, 'little', signed=False))
                slave.sdo_write(0x6083, 0x00, self._accel.to_bytes(4, 'little', signed=False))
                slave.sdo_write(0x6084, 0x00, self._decel.to_bytes(4, 'little', signed=False))

            self.master.config_map()
            time.sleep(0.5)

            self.master.state = pysoem.SAFEOP_STATE
            self.master.write_state()
            for retry in range(3):
                try:
                    self.master.state_check(pysoem.SAFEOP_STATE, 50000)
                    break
                except:
                    if retry < 2:
                        time.sleep(1.0)
                        self.master.write_state()
                    else:
                        raise

            for _ in range(10):
                self.master.send_processdata()
                self.master.receive_processdata(2000)
                time.sleep(0.001)

            self.master.state = pysoem.OP_STATE
            self.master.write_state()
            for _ in range(50):
                self.master.send_processdata()
                self.master.receive_processdata(2000)
                time.sleep(0.001)
            self.master.state_check(pysoem.OP_STATE, 50000)

            self.connected = True
            self._start_pdo_thread()
            time.sleep(0.1)

            for i in range(self.slaves_count):
                self._sync_position(i)

            return {'ok': True, 'slaves': self.slaves_count}
        except Exception as e:
            return {'ok': False, 'error': str(e)}

    def disconnect(self):
        if not self.connected:
            return
        self._pdo_running = False
        if self._pdo_thread:
            self._pdo_thread.join(timeout=2.0)
        try:
            for i in range(self.slaves_count):
                with self._pdo_lock:
                    self._control_word[i] = self.CONTROL_DISABLE
            time.sleep(0.1)
            self.master.state = pysoem.INIT_STATE
            self.master.write_state()
            self.master.close()
        except:
            pass
        self.connected = False
        self.slaves_count = 0

    def _start_pdo_thread(self):
        self._pdo_running = True
        self._pdo_thread = threading.Thread(target=self._pdo_loop, daemon=True)
        self._pdo_thread.start()

    def _pdo_loop(self):
        use_perf = False
        kernel32 = None
        winmm = None
        perf_freq = 0
        try:
            import ctypes
            kernel32 = ctypes.windll.kernel32
            winmm = ctypes.windll.winmm
            winmm.timeBeginPeriod(1)
            current_thread = kernel32.GetCurrentThread()
            kernel32.SetThreadPriority(current_thread, 15)
            freq = ctypes.c_int64()
            kernel32.QueryPerformanceFrequency(ctypes.byref(freq))
            perf_freq = freq.value
            use_perf = True
        except:
            pass

        cycle_time = 0.001 if self.mode == self.MODE_CSP else 0.005

        try:
            while self._pdo_running:
                if use_perf:
                    import ctypes
                    start = ctypes.c_int64()
                    kernel32.QueryPerformanceCounter(ctypes.byref(start))
                    cycle_start = start.value
                else:
                    t0 = time.perf_counter()

                try:
                    with self._pdo_lock:
                        self.master.send_processdata()
                        self.master.receive_processdata(2000)

                        for i in range(self.slaves_count):
                            slave = self.master.slaves[i]
                            self._cached_input[i] = bytes(slave.input)

                            if self.mode == self.MODE_CSP:
                                with self._trajectory_locks[i]:
                                    if self._trajectory_active.get(i, False):
                                        err = self._trajectory_target.get(i, 0) - self._target_position.get(i, 0)
                                        csp_vel = self._slave_csp_velocity.get(i, self.DEFAULT_CSP_VELOCITY)
                                        if abs(err) > csp_vel:
                                            step = csp_vel if err > 0 else -csp_vel
                                            self._target_position[i] = self._target_position.get(i, 0) + step
                                        else:
                                            self._target_position[i] = self._trajectory_target.get(i, 0)

                            out_len = len(slave.output)
                            out_data = bytearray(out_len)
                            struct.pack_into('<H', out_data, 0, self._control_word.get(i, 0))
                            struct.pack_into('<i', out_data, 2, self._target_position.get(i, 0))
                            if out_len >= 7:
                                out_data[6] = self.mode
                            slave.output = bytes(out_data)
                except Exception as e:
                    pass  # PDO errors handled silently

                if use_perf:
                    import ctypes
                    target_count = cycle_start + int(perf_freq * cycle_time)
                    while True:
                        now = ctypes.c_int64()
                        kernel32.QueryPerformanceCounter(ctypes.byref(now))
                        if now.value >= target_count:
                            break
                        kernel32.Sleep(0)
                else:
                    elapsed = time.perf_counter() - t0
                    remaining = cycle_time - elapsed
                    if remaining > 0:
                        time.sleep(remaining)
        finally:
            try:
                if winmm: winmm.timeEndPeriod(1)
            except:
                pass

    def _get_pos_scaled(self, idx):
        inp = self._cached_input.get(idx, bytes())
        if len(inp) >= self.POSITION_OFFSET + 4:
            raw = struct.unpack_from('<i', inp, self.POSITION_OFFSET)[0]
            return int(raw * self.SCALE_FACTOR)
        return 0

    def _sync_position(self, idx):
        pos = self._get_pos_scaled(idx)
        with self._pdo_lock:
            self._target_position[idx] = pos
        with self._trajectory_locks[idx]:
            self._trajectory_target[idx] = pos
            self._trajectory_active[idx] = False
        return pos

    def read_status(self, idx):
        inp = self._cached_input.get(idx, bytes())
        if len(inp) >= self.STATUS_OFFSET + 2:
            return struct.unpack_from('<H', inp, self.STATUS_OFFSET)[0]
        return 0

    def read_position_meters(self, idx):
        return self._get_pos_scaled(idx) / self.ACTUAL_STEPS_PER_METER

    def is_enabled(self, idx):
        return (self.read_status(idx) & 0x006F) == 0x0027

    def has_fault(self, idx):
        return bool(self.read_status(idx) & self.STATUS_FAULT)

    def is_target_reached(self, idx):
        return bool(self.read_status(idx) & self.STATUS_TARGET_REACHED)

    def meters_to_scaled(self, meters):
        return int(meters * self.ACTUAL_STEPS_PER_METER)

    def enable_drive(self, idx):
        if self.has_fault(idx):
            self.reset_fault(idx)
            time.sleep(0.1)
        for _ in range(5):
            self._sync_position(idx)
            time.sleep(0.005)
        for retry in range(3):
            self._sync_position(idx)
            with self._pdo_lock: self._control_word[idx] = self.CONTROL_SHUTDOWN
            time.sleep(0.05)
            with self._pdo_lock: self._control_word[idx] = self.CONTROL_SWITCH_ON
            time.sleep(0.05)
            self._sync_position(idx)
            time.sleep(0.01)
            with self._pdo_lock: self._control_word[idx] = self.CONTROL_ENABLE_OP
            time.sleep(0.05)
            if self.is_enabled(idx):
                self._sync_position(idx)
                return True
            if self.has_fault(idx):
                self.reset_fault(idx)
                time.sleep(0.1)
        return False

    def enable_all(self):
        results = {}
        for i in range(self.slaves_count):
            results[i] = self.enable_drive(i)
        return results

    def disable_drive(self, idx):
        with self._trajectory_locks[idx]:
            self._trajectory_active[idx] = False
        with self._pdo_lock:
            self._control_word[idx] = self.CONTROL_DISABLE
        time.sleep(0.02)

    def disable_all(self):
        for i in range(self.slaves_count):
            self.disable_drive(i)

    def reset_fault(self, idx):
        with self._pdo_lock: self._control_word[idx] = self.CONTROL_FAULT_RESET
        time.sleep(0.05)
        with self._pdo_lock: self._control_word[idx] = 0
        time.sleep(0.05)
        self._sync_position(idx)

    def reset_all(self):
        for i in range(self.slaves_count):
            self.reset_fault(i)

    def move_to_meters(self, idx, meters):
        target = self.meters_to_scaled(meters)
        if self.mode == self.MODE_PP:
            with self._pdo_lock:
                self._target_position[idx] = target
                self._control_word[idx] = self.CONTROL_ENABLE_OP | self.CONTROL_NEW_SETPOINT | self.CONTROL_CHANGE_SET_IMMEDIATELY
            time.sleep(0.015)
            with self._pdo_lock:
                self._control_word[idx] = self.CONTROL_ENABLE_OP
        else:
            with self._trajectory_locks[idx]:
                self._trajectory_target[idx] = target
                self._trajectory_active[idx] = True

    def move_all_to_meters(self, positions):
        """positions: dict {slave_idx: meters}"""
        for idx, meters in positions.items():
            target = self.meters_to_scaled(meters)
            if self.mode == self.MODE_PP:
                with self._pdo_lock:
                    self._target_position[idx] = target
            else:
                with self._trajectory_locks[idx]:
                    self._trajectory_target[idx] = target
        if self.mode == self.MODE_PP:
            with self._pdo_lock:
                for idx in positions:
                    self._control_word[idx] = self.CONTROL_ENABLE_OP | self.CONTROL_NEW_SETPOINT | self.CONTROL_CHANGE_SET_IMMEDIATELY
            time.sleep(0.015)
            with self._pdo_lock:
                for idx in positions:
                    self._control_word[idx] = self.CONTROL_ENABLE_OP
        else:
            for idx in positions:
                with self._trajectory_locks[idx]:
                    self._trajectory_active[idx] = True

    def home_all(self):
        positions = {i: 0.0 for i in range(self.slaves_count)}
        self.move_all_to_meters(positions)

    def stop_all(self):
        for i in range(self.slaves_count):
            with self._trajectory_locks[i]:
                self._trajectory_active[i] = False
            pos = self._get_pos_scaled(i)
            with self._pdo_lock:
                self._target_position[i] = pos
        if self.mode == self.MODE_PP:
            for i in range(self.slaves_count):
                with self._pdo_lock:
                    self._control_word[i] = self.CONTROL_ENABLE_OP | self.CONTROL_HALT
            time.sleep(0.01)
            for i in range(self.slaves_count):
                pos = self._get_pos_scaled(i)
                with self._pdo_lock:
                    self._target_position[i] = pos
                    self._control_word[i] = self.CONTROL_ENABLE_OP | self.CONTROL_NEW_SETPOINT
            time.sleep(0.01)
            with self._pdo_lock:
                for i in range(self.slaves_count):
                    self._control_word[i] = self.CONTROL_ENABLE_OP

    def set_speed(self, idx, velocity, accel=None, decel=None):
        self._slave_velocity[idx] = velocity
        self._slave_accel[idx] = accel or velocity // 2
        self._slave_decel[idx] = decel or velocity // 2
        if self.connected and idx < self.slaves_count:
            slave = self.master.slaves[idx]
            try:
                slave.sdo_write(0x6081, 0x00, velocity.to_bytes(4, 'little', signed=False))
                slave.sdo_write(0x6083, 0x00, (accel or velocity//2).to_bytes(4, 'little', signed=False))
                slave.sdo_write(0x6084, 0x00, (decel or velocity//2).to_bytes(4, 'little', signed=False))
            except:
                pass

    def set_csp_velocity(self, idx, vel):
        self._slave_csp_velocity[idx] = vel

    def get_status_all(self):
        result = []
        for i in range(self.slaves_count):
            result.append({
                'idx': i,
                'enabled': self.is_enabled(i),
                'fault': self.has_fault(i),
                'target_reached': self.is_target_reached(i),
                'position_m': round(self.read_position_meters(i), 6)
            })
        return result

# Global singleton
_ec_engine = None

def get_engine():
    global _ec_engine
    if _ec_engine is None:
        _ec_engine = EtherCATEngine()
    return _ec_engine

def destroy_engine():
    global _ec_engine
    if _ec_engine is not None:
        _ec_engine.disconnect()
        _ec_engine = None
