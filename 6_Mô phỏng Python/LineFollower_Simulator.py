import sys
import math
import numpy as np
import json
import os
from dataclasses import dataclass
from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtCore import Qt

# =============================================================================
# 1. HELPER: MATH & ALGORITHMS
# =============================================================================

MAP_FILE = "my_maps.json"

def chaikin_smooth(points, iterations=3):
    """
    Thuật toán Chaikin: Biến các điểm gãy khúc thành đường cong mềm mại bằng cách cắt góc.
    """
    if len(points) < 3:
        return points
    
    output = list(points)
    for _ in range(iterations):
        new_points = [output[0]] 
        for i in range(len(output) - 1):
            p0 = output[i]
            p1 = output[i+1]
            
            Q = (0.75 * p0[0] + 0.25 * p1[0], 0.75 * p0[1] + 0.25 * p1[1])
            R = (0.25 * p0[0] + 0.75 * p1[0], 0.25 * p0[1] + 0.75 * p1[1])
            new_points.append(Q)
            new_points.append(R)
        
        new_points.append(output[-1])
        output = new_points
    return output

def generate_semicircle(p1, p2, direction=1, steps=20):
    """
    Tạo danh sách các điểm tạo thành nửa đường tròn nối giữa p1 và p2.
    """
    x1, y1 = p1
    x2, y2 = p2
    mx, my = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    radius = math.hypot(x2 - x1, y2 - y1) / 2.0
    base_angle = math.atan2(y2 - y1, x2 - x1)
    
    start_angle = base_angle + math.pi
    sweep = math.pi if direction > 0 else -math.pi
    
    angles = np.linspace(start_angle, start_angle + sweep, steps)
    
    arc_points = []
    for theta in angles:
        px = mx + radius * math.cos(theta)
        py = my + radius * math.sin(theta)
        arc_points.append((px, py))
        
    return arc_points

def dist_sq_point_to_segment(px, py, x1, y1, x2, y2):
    l2 = (x1 - x2)**2 + (y1 - y2)**2
    if l2 == 0: return (px - x1)**2 + (py - y1)**2
    t = ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / l2
    t = max(0, min(1, t))
    cx = x1 + t * (x2 - x1)
    cy = y1 + t * (y2 - y1)
    return (px - cx)**2 + (py - cy)**2

# =============================================================================
# DATA: MAP STRUCTURE
# =============================================================================
DEFAULT_MAPS = {
    "Map 1: Basic (Default)": [
        (0, 4), (8, 4), (9, 5), (10, 4), (14, 4),
        (14, 0.5), (11, 0.5), (10, 1.5), (8, 1.5),
        (8, 0), (5, 0),
        (5, 2.5), (2.5, 2.5), (2.5, 0), (0, 0)
    ],
    "Map 2: The Maze (Hard 90 Turns)": [
        (0, 0), (5, 0), (5, 5), (10, 5), (10, 0), 
        (15, 0), (15, 10), (10, 10), (10, 15), 
        (5, 15), (5, 10), (0, 10), (0, 10), (0, 5), (-5, 5), (-5, 0), (0, 0)
    ],
    "Map 3: T-Junction Test": [
        (0,0), (10,0), (10, -5), (10, 5), (10, 0), (15, 0) 
    ]
}

class MapManager:
    def __init__(self):
        self.maps = DEFAULT_MAPS.copy()
        self.load_from_file()

    def load_from_file(self):
        if os.path.exists(MAP_FILE):
            try:
                with open(MAP_FILE, 'r') as f:
                    custom_maps = json.load(f)
                    self.maps.update(custom_maps)
            except Exception as e:
                print(f"Error loading maps: {e}")

    def save_to_file(self):
        custom_maps = {k: v for k, v in self.maps.items() if k not in DEFAULT_MAPS}
        try:
            with open(MAP_FILE, 'w') as f:
                json.dump(custom_maps, f)
        except Exception as e:
            print(f"Error saving maps: {e}")

    def add_map(self, name, points):
        self.maps[name] = points
        self.save_to_file()

    def delete_map(self, name):
        if name in DEFAULT_MAPS: return False
        if name in self.maps:
            del self.maps[name]
            self.save_to_file()
            return True
        return False

map_manager = MapManager()

# =============================================================================
# 2. LOGIC SIMULATION
# =============================================================================

@dataclass
class MotorCommand:
    left_pwm: int
    right_pwm: int

class MotorDriver:
    def __init__(self, pwm_max=255, vel_at_255=120.0):
        self.pwm_max = int(pwm_max)
        self.vel_at_255 = float(vel_at_255)
    def clamp(self, pwm: int) -> int:
        return max(-self.pwm_max, min(self.pwm_max, int(pwm)))
    def set_pwm(self, left_pwm: int, right_pwm: int) -> MotorCommand:
        return MotorCommand(self.clamp(left_pwm), self.clamp(right_pwm))
    def pwm_to_wheel_velocity(self, pwm: int) -> float:
        return (pwm / float(self.pwm_max)) * self.vel_at_255

class PIDController:
    def __init__(self, kp=2.8, ki=0.001, kd=10.0): 
        self.kp = float(kp); self.ki = float(ki); self.kd = float(kd)
        self.e_prev = 0.0; self.e_sum = 0.0
    def reset(self):
        self.e_prev = 0.0; self.e_sum = 0.0
    def update_gains(self, kp, ki, kd):
        self.kp = float(kp); self.ki = float(ki); self.kd = float(kd)
    def compute(self, error: float) -> float:
        de = error - self.e_prev
        self.e_sum += error
        self.e_sum = max(-3000, min(3000, self.e_sum)) 
        u = self.kp * error + self.kd * de + self.ki * self.e_sum
        self.e_prev = error
        return u

@dataclass
class SensorReadout:
    adc: list; bit: list; bit_string: str; sum_on: int; weight_sum: float; position: float | None

class LineSensorArray:
    def __init__(self, sensor_number=8, threshold=120):
        self.n = int(sensor_number)
        self.threshold = int(threshold)
        self.weights = [-40, -30, -20, -10, 10, 20, 30, 40]
        self.cen_pos = 0.0 
        self.sensor_offset_x = 2.0 
        self.sensor_offsets_y = np.linspace(-20, 20, self.n)
    
    def update_threshold(self, threshold):
        self.threshold = int(threshold)
    
    def configure_layout_tight_to_line(self, line_half_width: float):
        line_width = max(1.0, line_half_width * 2.0)
        spacing = line_width * 0.8
        offsets = (np.arange(self.n) - (self.n - 1) / 2.0) * spacing
        self.sensor_offsets_y = offsets
        self.sensor_offset_x = 2.0 
    
    def read(self, car, track) -> SensorReadout:
        sensor_adc = []
        bit_sensor = []
        bit_str = ""
        sum_on = 0
        weight_sum = 0.0
        for i in range(self.n):
            sx = (car.x + math.cos(car.theta) * self.sensor_offset_x - math.sin(car.theta) * self.sensor_offsets_y[i])
            sy = (car.y + math.sin(car.theta) * self.sensor_offset_x + math.cos(car.theta) * self.sensor_offsets_y[i])
            _, dist = track.closest_point(sx, sy)
            on_line = 1 if dist < track.line_half else 0
            adc = 0 if on_line else 1023
            bit = 1 if adc < self.threshold else 0
            sensor_adc.append(adc)
            bit_sensor.append(bit)
            bit_str += str(bit)
            sum_on += bit
            weight_sum += bit * self.weights[i]
        position = (weight_sum / sum_on) if sum_on > 0 else None
        return SensorReadout(sensor_adc, bit_sensor, bit_str, sum_on, weight_sum, position)

class TurnLogic:
    def __init__(self, turn_speed=160, search_bias=120):
        self.turn_speed = int(turn_speed)
        self.search_bias = int(search_bias)
        self.turn_cooldown = 0 
    
    def check_90_degree(self, bit_sensor: list) -> MotorCommand | None:
        """
        Kiểm tra góc vuông dựa trên cảm biến.
        Nếu thấy vạch ngang bên trái/phải rõ ràng -> Trả về lệnh quay vuông góc.
        """
        if self.turn_cooldown > 0:
            self.turn_cooldown -= 1
            return None
        n = len(bit_sensor)
        mid = n // 2
        left_part = bit_sensor[:mid]; right_part = bit_sensor[mid:]  
        sum_left = sum(left_part); sum_right = sum(right_part)
        is_left_turn = (sum_left >= 3 and sum_right == 0)
        is_right_turn = (sum_right >= 3 and sum_left == 0)
        if is_left_turn:
            self.turn_cooldown = 20
            return MotorCommand(-100, 180) 
        if is_right_turn:
            self.turn_cooldown = 20
            return MotorCommand(180, -100)
        return None
    
    def recovery_when_lost_line(self, prev_error: float, base_speed: int) -> MotorCommand:
        """Logic tìm lại line khi mất dấu hoàn toàn (quay tại chỗ theo hướng lỗi cuối cùng)"""
        spin_speed = 130 
        if prev_error > 5: return MotorCommand(spin_speed, -spin_speed)
        if prev_error < -5: return MotorCommand(-spin_speed, spin_speed)
        return MotorCommand(60, 60)

class Car:
    def __init__(self, x, y, theta=0.0):
        self.x = x; self.y = y; self.theta = theta
        self.wheel_base = 40.0
    def update_diff_drive(self, vl, vr, dt):
        v = (vl + vr) / 2.0
        omega = (vr - vl) / self.wheel_base
        if abs(omega) < 1e-6:
            self.x += v * math.cos(self.theta) * dt
            self.y += v * math.sin(self.theta) * dt
        else:
            R = v / omega
            cx = self.x - R * math.sin(self.theta)
            cy = self.y + R * math.cos(self.theta)
            self.theta += omega * dt
            self.x = cx + R * math.sin(self.theta)
            self.y = cy - R * math.cos(self.theta)

class PathTrack:
    def __init__(self):
        self.points = [] 
        self.line_half = 2.0
    def make_from_point_list(self, pts):
        valid_pts = [p for p in pts if p is not None]
        if not valid_pts: return
        xs = [p[0] for p in valid_pts]; ys = [p[1] for p in valid_pts]
        minx, maxx = min(xs), max(xs); miny, maxy = min(ys), max(ys)
        iw = maxx - minx if maxx > minx else 1.0
        ih = maxy - miny if maxy > miny else 1.0
        center_x, center_y = 500.0, 300.0
        w, h = 600.0, 300.0
        scale = min(w / iw, h / ih) * 0.9
        tx = center_x - (minx + iw / 2.0) * scale
        ty = center_y - (miny + ih / 2.0) * scale
        self.points = []
        for p in pts:
            if p is None: self.points.append(None)
            else: self.points.append((float(p[0]) * scale + tx, float(p[1]) * scale + ty))
    def closest_point(self, px, py):
        best = None; best_d2 = float("inf")
        if not self.points: return (0,0), float("inf")
        for i in range(len(self.points) - 1):
            p1 = self.points[i]; p2 = self.points[i + 1]
            if p1 is None or p2 is None: continue
            x1, y1 = p1; x2, y2 = p2
            dx = x2 - x1; dy = y2 - y1
            if dx == 0 and dy == 0: t = 0.0
            else:
                t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
                t = max(0.0, min(1.0, t))
            cx = x1 + t * dx; cy = y1 + t * dy
            d2 = (px - cx) ** 2 + (py - cy) ** 2
            if d2 < best_d2: best_d2 = d2; best = (cx, cy)
        if best is None: return (0,0), float("inf")
        return best, math.sqrt(best_d2)
    def bounds(self):
        valid_pts = [p for p in self.points if p is not None]
        if not valid_pts: return 0,0,1,1
        xs = [p[0] for p in valid_pts]; ys = [p[1] for p in valid_pts]
        return min(xs), min(ys), max(xs), max(ys)

class LineFollowerRobot:
    def __init__(self, track: PathTrack):
        self.track = track
        self.sensors = LineSensorArray(sensor_number=8, threshold=120)
        self.pid = PIDController(kp=2.8, ki=0.001, kd=10.0)
        self.turn = TurnLogic(turn_speed=160, search_bias=180)
        self.motor = MotorDriver(pwm_max=255, vel_at_255=120.0)
        self.max_pwm = 230; self.min_pwm = 90; self.k_reduce = 2.0
        self.car = self._create_car_at_track_start()
        self.last_error = 0.0
        self.sensors.configure_layout_tight_to_line(self.track.line_half)
        
        # --- BIẾN ĐIỀU KHIỂN LOGIC ĐẶC BIỆT ---
        self.is_crossing_tjunc = False
        self.TJUNC_FORWARD_MS_DURATION = 0.05
        self.tjunc_timer = 0.0
        self.is_force_turning_left = False
        self.turn_timer = 0.0
        self.tjunc_cooldown = 0.0
        self.debug_turn_counter = 0

        # Timer để đếm thời gian Full Line (dùng cho việc dừng xe hoặc qua ngã tư)
        self.full_line_timer = 0.0
        self.is_checking_line = False

    def _create_car_at_track_start(self):
        valid_indices = [i for i, p in enumerate(self.track.points) if p is not None]
        if len(valid_indices) < 2: return Car(0,0,0)
        idx = valid_indices[0]
        if idx + 1 < len(self.track.points) and self.track.points[idx+1] is not None:
             sx, sy = self.track.points[idx]; nx, ny = self.track.points[idx+1]
        else:
             sx, sy = self.track.points[idx]; nx, ny = sx + 1, sy
        theta = math.atan2(ny - sy, nx - sx)
        car_x = sx - math.cos(theta) * self.sensors.sensor_offset_x
        car_y = sy - math.sin(theta) * self.sensors.sensor_offset_x
        car = Car(car_x, car_y, theta)
        car.wheel_base = max(12.0, self.track.line_half * 3.0)
        return car
    
    def reset(self):
        self.car = self._create_car_at_track_start()
        self.pid.reset()
        self.last_error = 0.0
        self.is_crossing_tjunc = False
        self.is_force_turning_left = False
        self.tjunc_timer = 0.0
        self.tjunc_cooldown = 0.0
        self.debug_turn_counter = 0 
        self.turn_timer = 0.0
        self.full_line_timer = 0.0
        self.is_checking_line = False

    def update_params(self, kp, ki, kd, k_reduce, threshold, max_pwm, min_pwm):
        self.pid.update_gains(kp, ki, kd)
        self.k_reduce = float(k_reduce)
        self.sensors.update_threshold(threshold)
        self.max_pwm = int(max_pwm); self.min_pwm = int(min_pwm)

    def control_step(self, dt=0.03) -> tuple[SensorReadout, MotorCommand, float]:
        """
        Hàm điều khiển chính.
        Ưu tiên: Full Line -> T-Junction (đi thẳng mù) -> PID/Turn 90.
        """
        readout = self.sensors.read(self.car, self.track)
        
        if self.tjunc_cooldown > 0:
            self.tjunc_cooldown -= dt

       # =========================================================================
        # 1. LOGIC XỬ LÝ FULL LINE (NGÃ 3, NGÃ 4, ĐÍCH)
        # =========================================================================
        
        # Phát hiện bắt đầu vào vạch ngang (8 mắt đen)
        if readout.sum_on >= self.sensors.n and not self.is_checking_line:
            self.is_checking_line = True
            self.full_line_timer = 0.0

        # Đang trong quá trình đi qua vạch
        if self.is_checking_line:
            self.full_line_timer += dt
            
            # Giữ tốc độ cao để lao qua ngã tư, tránh bị khựng
            pass_speed = 140 
            cmd = self.motor.set_pwm(pass_speed, pass_speed)
            
            # Sau 0.3s kiểm tra lại tình trạng
            if self.full_line_timer > 0.3:
                # TRƯỜNG HỢP 1: Hết đường (Ngã 3 cụt) -> Buộc Rẽ Trái
                if readout.sum_on == 0: 
                    self.is_force_turning_left = True
                    self.turn_timer = 0.0
                    self.debug_turn_counter = 0
                    self.is_checking_line = False      
                    self.tjunc_cooldown = 1.0          
                    return readout, cmd, 0.0

                # TRƯỜNG HỢP 2: Vẫn đen sì (Vạch đích) -> Dừng
                elif readout.sum_on >= 7:
                    cmd = self.motor.set_pwm(0, 0)
                    return readout, cmd, 0.0
                
                # TRƯỜNG HỢP 3: Đã qua ngã tư và thấy line bình thường -> Trả về PID
                else:
                    self.is_checking_line = False 
            
            # Nếu chưa hết timer mà đã thấy line trở lại (đã qua ngã tư) -> Thoát ngay
            if readout.sum_on < 7 and readout.sum_on > 0:
                 self.is_checking_line = False

            return readout, cmd, 0.0

        # =========================================================================
        # 2. LOGIC ĐANG ĐI THẲNG 50ms KHI GẶP T-JUNCTION
        # =========================================================================
        if self.is_crossing_tjunc:
            self.tjunc_timer -= dt
            
            forward_speed = 120 
            cmd = self.motor.set_pwm(forward_speed, forward_speed)
            
            # Hết thời gian đi thẳng mù -> Chuyển sang rẽ trái (quy tắc mê cung)
            if self.tjunc_timer <= 0:
                self.is_crossing_tjunc = False
                self.is_force_turning_left = True 
                self.turn_timer = 0.0 
                self.debug_turn_counter = 0 
                self.tjunc_cooldown = 0.5 
            
            return readout, cmd, 0.0

        # =========================================================================
        # 3. PHÁT HIỆN T-JUNCTION (NHƯNG KHÔNG PHẢI FULL LINE)
        # =========================================================================
        if readout.sum_on >= (self.sensors.n - 1) and self.tjunc_cooldown <= 0:
            self.is_crossing_tjunc = True
            self.tjunc_timer = self.TJUNC_FORWARD_MS_DURATION 
            cmd = self.motor.set_pwm(120, 120)
            return readout, cmd, 0.0

        # =========================================================================
        # 4. TURN LOGIC (VUÔNG GÓC THÔNG THƯỜNG)
        # =========================================================================
        special_turn = self.turn.check_90_degree(readout.bit)
        if special_turn is not None and not self.is_crossing_tjunc:
            cmd = self.motor.set_pwm(special_turn.left_pwm, special_turn.right_pwm)
            self.pid.reset()
            return readout, cmd, self.last_error
        
        # =========================================================================
        # 5. PID LINE FOLLOWING (CHẾ ĐỘ BÌNH THƯỜNG)
        # =========================================================================
        if readout.sum_on > 0 and readout.position is not None:
            error = self.sensors.cen_pos - readout.position
            
            # Xử lý khi lỗi quá lớn (suýt văng)
            if error > 30: 
                cmd = self.motor.set_pwm(180, -80); self.last_error = error; return readout, cmd, error
            if error < -30: 
                cmd = self.motor.set_pwm(-80, 180); self.last_error = error; return readout, cmd, error
            
            u = self.pid.compute(error)
            dyn_speed = self.max_pwm - self.k_reduce * abs(error)
            dyn_speed = max(self.min_pwm, min(self.max_pwm, dyn_speed))
            
            left = int(dyn_speed + u)
            right = int(dyn_speed - u)
            cmd = self.motor.set_pwm(left, right)
            self.last_error = error
            return readout, cmd, error
            
        # =========================================================================
        # 6. LOST LINE RECOVERY (MẤT LINE)
        # =========================================================================
        rec = self.turn.recovery_when_lost_line(self.last_error, 100)
        cmd = self.motor.set_pwm(rec.left_pwm, rec.right_pwm)
        return readout, cmd, self.last_error

    def physics_step(self, cmd: MotorCommand, dt: float):
        vl = self.motor.pwm_to_wheel_velocity(cmd.left_pwm)
        vr = self.motor.pwm_to_wheel_velocity(cmd.right_pwm)
        self.car.update_diff_drive(vl, vr, dt)

# =============================================================================
# 3. GUI (UI)
# =============================================================================

LIGHT_THEME = """
QMainWindow { background-color: #f5f5f5; color: #000000; }
QLabel { color: #333333; font-family: 'Segoe UI', sans-serif; font-size: 13px; font-weight: 500; }
QGroupBox { border: 1px solid #aaaaaa; border-radius: 6px; margin-top: 20px; background-color: #ffffff; color: #006064; font-weight: bold; }
QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top center; padding: 0 5px; background-color: #ffffff; }
QPushButton { background-color: #e0e0e0; color: #000000; border: 1px solid #bdbdbd; padding: 8px; border-radius: 4px; font-weight: bold; }
QPushButton:hover { background-color: #d5d5d5; border: 1px solid #9e9e9e; }
QPushButton:checked { background-color: #ff9800; color: black; border: 1px solid #ef6c00; }
QPushButton:disabled { background-color: #f0f0f0; color: #9e9e9e; border: 1px solid #e0e0e0; }
QComboBox { background-color: #ffffff; color: #000000; border: 1px solid #bdbdbd; padding: 5px; border-radius: 4px; selection-background-color: #b2ebf2; selection-color: #000000; }
QTextEdit { background-color: #ffffff; color: #212121; border: 1px solid #bdbdbd; font-family: 'Consolas', monospace; font-size: 12px; }
QCheckBox { color: #000000; spacing: 5px; } 
QDoubleSpinBox, QSpinBox { background-color: #ffffff; color: #000000; border: 1px solid #bdbdbd; padding: 2px; }
"""

class CanvasWidget(QtWidgets.QLabel):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMouseTracking(True)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.setMinimumSize(400, 300)

class Simulator(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Line Follower Simulator")
        self.resize(1280, 850)
        self.setStyleSheet(LIGHT_THEME)

        self.track = PathTrack()
        first_map_name = list(map_manager.maps.keys())[0]
        self.track.make_from_point_list(map_manager.maps[first_map_name])
        
        self.robot = LineFollowerRobot(self.track)
        self.sensor_highlight = [0.0] * self.robot.sensors.n

        # Transform settings
        self.sensor_base_radius = 1.0
        self.fit_view = True
        self.view_scale = 1.0
        self.view_offset = (0, 0)
        
        # Mouse Interaction
        self._panning = False
        self._last_mouse_pos = None
        self._dragging_car = False
        self._drag_offset = (0.0, 0.0)
        self._is_hovering_car = False

        # Drawing Mode State
        self.is_drawing_mode = False
        self.is_arc_mode = False 
        self.arc_direction = 1 
        self.drawing_paths = [[]] 
        self.mouse_world_pos = (0, 0) 
        
        # Metadata để quản lý Arc & Style
        self.path_metadata = [[]] 
        self.path_styles = [False] 
        self.arc_counter = 0 
        
        # Edit Existing Map Variables
        self.is_editing_existing = False
        self.editing_map_name = ""

        # Eraser Logic
        self.is_eraser_mode = False
        self.hovered_path_index = -1 
        self.hovered_segment_index = -1 

        self.running = False
        self.init_ui()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.step)
        self.timer.start(30) # 30ms per frame

    def init_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        main_layout = QtWidgets.QHBoxLayout(central)
        main_layout.setContentsMargins(10, 10, 10, 10)

        # Left Canvas
        canvas_container = QtWidgets.QWidget()
        canvas_layout = QtWidgets.QVBoxLayout(canvas_container)
        canvas_layout.setContentsMargins(0, 0, 0, 0)
        self.canvas_label = CanvasWidget()
        self.canvas_label.installEventFilter(self)
        self.canvas_label.setStyleSheet("background-color: #ffffff; border: 1px solid #ccc; border-radius: 6px;")
        canvas_layout.addWidget(self.canvas_label)
        main_layout.addWidget(canvas_container, stretch=3)

        # Right Controls
        controls_widget = QtWidgets.QWidget()
        controls_widget.setFixedWidth(340)
        controls_layout = QtWidgets.QVBoxLayout(controls_widget)
        controls_layout.setContentsMargins(0, 0, 0, 0)

        # 1. Simulation Controls
        grp_status = QtWidgets.QGroupBox("Simulation Control")
        l_status = QtWidgets.QVBoxLayout(grp_status)
        self.btn_startstop = QtWidgets.QPushButton("START Simulation")
        self.btn_startstop.setCheckable(True)
        self.btn_startstop.setStyleSheet("""
            QPushButton { background-color: #4caf50; color: white; font-size: 14px; height: 30px; font-weight: bold; border: 1px solid #388e3c; } 
            QPushButton:checked { background-color: #f44336; border: 1px solid #d32f2f; }
            QPushButton:hover { background-color: #66bb6a; }
        """)
        self.btn_startstop.clicked.connect(lambda: self.toggle_start(self.btn_startstop))
        btn_reset = QtWidgets.QPushButton("Reset Position")
        btn_reset.setStyleSheet("""
            QPushButton { background-color: #039be5; color: white; border: 1px solid #0277bd; }
            QPushButton:hover { background-color: #29b6f6; }
        """)
        btn_reset.clicked.connect(self.reset_robot)
        l_status.addWidget(self.btn_startstop)
        l_status.addWidget(btn_reset)
        controls_layout.addWidget(grp_status)

        # 2. Map Creator & Editor
        grp_draw = QtWidgets.QGroupBox("Map Creator & Editor")
        l_draw = QtWidgets.QVBoxLayout(grp_draw)
        
        row_create_edit = QtWidgets.QHBoxLayout()
        self.btn_draw_mode = QtWidgets.QPushButton("✎ CREATE NEW")
        self.btn_draw_mode.setCheckable(True)
        self.btn_draw_mode.clicked.connect(self.toggle_draw_mode)
        
        self.btn_edit_map = QtWidgets.QPushButton("✎ EDIT Current")
        self.btn_edit_map.setStyleSheet("""
            QPushButton { background-color: #ffb74d; color: black; border: 1px solid #ffa726; font-weight: bold;}
            QPushButton:hover { background-color: #ffcc80; }
        """)
        self.btn_edit_map.clicked.connect(self.edit_current_map)
        self.btn_edit_map.setToolTip("Sửa bản đồ đang chọn (Không sửa được bản đồ mặc định)")
        
        row_create_edit.addWidget(self.btn_draw_mode)
        row_create_edit.addWidget(self.btn_edit_map)
        l_draw.addLayout(row_create_edit)
        
        # Tools
        row_tools = QtWidgets.QHBoxLayout()
        self.btn_break_line = QtWidgets.QPushButton("✂ Break Line")
        self.btn_break_line.setStyleSheet("""
            QPushButton { background-color: #ab47bc; color: white; border: 1px solid #8e24aa; }
            QPushButton:hover { background-color: #ba68c8; }
        """)
        self.btn_break_line.clicked.connect(self.break_current_line)
        self.btn_break_line.setToolTip("Ngắt nét hiện tại để vẽ nét mới")
        
        self.btn_eraser = QtWidgets.QPushButton("Eraser (Xoá)")
        self.btn_eraser.setCheckable(True)
        self.btn_eraser.setStyleSheet("""
            QPushButton { background-color: #e0e0e0; border: 1px solid #999; color: black; }
            QPushButton:checked { background-color: #ff5252; color: white; border: 1px solid #d32f2f; }
            QPushButton:hover { background-color: #bdbdbd; }
        """)
        self.btn_eraser.clicked.connect(self.toggle_eraser)

        row_tools.addWidget(self.btn_break_line)
        row_tools.addWidget(self.btn_eraser)
        l_draw.addLayout(row_tools)

        # ARC TOOLS
        row_arc = QtWidgets.QHBoxLayout()
        self.btn_arc_mode = QtWidgets.QPushButton("Mode: Line")
        self.btn_arc_mode.setCheckable(True)
        self.btn_arc_mode.clicked.connect(self.toggle_arc_mode)
        self.btn_arc_mode.setStyleSheet("""
            QPushButton { background-color: #009688; color: white; border: 1px solid #00796b; }
            QPushButton:hover { background-color: #26a69a; }
        """)

        self.btn_flip_arc = QtWidgets.QPushButton("Flip Arc (Đảo chiều)")
        self.btn_flip_arc.clicked.connect(self.flip_arc_direction)
        self.btn_flip_arc.setStyleSheet("""
            QPushButton { background-color: #00bcd4; color: white; border: 1px solid #0097a7; }
            QPushButton:hover { background-color: #26c6da; }
        """)
        
        row_arc.addWidget(self.btn_arc_mode)
        row_arc.addWidget(self.btn_flip_arc)
        l_draw.addLayout(row_arc)
        
        self.btn_break_line.setVisible(False)
        self.btn_eraser.setVisible(False)
        self.btn_arc_mode.setVisible(False)
        self.btn_flip_arc.setVisible(False)
        
        self.chk_smooth = QtWidgets.QCheckBox("Smooth Path (Cong mềm)")
        self.chk_smooth.setChecked(False) 
        self.chk_smooth.clicked.connect(self.update_current_path_style)
        l_draw.addWidget(self.chk_smooth)

        # Action Buttons
        h_btns = QtWidgets.QHBoxLayout()
        self.btn_clear_draw = QtWidgets.QPushButton("Clear All")
        self.btn_clear_draw.clicked.connect(self.clear_drawing)
        self.btn_save_map = QtWidgets.QPushButton("💾 Save Map")
        self.btn_save_map.setStyleSheet("""
            QPushButton { background-color: #fbc02d; color: black; border: 1px solid #f9a825; }
            QPushButton:hover { background-color: #fdd835; }
        """)
        self.btn_save_map.clicked.connect(self.save_custom_map)
        
        self.btn_clear_draw.setVisible(False)
        self.btn_save_map.setVisible(False)
        self.chk_smooth.setVisible(False)
        
        h_btns.addWidget(self.btn_clear_draw)
        h_btns.addWidget(self.btn_save_map)
        l_draw.addLayout(h_btns)
        controls_layout.addWidget(grp_draw)

        # 3. Quick Orientation
        grp_orient = QtWidgets.QGroupBox("Quick Orientation")
        l_orient = QtWidgets.QGridLayout(grp_orient)
        l_orient.setSpacing(5)
        btn_0 = QtWidgets.QPushButton("0° (Right)"); btn_0.clicked.connect(lambda: self.set_car_angle(0))
        btn_up = QtWidgets.QPushButton("-90° (Up)"); btn_up.clicked.connect(lambda: self.set_car_angle(-math.pi/2))
        btn_down = QtWidgets.QPushButton("90° (Down)"); btn_down.clicked.connect(lambda: self.set_car_angle(math.pi/2))
        btn_uturn = QtWidgets.QPushButton("↺ 180°"); btn_uturn.clicked.connect(self.do_u_turn)
        l_orient.addWidget(btn_up, 0, 0, 1, 2)
        l_orient.addWidget(btn_0, 1, 0)
        l_orient.addWidget(btn_uturn, 1, 1)
        l_orient.addWidget(btn_down, 2, 0, 1, 2)
        controls_layout.addWidget(grp_orient)

        # 4. Track & Visuals
        grp_track = QtWidgets.QGroupBox("Map Selection & Visuals")
        l_track = QtWidgets.QVBoxLayout(grp_track)
        
        row_map = QtWidgets.QHBoxLayout()
        self.cb_map = QtWidgets.QComboBox()
        self.btn_del_map = QtWidgets.QPushButton("❌")
        self.btn_del_map.setFixedWidth(30)
        self.btn_del_map.setStyleSheet("""
            QPushButton { background-color: #e53935; color: white; border: 1px solid #c62828; }
            QPushButton:hover { background-color: #ef5350; }
        """)
        self.btn_del_map.clicked.connect(self.delete_current_map)
        
        row_map.addWidget(self.cb_map)
        row_map.addWidget(self.btn_del_map)
        l_track.addLayout(row_map)
        
        self.update_map_list() 
        self.cb_map.currentIndexChanged.connect(self.change_map)
        
        # Line width and Sensor radius
        default_width = int(self.track.line_half)
        self.lw_lbl = QtWidgets.QLabel(f"Line Half-Width: {default_width} cm")
        self.lw_s = QtWidgets.QSlider(Qt.Horizontal)
        self.lw_s.setRange(1, 20)
        self.lw_s.setValue(default_width)
        self.lw_s.valueChanged.connect(self.update_track_width)
        l_track.addWidget(self.lw_lbl)
        l_track.addWidget(self.lw_s)

        self.sr_lbl = QtWidgets.QLabel(f"Sensor Radius: {self.sensor_base_radius:.1f}")
        self.sr_s = QtWidgets.QSlider(Qt.Horizontal)
        self.sr_s.setRange(5, 50) 
        self.sr_s.setValue(int(self.sensor_base_radius * 10))
        self.sr_s.valueChanged.connect(self.update_sensor_radius)
        l_track.addWidget(self.sr_lbl)
        l_track.addWidget(self.sr_s)

        controls_layout.addWidget(grp_track)

        # 5. Tuning (PID)
        grp_pid = QtWidgets.QGroupBox("Tuning (PID & Speed)")
        l_pid = QtWidgets.QVBoxLayout(grp_pid)

        def _add_float_control(name, min_val, max_val, default_val, scale, decimal=2):
            row = QtWidgets.QHBoxLayout()
            lbl = QtWidgets.QLabel(name)
            spin = QtWidgets.QDoubleSpinBox()
            spin.setRange(min_val / scale, max_val / scale)
            spin.setValue(default_val / scale)
            spin.setSingleStep(1.0 / scale if scale < 1000 else 0.001)
            spin.setDecimals(decimal)
            
            row.addWidget(lbl)
            row.addWidget(spin)
            l_pid.addLayout(row)

            slider = QtWidgets.QSlider(Qt.Horizontal)
            slider.setRange(min_val, max_val)
            slider.setValue(default_val)
            l_pid.addWidget(slider)

            slider.valueChanged.connect(lambda v: spin.setValue(v / scale))
            spin.valueChanged.connect(lambda v: slider.setValue(int(v * scale)))

            return spin, slider

        def _add_int_control(name, min_val, max_val, default_val):
            row = QtWidgets.QHBoxLayout()
            lbl = QtWidgets.QLabel(name)
            spin = QtWidgets.QSpinBox()
            spin.setRange(min_val, max_val)
            spin.setValue(default_val)
            
            row.addWidget(lbl)
            row.addWidget(spin)
            l_pid.addLayout(row)

            slider = QtWidgets.QSlider(Qt.Horizontal)
            slider.setRange(min_val, max_val)
            slider.setValue(default_val)
            l_pid.addWidget(slider)

            slider.valueChanged.connect(spin.setValue)
            spin.valueChanged.connect(slider.setValue)

            return spin, slider

        self.kp_spin, self.kp_s = _add_float_control("Kp:", 0, 2000, 1400, 100.0)
        self.ki_spin, self.ki_s = _add_float_control("Ki:", 0, 100, 0, 1000.0, decimal=3)
        self.kd_spin, self.kd_s = _add_float_control("Kd:", 0, 3000, 500, 100.0)
        self.max_spin, self.max_s = _add_int_control("Max PWM:", 0, 255, self.robot.max_pwm)

        btn_rst_pid = QtWidgets.QPushButton("⟲ Reset PID Defaults")
        btn_rst_pid.setStyleSheet("""
            QPushButton { background-color: #795548; color: white; border: 1px solid #5d4037; }
            QPushButton:hover { background-color: #8d6e63; }
        """)
        btn_rst_pid.clicked.connect(self.reset_pid_defaults) 
        l_pid.addWidget(btn_rst_pid)

        controls_layout.addWidget(grp_pid)

        controls_layout.addStretch()
        self.log = QtWidgets.QTextEdit()
        self.log.setMaximumHeight(100)
        self.log.setReadOnly(True)
        controls_layout.addWidget(self.log)
        main_layout.addWidget(controls_widget)

    def reset_pid_defaults(self):
        self.kp_s.setValue(1400) # Kp = 14.0
        self.ki_s.setValue(0)    # Ki = 0.0
        self.kd_s.setValue(500)  # Kd = 5.0
        self.max_s.setValue(230) # Max PWM
        self.log.append(">> PID parameters reset to defaults.")

    # --- MAP MANAGER ---
    def update_map_list(self):
        curr = self.cb_map.currentText()
        self.cb_map.blockSignals(True)
        self.cb_map.clear()
        self.cb_map.addItems(list(map_manager.maps.keys()))
        if curr in map_manager.maps: 
            self.cb_map.setCurrentText(curr)
        self.cb_map.blockSignals(False)
        self.btn_del_map.setEnabled(curr not in DEFAULT_MAPS)
        self.btn_edit_map.setEnabled(curr not in DEFAULT_MAPS) 

    def change_map(self):
        map_name = self.cb_map.currentText()
        if map_name in map_manager.maps:
            if self.running: self.toggle_start(self.btn_startstop)
            points = map_manager.maps[map_name]
            self.track.make_from_point_list(points)
            self.track.line_half = float(self.lw_s.value())
            self.fit_view = True
            self.reset_robot()
            self.step()
            self.log.append(f">> Map changed to: {map_name}")
            self.btn_del_map.setEnabled(map_name not in DEFAULT_MAPS)
            self.btn_edit_map.setEnabled(map_name not in DEFAULT_MAPS)

    def delete_current_map(self):
        name = self.cb_map.currentText()
        if map_manager.delete_map(name):
            self.log.append(f">> Deleted: {name}")
            self.update_map_list()
            self.change_map()
        else:
            self.log.append("!! Cannot delete Default Map.")

    # --- DRAWING LOGIC ---
    def toggle_draw_mode(self):
        self.is_drawing_mode = self.btn_draw_mode.isChecked()
        vis = self.is_drawing_mode
        self.btn_clear_draw.setVisible(vis)
        self.btn_save_map.setVisible(vis)
        self.chk_smooth.setVisible(vis)
        self.btn_break_line.setVisible(vis) 
        self.btn_eraser.setVisible(vis)
        self.btn_arc_mode.setVisible(vis)
        self.btn_flip_arc.setVisible(vis)
        self.btn_edit_map.setEnabled(not vis) 
        
        self.btn_startstop.setEnabled(not vis)
        self.cb_map.setEnabled(not vis)
        self.btn_del_map.setEnabled(not vis)
        
        if vis:
            self.running = False
            self.btn_startstop.setChecked(False)
            self.btn_startstop.setText("START Simulation")
            
            if not self.is_editing_existing:
                self.log.append(">> CREATE MODE: Left Click to add points.")
                self.drawing_paths = [[]] 
                self.path_metadata = [[]] 
                self.path_styles = [self.chk_smooth.isChecked()]
            else:
                self.log.append(f">> EDIT MODE: Editing '{self.editing_map_name}'. Add/Erase points.")
            
            self.setCursor(Qt.CrossCursor)
            self.arc_counter = 0
            self.hovered_path_index = -1
            self.hovered_segment_index = -1
        else:
            if self.is_eraser_mode:
                self.btn_eraser.setChecked(False)
                self.is_eraser_mode = False
            self.setCursor(Qt.ArrowCursor)
            
            if self.is_editing_existing:
                self.is_editing_existing = False
                self.editing_map_name = ""
                self.btn_edit_map.setText("✎ EDIT Current")
                
            self.log.append(">> Exited Draw Mode")

    def edit_current_map(self):
        name = self.cb_map.currentText()
        if name in DEFAULT_MAPS:
            self.log.append("!! Cannot edit Default Maps. Please create a new one.")
            return

        points = map_manager.maps[name]
        
        reconstructed_paths = []
        current_segment = []
        for p in points:
            if p is None:
                if current_segment:
                    reconstructed_paths.append(current_segment)
                    current_segment = []
            else:
                current_segment.append(p)
        if current_segment:
            reconstructed_paths.append(current_segment)

        if not reconstructed_paths:
            self.log.append("!! Map data is empty or invalid.")
            return

        self.is_editing_existing = True
        self.editing_map_name = name
        
        self.btn_draw_mode.setChecked(True)
        self.toggle_draw_mode() 

        self.drawing_paths = reconstructed_paths
        
        # Vì dữ liệu lưu không giữ metadata, gán mặc định là LINE
        self.path_metadata = [["LINE"] * (len(seg)-1) for seg in self.drawing_paths]
        # Gán style mặc định là thẳng (False)
        self.path_styles = [False] * len(self.drawing_paths)

        self.log.append(f">> Loaded '{name}' into Editor.")
        self.step()

    def toggle_arc_mode(self):
        self.is_arc_mode = self.btn_arc_mode.isChecked()
        if self.is_arc_mode:
            self.btn_arc_mode.setText("Mode: Semicircle (Vòng cung)")
            self.log.append(">> Mode: Semicircle. Click next point to create arc.")
        else:
            self.btn_arc_mode.setText("Mode: Line (Thẳng)")
            self.log.append(">> Mode: Straight Line.")
    
    def update_current_path_style(self):
        """
        Khi đổi style (Smooth/Straight), tự động ngắt nét cũ để áp dụng style mới cho nét tiếp theo.
        """
        if not self.is_drawing_mode:
            return

        current_path = self.drawing_paths[-1]

        # Case 1: Nét hiện tại chưa có điểm -> Đổi style luôn
        if not current_path:
            if self.path_styles:
                self.path_styles[-1] = self.chk_smooth.isChecked()
        
        # Case 2: Nét hiện tại đã vẽ -> Ngắt ra tạo nét mới
        else:
            last_point = current_path[-1] 
            
            self.drawing_paths.append([last_point]) 
            self.path_metadata.append([])
            self.path_styles.append(self.chk_smooth.isChecked()) 
            
            self.log.append(">> Auto-split path to change style.")

        self.step() 

    def flip_arc_direction(self):
        self.arc_direction *= -1
        self.log.append(f">> Arc Direction Flipped ({'CW' if self.arc_direction > 0 else 'CCW'})")
        self.step()

    def toggle_eraser(self):
        self.is_eraser_mode = self.btn_eraser.isChecked()
        if self.is_eraser_mode:
            self.log.append(">> ERASER ON: Hover over a line or arc to delete it.")
            self.setCursor(Qt.ForbiddenCursor)
        else:
            self.log.append(">> ERASER OFF: Back to drawing.")
            self.setCursor(Qt.CrossCursor)
            self.hovered_path_index = -1
            self.hovered_segment_index = -1

    def break_current_line(self):
        if self.is_drawing_mode and self.drawing_paths[-1]:
            self.drawing_paths.append([])
            self.path_metadata.append([])
            self.path_styles.append(self.chk_smooth.isChecked())
            self.log.append(">> Line Broken. Starting new segment.")

    def clear_drawing(self):
        self.drawing_paths = [[]]
        self.path_metadata = [[]]
        self.path_styles = [self.chk_smooth.isChecked()]
        self.log.append(">> Drawing Cleared")
        self.step() 

    def save_custom_map(self):
        total_points = sum(len(p) for p in self.drawing_paths)
        if total_points < 2:
            self.log.append("!! Error: Map needs at least 2 points.")
            return
        
        final_pts = []
        for idx, segment in enumerate(self.drawing_paths):
            if not segment: continue
            
            is_smooth = False
            if idx < len(self.path_styles):
                is_smooth = self.path_styles[idx]

            seg_processed = segment
            if is_smooth:
                seg_processed = chaikin_smooth(segment, iterations=4)
            
            final_pts.extend(seg_processed)
            final_pts.append(None)
            
        if final_pts and final_pts[-1] is None: final_pts.pop()
        valid_pts = [p for p in final_pts if p is not None]
        if not valid_pts: return
        
        # Case 1: Edit Mode -> Ghi đè
        if self.is_editing_existing and self.editing_map_name:
            name = self.editing_map_name
            map_manager.add_map(name, final_pts) 
            self.log.append(f">> Updated Existing Map: {name} successfully!")
            
        # Case 2: Create Mode -> Tạo tên mới và Normalize toạ độ
        else:
            min_x = min(p[0] for p in valid_pts)
            min_y = min(p[1] for p in valid_pts)
            normalized_pts = []
            for p in final_pts:
                if p is None: normalized_pts.append(None)
                else: normalized_pts.append((p[0] - min_x, p[1] - min_y))
                
            count = len([k for k in map_manager.maps.keys() if "My Map" in k]) + 1
            name = f"My Map {count} (Mixed)"
            map_manager.add_map(name, normalized_pts)
            self.log.append(f">> Created New Map: {name}!")
        
        self.update_map_list()
        self.cb_map.setCurrentText(name) 
        
        self.drawing_paths = [[]]
        self.path_metadata = [[]]
        self.path_styles = [self.chk_smooth.isChecked()]
        
        if self.btn_draw_mode.isChecked():
            self.btn_draw_mode.setChecked(False)
            self.toggle_draw_mode()
        
        self.change_map()

    # --- EVENTS ---
    def _get_transform_params(self):
        w, h = self.canvas_label.width(), self.canvas_label.height()
        has_points = False
        if not self.is_drawing_mode: has_points = bool(self.track.points)
        else: has_points = any(len(p) > 0 for p in self.drawing_paths)
        if self.is_drawing_mode: 
            base_scale = 1.0; base_ox = w / 2; base_oy = h / 2
        elif self.fit_view and has_points:
            minx, miny, maxx, maxy = self.track.bounds()
            pad = 40; track_w = maxx - minx if maxx > minx else 1; track_h = maxy - miny if maxy > miny else 1
            sx = (w - 2 * pad) / track_w; sy = (h - 2 * pad) / track_h
            base_scale = min(sx, sy)
            base_ox = (w - track_w * base_scale) / 2 - minx * base_scale
            base_oy = (h - track_h * base_scale) / 2 - miny * base_scale
        else: base_scale = 1.0; base_ox = 0; base_oy = 0
        scale = base_scale * self.view_scale
        ox = base_ox + self.view_offset[0]
        oy = base_oy + self.view_offset[1]
        if scale == 0: scale = 1
        return scale, ox, oy

    def eventFilter(self, obj, event):
        if obj is self.canvas_label:
            et = event.type()
            scale, ox, oy = self._get_transform_params()
            
            if et == QtCore.QEvent.MouseMove:
                p = event.pos()
                wx = (p.x() - ox) / scale
                wy = (p.y() - oy) / scale
                self.mouse_world_pos = (wx, wy)

                if self.is_drawing_mode:
                    hit_threshold = 10.0 / scale
                    closest_idx = -1; closest_seg_idx = -1; min_dist = float('inf')
                    for idx, segment in enumerate(self.drawing_paths):
                        if len(segment) < 2: continue
                        for i in range(len(segment)-1):
                            d2 = dist_sq_point_to_segment(wx, wy, segment[i][0], segment[i][1], segment[i+1][0], segment[i+1][1])
                            if d2 < min_dist:
                                min_dist = d2; closest_idx = idx; closest_seg_idx = i
                    if min_dist < (hit_threshold ** 2):
                        self.hovered_path_index = closest_idx; self.hovered_segment_index = closest_seg_idx
                    else:
                        self.hovered_path_index = -1; self.hovered_segment_index = -1

                if self._dragging_car and not self.is_drawing_mode:
                    self.robot.car.x = wx + self._drag_offset[0]
                    self.robot.car.y = wy + self._drag_offset[1]; return True
                if self._panning and self._last_mouse_pos is not None:
                    dx = p.x() - self._last_mouse_pos.x(); dy = p.y() - self._last_mouse_pos.y()
                    self.view_offset = (self.view_offset[0] + dx, self.view_offset[1] + dy)
                    self._last_mouse_pos = p; return True
                if not self.running and not self.is_drawing_mode:
                    car_sx = self.robot.car.x * scale + ox; car_sy = self.robot.car.y * scale + oy
                    if math.hypot(p.x() - car_sx, p.y() - car_sy) < 40.0:
                        if not self._is_hovering_car: self.setCursor(Qt.OpenHandCursor); self._is_hovering_car = True
                    else:
                        if self._is_hovering_car: self.setCursor(Qt.ArrowCursor); self._is_hovering_car = False
            
            if et == QtCore.QEvent.MouseButtonPress:
                btn = event.button(); p = event.pos()
                wx = (p.x() - ox) / scale; wy = (p.y() - oy) / scale

                if self.is_drawing_mode:
                    if btn == Qt.LeftButton:
                        # ==============================================================================
                        # ERASER LOGIC
                        # ==============================================================================
                        if self.is_eraser_mode and self.hovered_path_index != -1 and self.hovered_segment_index != -1:
                            path_idx = self.hovered_path_index
                            seg_idx = self.hovered_segment_index
                            
                            current_path = self.drawing_paths[path_idx]
                            current_meta = self.path_metadata[path_idx]
                            
                            current_style = False
                            if path_idx < len(self.path_styles):
                                current_style = self.path_styles[path_idx]

                            target_type = current_meta[seg_idx]
                            start_seg = seg_idx
                            end_seg = seg_idx
                            
                            if isinstance(target_type, str) and target_type.startswith("ARC_"):
                                while start_seg > 0:
                                    if current_meta[start_seg - 1] == target_type:
                                        start_seg -= 1
                                    else:
                                        break
                                while end_seg < len(current_meta) - 1:
                                    if current_meta[end_seg + 1] == target_type:
                                        end_seg += 1
                                    else:
                                        break
                            
                            # Cắt path
                            left_pts = current_path[:start_seg + 1]
                            left_meta = current_meta[:start_seg]
                            right_pts = current_path[end_seg + 1:]
                            right_meta = current_meta[end_seg + 1:]
                            
                            del self.drawing_paths[path_idx]
                            del self.path_metadata[path_idx]
                            if path_idx < len(self.path_styles):
                                del self.path_styles[path_idx]
                            
                            # Thêm lại các phần còn dư
                            if len(left_pts) >= 2:
                                self.drawing_paths.append(left_pts)
                                self.path_metadata.append(left_meta)
                                self.path_styles.append(current_style)

                            if len(right_pts) >= 2:
                                self.drawing_paths.append(right_pts)
                                self.path_metadata.append(right_meta)
                                self.path_styles.append(current_style)

                            if not self.drawing_paths: 
                                self.drawing_paths = [[]]
                                self.path_metadata = [[]]
                                self.path_styles = [self.chk_smooth.isChecked()]
                            
                            self.hovered_path_index = -1
                            self.hovered_segment_index = -1
                            self.log.append(">> [ERASER] Splitted path & deleted segment successfully.")
                            return True
                        
                        # LOGIC VẼ (ADD POINTS)
                        if not self.is_eraser_mode:
                            if not self.drawing_paths: 
                                self.drawing_paths.append([])
                                self.path_metadata.append([])
                                self.path_styles.append(self.chk_smooth.isChecked())
                            
                            # Nếu vẽ Arc và đã có điểm neo
                            if self.is_arc_mode and self.drawing_paths[-1]:
                                last_pt = self.drawing_paths[-1][-1]
                                arc_pts = generate_semicircle(last_pt, (wx, wy), direction=self.arc_direction, steps=15)
                                
                                arc_id = f"ARC_{self.arc_counter}"
                                self.arc_counter += 1
                                
                                self.drawing_paths[-1].extend(arc_pts[1:])
                                self.path_metadata[-1].extend([arc_id] * (len(arc_pts) - 1))
                                return True
                            else:
                                # Vẽ Line thường
                                self.drawing_paths[-1].append((wx, wy))
                                if len(self.drawing_paths[-1]) > 1:
                                    self.path_metadata[-1].append("LINE")
                                return True
                        
                    if btn == Qt.RightButton:
                        self.break_current_line(); return True
                    if btn == Qt.MiddleButton:
                        self._panning = True; self._last_mouse_pos = p; self.setCursor(Qt.SizeAllCursor); return True
                    return True

                if btn == Qt.LeftButton:
                    if self.running: return True
                    car_sx = self.robot.car.x * scale + ox; car_sy = self.robot.car.y * scale + oy
                    if math.hypot(p.x() - car_sx, p.y() - car_sy) < 40.0:
                        self._dragging_car = True; self._drag_offset = (self.robot.car.x - wx, self.robot.car.y - wy); self.fit_view = False; self.setCursor(Qt.ClosedHandCursor); return True
                if btn == Qt.MiddleButton:
                    self._panning = True; self._last_mouse_pos = p; self.fit_view = False; self.setCursor(Qt.SizeAllCursor); return True

            if et == QtCore.QEvent.MouseButtonRelease:
                if self._dragging_car: self._dragging_car = False; self.setCursor(Qt.OpenHandCursor)
                if self._panning:
                    self._panning = False
                    cursor = Qt.ForbiddenCursor if self.is_eraser_mode else (Qt.CrossCursor if self.is_drawing_mode else Qt.ArrowCursor)
                    self.setCursor(cursor); self._last_mouse_pos = None
            
            if et == QtCore.QEvent.Wheel:
                delta = event.angleDelta().y()
                if delta == 0: return False
                old_scale = self.view_scale; factor = 1.1 ** (delta / 120.0)
                self.view_scale = max(0.05, min(10.0, old_scale * factor))
                self.fit_view = False; return True
        return super().eventFilter(obj, event)

    # --- SIM LOGIC ---
    def update_track_width(self, val):
        self.track.line_half = float(val)
        self.lw_lbl.setText(f"Line Half-Width: {val} cm")
        if not self.running: 
            self.step()

    def update_sensor_radius(self, val):
        self.sensor_base_radius = val / 10.0 
        self.sr_lbl.setText(f"Sensor Radius: {self.sensor_base_radius:.1f}")
        if not self.running: 
            self.step()
            
    def set_car_angle(self, theta_rad):
        self.robot.car.theta = float(theta_rad); self.step(); self.log.append(f">> Set Angle: {math.degrees(theta_rad):.0f}°")
    def do_u_turn(self):
        self.robot.car.theta += math.pi; self.robot.car.theta %= (2 * math.pi); self.step(); self.log.append(">> U-Turn Executed")
    def reset_robot(self):
        self.robot.reset(); self.log.append(">> Robot Reset")
    def toggle_start(self, btn):
        self.running = not self.running
        if self.running: btn.setText("STOP Simulation")
        else: btn.setText("START Simulation")
    
    def step(self):
        dt = 0.03 # 30ms simulation step
        raw_kp = self.kp_s.value() / 100.0; raw_kd = self.kd_s.value() / 100.0 
        sim_kp = raw_kp / 5.0; sim_kd = raw_kd * 2.0; sim_ki = self.ki_s.value() / 1000.0
        mx = self.max_s.value()
        self.robot.update_params(sim_kp, sim_ki, sim_kd, 2.0, 120, mx, 90)
        
        if self.running:
            readout, cmd, err = self.robot.control_step(dt)
            self.robot.physics_step(cmd, dt)
        else:
            readout = self.robot.sensors.read(self.robot.car, self.track)
            cmd = MotorCommand(0, 0); err = self.robot.last_error
            
        for i in range(self.robot.sensors.n):
            if readout.bit[i] == 0: self.sensor_highlight[i] = 1.0
            else: self.sensor_highlight[i] *= 0.1
        self._draw_scene(readout, cmd, err)
        
        status_str = "RUNNING" if self.running else "STOPPED"
        if self.is_drawing_mode: 
            status_str = "DRAWING MODE"
            if self.is_editing_existing: status_str = "EDITING MODE"
            if self.is_eraser_mode: status_str += " (ERASER ON)"
            elif self.is_arc_mode: status_str += " (SEMICIRCLE)"
            
        pos_str = f"{readout.position:.1f}" if readout.position is not None else "NA"
        
        logic_status = "NORMAL"
        if readout.sum_on >= 8 and self.robot.full_line_timer > 0.06:
            logic_status = "STOPPED (FULL LINE)"
        elif readout.sum_on >= 8:
            logic_status = "SLOW (DETECTING...)"
        elif self.robot.is_crossing_tjunc:
            logic_status = f"T-JUNC WAIT ({self.robot.tjunc_timer:.2f}s)"
        elif self.robot.is_force_turning_left:
            logic_status = f"[TURNING BLIND: {self.robot.turn_timer:.2f}s]"
            
        log_text = f">> STATUS: {status_str} | LOGIC: {logic_status}\nSENSORS : {readout.bit_string}\nPOSITION: {pos_str}\nERROR   : {err:.2f}\nPWM (L/R): {cmd.left_pwm} | {cmd.right_pwm}"
        self.log.setPlainText(log_text)

    def _draw_scene(self, readout, cmd, err):
        w = self.canvas_label.width(); h = self.canvas_label.height()
        if w < 10 or h < 10: return
        
        pix = QtGui.QPixmap(w, h); pix.fill(QtGui.QColor("#ffffff"))
        painter = QtGui.QPainter(pix); painter.setRenderHint(QtGui.QPainter.Antialiasing)
        scale, ox, oy = self._get_transform_params()
        def to_view(pt): return QtCore.QPointF(pt[0] * scale + ox, pt[1] * scale + oy)

        painter.setPen(QtGui.QPen(QtGui.QColor(220, 220, 220), 1, Qt.DotLine))
        step = 50 * scale
        if step > 20:
            for x in range(0, w, int(step)): painter.drawLine(x, 0, x, h)
            for y in range(0, h, int(step)): painter.drawLine(0, y, w, y)

        if not self.is_drawing_mode and self.track.points:
            path = QtGui.QPainterPath(); start_new_segment = True
            for p in self.track.points:
                if p is None: start_new_segment = True; continue
                v_pt = to_view(p)
                if start_new_segment: path.moveTo(v_pt); start_new_segment = False
                else: path.lineTo(v_pt)
            
            painter.setPen(QtGui.QPen(QtGui.QColor("black"), int((self.track.line_half*2+2)*scale), Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
            painter.drawPath(path)
            
            painter.setPen(QtGui.QPen(QtGui.QColor("#222"), int(self.track.line_half*2*scale), Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
            painter.drawPath(path)

        if self.is_drawing_mode:
            painter.setPen(Qt.NoPen); painter.setBrush(QtGui.QBrush(QtGui.QColor("#00bcd4")))
            for path_idx, segment in enumerate(self.drawing_paths):
                # Draw Vertices
                painter.setPen(Qt.NoPen); painter.setBrush(QtGui.QBrush(QtGui.QColor("#00bcd4")))
                for p_idx, p in enumerate(segment): painter.drawEllipse(to_view(p), 4, 4)
                
                display_pts = list(segment)
                is_current = (path_idx == len(self.drawing_paths) - 1)
                if is_current and len(display_pts) > 0 and not self.is_eraser_mode:
                      if self.is_arc_mode:
                          last_pt = display_pts[-1]
                          ghost_arc = generate_semicircle(last_pt, self.mouse_world_pos, direction=self.arc_direction, steps=15)
                          display_pts.extend(ghost_arc[1:])
                      else:
                          display_pts.append(self.mouse_world_pos)
                
                if len(display_pts) > 1:
                    is_path_smooth = False
                    if path_idx < len(self.path_styles):
                        is_path_smooth = self.path_styles[path_idx]

                    if is_path_smooth and len(display_pts) > 2:
                        smooth_pts = chaikin_smooth(display_pts, iterations=3)
                        path = QtGui.QPainterPath(); path.moveTo(to_view(smooth_pts[0]))
                        for pt in smooth_pts[1:]: path.lineTo(to_view(pt))
                        is_hovered_path = (path_idx == self.hovered_path_index)
                        
                        color = QtGui.QColor("red") if is_hovered_path else QtGui.QColor("#ff8f00")
                        width = 4 if is_hovered_path else 2
                        painter.setPen(QtGui.QPen(color, width, Qt.DashLine if not is_hovered_path else Qt.SolidLine)); painter.setBrush(Qt.NoBrush); painter.drawPath(path)
                    else:
                        for i in range(len(display_pts) - 1):
                            p1 = display_pts[i]; p2 = display_pts[i+1]
                            is_hovered_seg = (path_idx == self.hovered_path_index and i == self.hovered_segment_index)
                            
                            if is_hovered_seg:
                                painter.setPen(QtGui.QPen(QtGui.QColor("red"), 4, Qt.SolidLine))
                            else:
                                painter.setPen(QtGui.QPen(QtGui.QColor("#ef6c00"), 2, Qt.DashLine))
                            painter.drawLine(to_view(p1), to_view(p2))

        if not self.is_drawing_mode:
            car = self.robot.car
            cx, cy = car.x * scale + ox, car.y * scale + oy
            if self._is_hovering_car or self._dragging_car:
                painter.setPen(QtGui.QPen(QtGui.QColor("#ff9800"), 2, Qt.DashLine)); painter.setBrush(QtGui.QBrush(QtGui.QColor(255, 152, 0, 80))); painter.drawEllipse(QtCore.QPointF(cx, cy), 40, 40)
            painter.translate(cx, cy); painter.rotate(math.degrees(car.theta))
            
            color = QtGui.QColor("#00838f") if not self._dragging_car else QtGui.QColor("#fbc02d")
            painter.setBrush(QtGui.QBrush(color)); painter.setPen(QtGui.QPen(QtGui.QColor("#006064"), 2))
            
            back_shift = 20 * scale 
            head_x_center = 22 * scale - back_shift; head_depth = 6 * scale; head_width = 28 * scale
            neck_width = 8 * scale; body_width = 28 * scale
            neck_start_x = 12 * scale - back_shift; head_start_x = head_x_center - head_depth/2 
            rear_x = -6 * scale - back_shift; taper_start_x = 2 * scale - back_shift
            chassis_points = [
                QtCore.QPointF(rear_x, -body_width/2), QtCore.QPointF(taper_start_x, -body_width/2), 
                QtCore.QPointF(neck_start_x, -neck_width/2), QtCore.QPointF(head_start_x, -neck_width/2), 
                QtCore.QPointF(head_start_x, neck_width/2), QtCore.QPointF(neck_start_x, neck_width/2), 
                QtCore.QPointF(taper_start_x, body_width/2), QtCore.QPointF(rear_x, body_width/2)
            ]
            painter.drawPolygon(QtGui.QPolygonF(chassis_points))
            painter.setBrush(QtGui.QBrush(color)); painter.drawRect(QtCore.QRectF(head_x_center - head_depth/2, -head_width/2, head_depth, head_width))
            wheel_len = 14 * scale; wheel_width = 8 * scale; wheel_offset_y = body_width/2 + wheel_width/2 
            
            painter.setPen(QtGui.QPen(QtGui.QColor("#999"), 1)); painter.setBrush(QtGui.QBrush(QtGui.QColor("#424242")))
            painter.drawRect(QtCore.QRectF(rear_x, -wheel_offset_y - wheel_width/2, wheel_len, wheel_width)); painter.drawRect(QtCore.QRectF(rear_x, wheel_offset_y - wheel_width/2, wheel_len, wheel_width))
            
            painter.setBrush(QtGui.QBrush(QtGui.QColor("#d32f2f"))); painter.setPen(Qt.NoPen)
            painter.drawPolygon(QtGui.QPolygonF([QtCore.QPointF(taper_start_x + 6*scale, 0), QtCore.QPointF(taper_start_x, 3*scale), QtCore.QPointF(taper_start_x, -3*scale)]))
            painter.rotate(-math.degrees(car.theta)); painter.translate(-cx, -cy)
            
            # --- VẼ CẢM BIẾN ---
            for i in range(self.robot.sensors.n):
                sx = (car.x + math.cos(car.theta) * self.robot.sensors.sensor_offset_x - math.sin(car.theta) * self.robot.sensors.sensor_offsets_y[i]) * scale + ox
                sy = (car.y + math.sin(car.theta) * self.robot.sensors.sensor_offset_x + math.cos(car.theta) * self.robot.sensors.sensor_offsets_y[i]) * scale + oy
                hl = self.sensor_highlight[i]
                
                base_r = self.sensor_base_radius * scale
                if hl > 0.1: painter.setBrush(QtGui.QColor(211, 47, 47, int(255*hl))); painter.setPen(Qt.NoPen); painter.drawEllipse(QtCore.QPointF(sx, sy), base_r * 1.8, base_r * 1.8)
                else: painter.setBrush(QtGui.QColor("#388e3c")); painter.setPen(QtGui.QPen(QtGui.QColor("white"), 1.5)); painter.drawEllipse(QtCore.QPointF(sx, sy), base_r * 1.5, base_r * 1.5)

        font = QtGui.QFont("Consolas", 12, QtGui.QFont.Bold); painter.setFont(font); 
        text_color = QtGui.QColor("#000000") 
        painter.setPen(text_color); 
        hud_x, hud_y = 20, 30
        
        if self.is_drawing_mode:
            header_text = f"MODE: {'EDITING MAP' if self.is_editing_existing else 'CREATE NEW MAP'}"
            painter.drawText(hud_x, hud_y, header_text)
            if self.is_eraser_mode: 
                painter.setPen(QtGui.QColor("#d32f2f")); 
                painter.drawText(hud_x + 200, hud_y, "[ERASER ON] Click to delete Arc/Line.")
            else: 
                mode_text = "SEMICIRCLE" if self.is_arc_mode else "LINE"; 
                painter.setPen(QtGui.QColor("#0d47a1")); 
                painter.drawText(hud_x + 200, hud_y, f"Tool: {mode_text} (L-Click: Add | R-Click: Break)")
            
            n_paths = len(self.drawing_paths); n_pts = sum(len(p) for p in self.drawing_paths); 
            painter.setPen(QtGui.QColor("#006064")); 
            painter.drawText(hud_x, hud_y + 20, f"Segments: {n_paths} | Total Points: {n_pts}")
        else:
            logic_mode_text = ""
            if self.robot.full_line_timer > 0.06 and readout.sum_on >= 8:
                 painter.setPen(QtGui.QColor("#b71c1c")) 
                 logic_mode_text = "[STOPPED - FULL LINE]"
            elif readout.sum_on >= 8:
                 painter.setPen(QtGui.QColor("#ff6f00")) 
                 logic_mode_text = f"[SLOWING... {self.robot.full_line_timer:.2f}s]"
            elif self.robot.is_crossing_tjunc:
                painter.setPen(QtGui.QColor("#e65100"))
                logic_mode_text = f"[WAITING: {self.robot.tjunc_timer:.1f}s]"
            elif self.robot.is_force_turning_left:
                painter.setPen(QtGui.QColor("#d50000"))
                logic_mode_text = f"[TURNING BLIND: {self.robot.turn_timer:.2f}s]"
            else:
                painter.setPen(QtGui.QColor("#000000"))
                logic_mode_text = "[PID RUNNING]"
            
            painter.drawText(hud_x, hud_y, f"SENSORS: [{readout.bit_string}]  {logic_mode_text}"); 
            painter.setPen(QtGui.QColor("#1565c0")); 
            painter.drawText(hud_x, hud_y + 20, f"ERROR: {err:.2f} | PWM: {cmd.left_pwm} / {cmd.right_pwm}")
        painter.end(); self.canvas_label.setPixmap(pix)

def main():
    app = QtWidgets.QApplication(sys.argv); app.setStyle("Fusion"); sim = Simulator(); sim.show(); sys.exit(app.exec_())

if __name__ == "__main__":
    main()