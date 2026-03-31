"""
╔══════════════════════════════════════════════════════════════════════╗
║       ProNAV Missile Interception Simulation — 3D / Pygame           ║
║  Thuật toán: Proportional Navigation Guidance (ProNAV)               ║
║  Hiển thị:  3D view (trái) + 3 mặt phẳng chiếu XY/XZ/YZ (phải)     ║
╚══════════════════════════════════════════════════════════════════════╝

Điều khiển:
  SPACE          — Tạm dừng / Tiếp tục
  R              — Reset mô phỏng
  ↑ / ↓          — Tăng / Giảm time scale (tua nhanh / chậm)
  ← / →          — Scrub: lùi / tiến SCRUB_STEP frame (xem chậm)
  Shift+← / →   — Scrub: lùi / tiến SCRUB_STEP_FAST frames (nhảy nhanh)
  Ctrl +← / →   — Scrub: lùi / tiến SCRUB_STEP_MED frames
  N / M          — Tăng / Giảm hệ số dẫn đường N
  1 / 2          — Đổi chế độ di chuyển mục tiêu (thẳng / cơ động)
  +/- (KP)       — Tăng / Giảm vận tốc missile
  [ / ]          — Giảm / Tăng vận tốc target
  W / S          — Tăng / Giảm vận tốc target theo X (Tiến/Lùi)
  A / D          — Tăng / Giảm vận tốc target theo Y (Phải/Trái)
  Q / E          — Giảm / Tăng vận tốc target theo Z (Xuống/Lên)
  Chuột Trái     — Xoay Camera 3D
"""

import pygame
import math
import sys
import random
from collections import deque
from copy import deepcopy
from dataclasses import dataclass, field
from enum import Enum
from typing import List


# ─────────────────────────────────────────────────────────────────────────────
# PARAMETERS  ← chỉnh tại đây để thay đổi giá trị mặc định toàn cục
# ─────────────────────────────────────────────────────────────────────────────

# ── Tốc độ & gia tốc ───────────────────────────────────────────────
TARGET_SPEED = 2      # m/s  (0 = target đứng yên)
TARGET_ACCEL_MAX = TARGET_SPEED * 2      # m/s² (giới hạn gia tốc cơ động)
MISSILE_SPEED = 150     # m/s
MISSILE_ACCEL_MAX = 15     # m/s²

# ── Khoảng cách bắt đầu (thông tin, không điều chỉnh vị trí) ─────────
DISTANCE_TO_TARGET_X = 2000   # m
DISTANCE_TO_TARGET_Y = 1000   # m
DISTANCE_TO_TARGET_Z = 500 # m

# ── Vị trí xuất phát (mét) ───────────────────────────────────────────

TARGET_START_X    =  0   # m (Gốc tọa độ)
TARGET_START_Y    =  0   # m
TARGET_START_Z    =  0   # m (Z > 0 là lên trên)

MISSILE_START_X   = DISTANCE_TO_TARGET_X   # m 
MISSILE_START_Y   = DISTANCE_TO_TARGET_Y   # m 
MISSILE_START_Z   = DISTANCE_TO_TARGET_Z   # m 

 

# ── Hướng bay ban đầu mục tiêu (được normalize tự động) ───────────────
TARGET_VEL_DIR_X  = -1.0
TARGET_VEL_DIR_Y  = -0.8
TARGET_VEL_DIR_Z  = 0.0

# ── Tham số dẫn đường & cơ động ───────────────────────────────────
NAV_GAIN          = 4.0     # hệ số ProNAV (thường 3–5)
NAV_GAIN_MAX      = 12.0    # giới hạn tăng nav gain
NAV_AUG_GAIN      = 0.5     # hệ số bù gia tốc mục tiêu (Augmented PN)
TARGET_MODE_INIT  = 1       # 1 = bay thẳng,  2 = cơ động (evasive)
MANEUVER_PERIOD   = 2.0     # s  — chu kỳ cơ động
INTERCEPT_RADIUS  = 1.0    # m  — bán kính tính là đánh chặn
 

# ── Subsampling quỹ đạo đầy đủ ──────────────────────────────────────
FULL_TRAIL_SUBSAMPLE = 4    # lưu 1 điểm vào full_trail mỗi N lần update

# ─────────────────────────────────────────────────────────────────────────────
# CONSTANTS & COLOURS
# ─────────────────────────────────────────────────────────────────────────────
# Layout: 3D view bên trái, 3 mặt phẳng chiếu bên phải
MAIN_W, MAIN_H   = 880, 720   # cửa sổ 3D chính
PANEL_W          = 400        # độ rộng cột panels bên phải
WIDTH, HEIGHT     = MAIN_W + PANEL_W, MAIN_H
FPS              = 60
BG_COLOR         = (10, 12, 20)
PANEL_BG         = (14, 18, 30)

# Palette
C_MISSILE_BODY  = (255, 80,  60)
C_MISSILE_TRAIL = (255, 140, 60)
C_TARGET_BODY   = (60,  200, 255)
C_TARGET_TRAIL  = (60,  130, 255)
C_VEL_ARROW     = (255, 255,  80)
C_LOS_LINE      = (100, 255, 100)
C_GRID          = (25,  35,  55)
C_TEXT_HEAD     = (200, 220, 255)
C_TEXT_VAL      = (140, 200, 140)
C_TEXT_WARN     = (255, 160,  60)
C_INTERCEPT     = (255, 255,   0)
C_PANEL_BORDER  = (40,  60,  100)
C_PANEL_GRID    = (30,  45,  70)
C_PLANE_XY      = (60,  180, 120)   # màu khung XY
C_PLANE_XZ      = (180, 100, 220)   # màu khung XZ
C_PLANE_YZ      = (220, 160,  50)   # màu khung YZ

TRAIL_LEN        = 6000   # số điểm lưu quỹ đạo (tăng lên để trail dài hơn)
HISTORY_MAX      = 1800  # số snapshot tối đa (~30s ở 60fps)
SNAPSHOT_EVERY   = 2     # lưu snapshot mỗi N frame (giảm xuống để scrub mịn hơn)
SCRUB_STEP       = 10     # ← → : 1 frame (xem từng khung hình)
SCRUB_STEP_MED   = 50    # Ctrl+← → : 10 frames
SCRUB_STEP_FAST  = 100    # Shift+← → : 30 frames (nhảy nhanh)

# Trail gradient colours (tail → head)
C_TARGET_TRAIL_TAIL  = (10,  50, 120)   # đầu cũ — xanh đậm
C_TARGET_TRAIL_HEAD  = (80, 230, 255)   # đầu mới — cyan sáng
C_MISSILE_TRAIL_TAIL = (100, 20,   5)   # đầu cũ — đỏ đậm
C_MISSILE_TRAIL_HEAD = (255, 200,  40)  # đầu mới — vàng cam sáng

# Màu scrubber
C_SCRUB_BG   = (20, 28, 50)
C_SCRUB_FILL = (70, 130, 220)
C_SCRUB_HEAD = (180, 210, 255)
C_SCRUB_REWD = (255, 200, 60)


# ─────────────────────────────────────────────────────────────────────────────
# VECTOR 3D
# ─────────────────────────────────────────────────────────────────────────────
class Vector3D:
    """Vector 3 chiều đơn giản, hỗ trợ các phép toán cơ bản."""

    __slots__ = ("x", "y", "z")

    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)

    def __add__(self, o):  return Vector3D(self.x+o.x, self.y+o.y, self.z+o.z)
    def __sub__(self, o):  return Vector3D(self.x-o.x, self.y-o.y, self.z-o.z)
    def __mul__(self, s):  return Vector3D(self.x*s,   self.y*s,   self.z*s)
    def __rmul__(self, s): return self.__mul__(s)
    def __truediv__(self, s): return Vector3D(self.x/s, self.y/s, self.z/s)
    def __neg__(self):     return Vector3D(-self.x, -self.y, -self.z)
    def __repr__(self):    return f"V3({self.x:.2f},{self.y:.2f},{self.z:.2f})"

    def norm(self) -> float:
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)

    def normalized(self) -> "Vector3D":
        n = self.norm()
        return self / n if n > 1e-9 else Vector3D()

    def dot(self, o) -> float:
        return self.x*o.x + self.y*o.y + self.z*o.z

    def cross(self, o) -> "Vector3D":
        return Vector3D(
            self.y*o.z - self.z*o.y,
            self.z*o.x - self.x*o.z,
            self.x*o.y - self.y*o.x,
        )

    def copy(self) -> "Vector3D":
        return Vector3D(self.x, self.y, self.z)


# ─────────────────────────────────────────────────────────────────────────────
# 3D → 2D PROJECTION  (Isometric / oblique camera)
# ─────────────────────────────────────────────────────────────────────────────
class Camera:
    def __init__(self, cx, cy, scale=0.45,
                 yaw=math.radians(40), pitch=math.radians(28)):
        self.cx    = cx
        self.cy    = cy
        self.scale = scale
        self.set_angles(yaw, pitch)

    def set_angles(self, yaw, pitch):
        self.yaw   = yaw
        # Clamp pitch to prevent flipping (e.g. -85 to 85 degrees)
        self.pitch = max(-math.radians(88), min(math.radians(88), pitch))
        
        self._cos_y = math.cos(self.yaw)
        self._sin_y = math.sin(self.yaw)
        self._cos_x = math.cos(self.pitch)
        self._sin_x = math.sin(self.pitch)

    def project(self, v: Vector3D) -> tuple:
        # Rotation around Z (yaw)
        rx = v.x * self._cos_y - v.y * self._sin_y
        ry = v.x * self._sin_y + v.y * self._cos_y
        # Rotation around X' (pitch)
        rz = v.z * self._cos_x - ry * self._sin_x
        # 2D Screen Mapping
        sx = int(self.cx + rx  * self.scale)
        sy = int(self.cy - rz  * self.scale)
        return sx, sy


# ─────────────────────────────────────────────────────────────────────────────
# PROJECTION PLANE PANEL (chiếu 2D trong 1 mặt phẳng)
# ─────────────────────────────────────────────────────────────────────────────
class PlanePanel:
    """
    Hiển thị vị trí tên lửa & mục tiêu chiếu lên 1 mặt phẳng 2D.
    axes = ('x','y') | ('x','z') | ('y','z')
    """

    def __init__(self, rect: pygame.Rect, axes: tuple, label: str,
                 color: tuple, font_sm, font_md, world_range=3000):
        self.rect    = rect      # pygame.Rect vị trí panel
        self.axes    = axes      # ví dụ ('x','y')
        self.label   = label     # ví dụ "XY"
        self.color   = color     # màu viền panel
        self.font_sm = font_sm
        self.font_md = font_md
        self.world_range = world_range

        # Nội dung panel (surface con) — cần SRCALPHA để trail alpha hoạt động
        self.surf = pygame.Surface((rect.width, rect.height), pygame.SRCALPHA)

        # Margin để dành cho trục
        self.margin = 32
        self.inner_w = rect.width  - self.margin * 2
        self.inner_h = rect.height - self.margin * 2

    def _world_to_panel(self, wx, wy):
        """Chuyển tọa độ thế giới → pixel trong panel."""
        r  = self.world_range
        px = self.margin + int((wx + r) / (2 * r) * self.inner_w)
        py = self.margin + int((1 - (wy + r) / (2 * r)) * self.inner_h)
        return px, py

    def _get_coords(self, pos: Vector3D):
        """Lấy 2 giá trị theo axes."""
        return getattr(pos, self.axes[0]), getattr(pos, self.axes[1])

    def _draw_full_trail(self, s, pts, color_tail, color_head):
        """Vẽ toàn bộ quỹ đạo (full_trail) với gradient màu."""
        n = len(pts)
        if n < 2:
            return
        r0, g0, b0 = color_tail
        r1, g1, b1 = color_head
        for i in range(1, n):
            frac = i / n
            r = int(r0 + (r1 - r0) * frac)
            g = int(g0 + (g1 - g0) * frac)
            b = int(b0 + (b1 - b0) * frac)
            a = int(30 + frac * 210)        # alpha: mờ → rực
            w = 1 if frac < 0.88 else 2    # dày ở đoạn cuối
            p0 = self._world_to_panel(*self._get_coords(pts[i - 1]))
            p1 = self._world_to_panel(*self._get_coords(pts[i]))
            pygame.draw.line(s, (r, g, b, a), p0, p1, w)

    def draw(self, screen: pygame.Surface,
             missile_pos: Vector3D, missile_vel: Vector3D,
             missile_trail, missile_full_trail,
             target_pos:  Vector3D, target_vel:  Vector3D,
             target_trail, target_full_trail):

        s = self.surf
        s.fill((*PANEL_BG, 255))  # fill cần đủ 4 kênh khi dùng SRCALPHA

        # ── Lưới nội ──────────────────────────────────────────────────────
        grid_steps = 6
        r = self.world_range
        step_world = (2 * r) / grid_steps
        for i in range(grid_steps + 1):
            val = -r + i * step_world
            # đường dọc
            px, _ = self._world_to_panel(val, 0)
            pygame.draw.line(s, C_PANEL_GRID, (px, self.margin),
                             (px, self.rect.height - self.margin), 1)
            # đường ngang
            _, py = self._world_to_panel(0, val)
            pygame.draw.line(s, C_PANEL_GRID, (self.margin, py),
                             (self.rect.width - self.margin, py), 1)

        # Trục gốc (nét sáng hơn — dùng màu riêng thay vì alpha tuple)
        ox, oy = self._world_to_panel(0, 0)
        axis_color = (50, 72, 110)   # sáng hơn C_PANEL_GRID một chút
        pygame.draw.line(s, axis_color,
                         (ox, self.margin), (ox, self.rect.height - self.margin), 1)
        pygame.draw.line(s, axis_color,
                         (self.margin, oy), (self.rect.width - self.margin, oy), 1)

        # ── Nhãn trục ─────────────────────────────────────────────────
        ax_lbl = self.font_sm.render(self.axes[0].upper(), True, (120, 160, 200))
        ay_lbl = self.font_sm.render(self.axes[1].upper(), True, (120, 160, 200))
        # Clamp nhãn vào trong panel — tránh vẽ ngoài viền
        ax_x = min(self.rect.width - self.margin + 2, self.rect.width - ax_lbl.get_width() - 2)
        s.blit(ax_lbl, (ax_x, max(oy - 8, 0)))
        ay_y = max(self.margin - 16, 0)
        s.blit(ay_lbl, (max(ox - 8, 0), ay_y))

        # ── Full trail TARGET (từ đầu đến hiện tại / va chạm) ───────────────
        self._draw_full_trail(s, target_full_trail,
                              C_TARGET_TRAIL_TAIL, C_TARGET_TRAIL_HEAD)

        # ── Full trail MISSILE (từ launch đến đánh chặn) ─────────────────
        self._draw_full_trail(s, missile_full_trail,
                              C_MISSILE_TRAIL_TAIL, C_MISSILE_TRAIL_HEAD)

        # ── Vận tốc target ─────────────────────────────────────────────────
        tx, ty = self._get_coords(target_pos)
        tvx, tvy = self._get_coords(target_vel)
        tp0 = self._world_to_panel(tx, ty)
        scale_vel = 200 / max(target_vel.norm(), 1)
        tp1 = self._world_to_panel(tx + tvx * scale_vel, ty + tvy * scale_vel)
        pygame.draw.line(s, C_VEL_ARROW, tp0, tp1, 2)
        pygame.draw.circle(s, C_VEL_ARROW, tp1, 3)

        # ── Vận tốc missile ────────────────────────────────────────────────
        mx, my = self._get_coords(missile_pos)
        mvx, mvy = self._get_coords(missile_vel)
        mp0 = self._world_to_panel(mx, my)
        scale_mvel = 200 / max(missile_vel.norm(), 1)
        mp1 = self._world_to_panel(mx + mvx * scale_mvel, my + mvy * scale_mvel)
        pygame.draw.line(s, (255, 160, 80), mp0, mp1, 2)
        pygame.draw.circle(s, (255, 160, 80), mp1, 3)

        # ── LOS line ───────────────────────────────────────────────────
        los_color = tuple(int(c * 0.25) for c in C_LOS_LINE)  # dim 25% thay cho alpha
        pygame.draw.line(s, los_color, tp0, mp0, 1)

        # ── Vẽ target ──────────────────────────────────────────────────────
        pygame.draw.circle(s, C_TARGET_BODY, tp0, 7)
        pygame.draw.circle(s, (255, 255, 255), tp0, 7, 1)
        lbl = self.font_sm.render("T", True, C_TARGET_BODY)
        s.blit(lbl, (tp0[0] + 9, tp0[1] - 7))

        # ── Vẽ missile ────────────────────────────────────────────────────
        pygame.draw.circle(s, C_MISSILE_BODY, mp0, 5)
        pygame.draw.circle(s, (255, 255, 255), mp0, 5, 1)
        lbl = self.font_sm.render("M", True, C_MISSILE_BODY)
        s.blit(lbl, (mp0[0] + 7, mp0[1] - 7))

        # ── Tiêu đề panel ─────────────────────────────────────────────────
        title = self.font_md.render(f"Plane {self.label}", True, self.color)
        s.blit(title, (6, 4))

        # ── Tọa độ hiện tại ────────────────────────────────────────────────
        coord_t = self.font_sm.render(
            f"T: {self.axes[0].upper()}={getattr(target_pos, self.axes[0]):+.0f}"
            f"  {self.axes[1].upper()}={getattr(target_pos, self.axes[1]):+.0f}",
            True, C_TARGET_BODY)
        coord_m = self.font_sm.render(
            f"M: {self.axes[0].upper()}={getattr(missile_pos, self.axes[0]):+.0f}"
            f"  {self.axes[1].upper()}={getattr(missile_pos, self.axes[1]):+.0f}",
            True, C_MISSILE_BODY)
        s.blit(coord_t, (6, self.rect.height - 30))
        s.blit(coord_m, (6, self.rect.height - 16))

        # ── Viền panel ────────────────────────────────────────────────────
        pygame.draw.rect(s, self.color, s.get_rect(), 2)

        screen.blit(s, (self.rect.x, self.rect.y))


# ─────────────────────────────────────────────────────────────────────────────
# TARGET (mục tiêu)
# ─────────────────────────────────────────────────────────────────────────────
class TargetMode(Enum):
    STRAIGHT    = 1
    MANEUVERING = 2


class Target:
    def __init__(self, pos: Vector3D, vel: Vector3D,
                 speed: float = TARGET_SPEED,
                 mode: TargetMode = TargetMode.STRAIGHT,
                 max_accel: float = TARGET_ACCEL_MAX,
                 maneuver_period: float = MANEUVER_PERIOD):
        self.pos_0    = pos.copy()
        self.vel_0    = vel.normalized() * speed if speed > 1e-9 else Vector3D()
        self.pos      = pos.copy()
        self.vel      = self.vel_0.copy()
        self.speed    = speed
        self.mode     = mode
        self.max_accel = max_accel

        self._t        = 0.0
        self._accel    = Vector3D()
        self._full_ctr = 0          # bộ đếm subsample full_trail

        self._maneuver_period = maneuver_period
        self._maneuver_amp    = max_accel * 0.7

        self.trail:      deque = deque(maxlen=TRAIL_LEN)
        self.full_trail: list  = []   # toàn bộ quỹ đạo từ đầu đến hiện tại

    def reset(self):
        self.pos        = self.pos_0.copy()
        self.vel        = self.vel_0.copy()
        self._t         = 0.0
        self._accel     = Vector3D()
        self._full_ctr  = 0
        self.trail      = deque(maxlen=TRAIL_LEN)
        self.full_trail = []

    def update(self, dt: float):
        self._t += dt

        if self.mode == TargetMode.STRAIGHT:
            self._accel = Vector3D()

        elif self.mode == TargetMode.MANEUVERING:
            phase    = 2 * math.pi * self._t / self._maneuver_period
            lat_mag  = self._maneuver_amp * math.sin(phase)
            vert_mag = self._maneuver_amp * 0.4 * math.cos(phase * 0.7)
            vn = self.vel.normalized()
            right = Vector3D(-vn.y, vn.x, 0.0)
            self._accel = right * lat_mag + Vector3D(0, 0, vert_mag)

        self.vel = self.vel + self._accel * dt
        current_speed = self.vel.norm()
        if current_speed > 1e-6:
            self.vel = self.vel.normalized() * self.speed
        elif self.speed > 1e-6:
            # Nếu mục tiêu đứng yên nhưng speed > 0, cho nó bay theo hướng X+ (Forward) mặc định
            self.vel = Vector3D(1.0, 0.0, 0.0) * self.speed

        self.pos = self.pos + self.vel * dt
        self.trail.append(self.pos.copy())

        # Full trail — subsampled (để tránh dùng quá nhiều bộ nhớ)
        self._full_ctr += 1
        if self._full_ctr >= FULL_TRAIL_SUBSAMPLE:
            self._full_ctr = 0
            self.full_trail.append(self.pos.copy())


# ─────────────────────────────────────────────────────────────────────────────
# MISSILE (tên lửa)
# ─────────────────────────────────────────────────────────────────────────────
class Missile:
    def __init__(self, pos: Vector3D, speed: float = MISSILE_SPEED,
                 nav_gain: float = NAV_GAIN,
                 nav_aug_gain: float = NAV_AUG_GAIN,
                 max_accel: float = MISSILE_ACCEL_MAX,
                 intercept_radius: float = INTERCEPT_RADIUS):
        self.pos_0     = pos.copy()
        self.pos       = pos.copy()
        self.speed     = speed
        self.nav_gain  = nav_gain
        self.nav_aug_gain = nav_aug_gain
        self.max_accel = max_accel
        self.vel       = Vector3D()
        self._vel_0    = Vector3D()
        self._prev_los = Vector3D()
        self._accel    = Vector3D()
        self.g_load    = 0.0
        self.Vc        = 0.0
        self.active          = False
        self.intercepted     = False
        self.trail:      deque = deque(maxlen=TRAIL_LEN)
        self.full_trail: list  = []   # toàn bộ quỹ đạo từ launch
        self._full_ctr       = 0
        self.radian_intercept = intercept_radius

    def launch(self, target_pos: Vector3D):
        direction      = (target_pos - self.pos).normalized()
        self.vel       = direction * self.speed
        self._vel_0    = self.vel.copy()
        self._prev_los = direction.copy()
        self.active    = True
        self.intercepted = False
        # Ghi vị trí xuất phát vào full_trail
        self.full_trail = [self.pos.copy()]
        self._full_ctr  = 0

    def reset(self):
        self.pos        = self.pos_0.copy()
        self.vel        = Vector3D()
        self._prev_los  = Vector3D()
        self._accel     = Vector3D()
        self.g_load     = 0.0
        self.Vc         = 0.0
        self.active     = False
        self.intercepted = False
        self.trail      = deque(maxlen=TRAIL_LEN)
        self.full_trail = []
        self._full_ctr  = 0

    def update(self, target: "Target", dt: float):
        if not self.active or self.intercepted:
            return

        r_vec = target.pos - self.pos
        r = r_vec.norm()

        if r < self.radian_intercept:
            self.intercepted = True
            # Ghi điểm cuối khi đánh chặn
            self.full_trail.append(self.pos.copy())
            return

        r_hat = r_vec / r
        v_rel = target.vel - self.vel
        
        # closing velocity Vc = - d/dt (range) = - (r_vec . v_rel) / r
        self.Vc = -(r_vec.dot(v_rel)) / (r + 1e-6)
        # Nếu Vc < 0 (missile đang bay xa hơn), ta vẫn dùng giá trị nhỏ để duy trì dẫn đường
        # nhưng kẹp tối thiểu để tránh số ảo hoặc mất lái hoàn toàn.
        Vc_active = max(self.Vc, 0.1)

        # LOS rate vector: omega = (r x v_rel) / (r^2)
        lambda_dot = r_vec.cross(v_rel) / (r * r + 1e-6)

        # ── PURE PN ─────────────────
        # a_pn = N * Vc * omega x r_hat
        a_pn = lambda_dot.cross(r_hat) * (self.nav_gain * Vc_active)

        # ── TARGET ACCEL (APN augmented) ──────
        a_target      = target._accel
        a_target_perp = a_target - r_hat * a_target.dot(r_hat)
        a_cmd = a_pn + self.nav_aug_gain * (self.nav_gain / 2.0) * a_target_perp

        # Lateral constraint (gia tốc vuông góc với vận tốc tên lửa)
        v_mag = self.vel.norm()
        v_hat = self.vel / v_mag if v_mag > 1e-6 else Vector3D()
        a_cmd = a_cmd - v_hat * a_cmd.dot(v_hat)

        # Clamp gia tốc
        a_mag = a_cmd.norm()
        if a_mag > self.max_accel:
            a_cmd = a_cmd * (self.max_accel / a_mag)

        self._accel = a_cmd
        self.g_load = a_mag / 9.81

        # Tich phân
        self.vel = self.vel + a_cmd * dt
        if self.vel.norm() > 1e-6:
            self.vel = self.vel.normalized() * self.speed

        self.pos = self.pos + self.vel * dt
        self.trail.append(self.pos.copy())

        # Full trail subsampled
        self._full_ctr += 1
        if self._full_ctr >= FULL_TRAIL_SUBSAMPLE:
            self._full_ctr = 0
            self.full_trail.append(self.pos.copy())


# ─────────────────────────────────────────────────────────────────────────────
# SNAPSHOT (lưu trạng thái để tua lại)
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class Snapshot:
    """Ảnh chụp toàn bộ trạng thái mô phỏng tại 1 frame."""
    elapsed_sim: float

    # Missile
    m_pos:   "Vector3D"
    m_vel:   "Vector3D"
    m_accel: "Vector3D"
    m_g_load: float
    m_active:      bool
    m_intercepted: bool
    m_trail: list   # list[Vector3D]
    m_Vc: float

    # Target
    t_pos:   "Vector3D"
    t_vel:   "Vector3D"
    t_accel: "Vector3D"
    t_t:     float
    t_trail: list   # list[Vector3D]

    flash_timer: float


# ─────────────────────────────────────────────────────────────────────────────
# SIMULATION
# ─────────────────────────────────────────────────────────────────────────────
class Simulation:
    """Quản lý toàn bộ vòng lặp mô phỏng và giao diện."""
    # Vị trí xuất phát — được đọc từ PARAMETERS trong __init__

    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        pygame.display.set_caption("ProNAV Missile Simulation — Press H for help")
        self.clock  = pygame.time.Clock()

        # Font
        self.font_sm = pygame.font.SysFont("consolas", 12)
        self.font_md = pygame.font.SysFont("consolas", 14, bold=True)
        self.font_lg = pygame.font.SysFont("consolas", 20, bold=True)

        # Camera (chỉ cho cửa sổ 3D bên trái)
        self.camera = Camera(cx=MAIN_W//2, cy=MAIN_H//2 + 50,
                             scale=0.11,
                             yaw=math.radians(40),
                             pitch=math.radians(28))

        # Mouse Interaction for 3D Orbit Camera
        self._dragging_cam  = False
        self._last_mouse_pos = (0, 0)

        # Surface tạm dùng chung cho trail — tránh alloc mỗi frame
        self._trail_surf = pygame.Surface((MAIN_W, HEIGHT), pygame.SRCALPHA)

        # Chia 3 panel bên phải theo chiều dọc
        ph = HEIGHT // 3
        self.panels = [
            PlanePanel(
                rect   = pygame.Rect(MAIN_W, 0,       PANEL_W, ph),
                axes   = ('x', 'y'),
                label  = "XY  (top-down)",
                color  = C_PLANE_XY,
                font_sm= self.font_sm,
                font_md= self.font_md,
            ),
            PlanePanel(
                rect   = pygame.Rect(MAIN_W, ph,      PANEL_W, ph),
                axes   = ('x', 'z'),
                label  = "XZ  (side)",
                color  = C_PLANE_XZ,
                font_sm= self.font_sm,
                font_md= self.font_md,
            ),
            PlanePanel(
                rect   = pygame.Rect(MAIN_W, ph*2,    PANEL_W, HEIGHT - ph*2),
                axes   = ('y', 'z'),
                label  = "YZ  (front)",
                color  = C_PLANE_YZ,
                font_sm= self.font_sm,
                font_md= self.font_md,
            ),
        ]

        # Tham số runtime — khởi tạo từ PARAMETERS
        self.nav_gain      = float(NAV_GAIN)
        self.missile_speed = float(MISSILE_SPEED)
        self.target_speed  = float(TARGET_SPEED)
        self.max_accel     = float(MISSILE_ACCEL_MAX)
        self.nav_aug_gain  = float(NAV_AUG_GAIN)
        self.time_scale    = 1.0

        # Hướng vận tốc target — lấy từ PARAMETERS
        self.target_vel_dir = Vector3D(TARGET_VEL_DIR_X, TARGET_VEL_DIR_Y, TARGET_VEL_DIR_Z)

        self.target_mode = TargetMode(TARGET_MODE_INIT)
        self.paused      = False
        self.show_help   = False
        self.elapsed_sim = 0.0
        self._flash_timer = 0.0
        self._flash_set   = False   # cờ để chỉ set flash 1 lần

        # ── History / Scrubbing ───────────────────────────────────────────────
        self._history: List[Snapshot] = []   # danh sách snapshot
        self._hist_idx: int = -1             # -1 = live (không scrub)
        self._scrubbing: bool = False        # đang xem lại lịch sử?
        self._frame_counter: int = 0         # đếm frame để snapshot

        self._build_objects()
        # Lưới hiện tại được vẽ trực tiếp trong _draw_grid (dynamic)

    def _build_objects(self):
        # Vị trí xuất phát được đọc từ PARAMETERS mỗi lần reset
        missile_start = Vector3D(MISSILE_START_X, MISSILE_START_Y, MISSILE_START_Z)
        target_start  = Vector3D(TARGET_START_X,  TARGET_START_Y,  TARGET_START_Z)

        self.target = Target(
            pos             = target_start,
            vel             = self.target_vel_dir.copy(),
            speed           = self.target_speed,
            mode            = self.target_mode,
            max_accel       = float(TARGET_ACCEL_MAX),
            maneuver_period = float(MANEUVER_PERIOD),
        )
        # Ghi vị trí xuất phát của target vào full_trail
        self.target.full_trail = [target_start.copy()]

        self.missile = Missile(
            pos              = missile_start,
            speed            = self.missile_speed,
            nav_gain         = self.nav_gain,
            nav_aug_gain     = self.nav_aug_gain,
            max_accel        = self.max_accel,
            intercept_radius = float(INTERCEPT_RADIUS),
        )
        self.missile.launch(self.target.pos)
        self.elapsed_sim = 0.0
        self._flash_timer = 0.0
        self._flash_set   = False

    def reset(self):
        self._history.clear()
        self._hist_idx = -1
        self._scrubbing = False
        self._frame_counter = 0
        self._flash_set = False   # BUG FIX: reset cờ flash để intercept mới hoạt động
        self._build_objects()

    # ── Snapshot helpers ──────────────────────────────────────────────────────
    def _save_snapshot(self):
        """Lưu trạng thái hiện tại vào history."""
        snap = Snapshot(
            elapsed_sim  = self.elapsed_sim,
            m_pos        = self.missile.pos.copy(),
            m_vel        = self.missile.vel.copy(),
            m_accel      = self.missile._accel.copy(),
            m_g_load     = self.missile.g_load,
            m_active     = self.missile.active,
            m_intercepted= self.missile.intercepted,
            m_trail      = list(self.missile.trail),   # deque → list copy
            m_Vc         = self.missile.Vc,
            t_pos        = self.target.pos.copy(),
            t_vel        = self.target.vel.copy(),
            t_accel      = self.target._accel.copy(),
            t_t          = self.target._t,
            t_trail      = list(self.target.trail),    # deque → list copy
            flash_timer  = self._flash_timer,
        )
        # BUG FIX: luôn append vào cuối (chế độ live), giới hạn bộ nhớ bằng list slice
        self._history.append(snap)
        if len(self._history) > HISTORY_MAX:
            self._history.pop(0)
        # Con trỏ luôn ở cuối khi đang live
        self._hist_idx = len(self._history) - 1

    def _restore_snapshot(self, idx: int):
        """Khôi phục trạng thái từ snapshot idx."""
        snap = self._history[idx]
        self.elapsed_sim        = snap.elapsed_sim
        self.missile.pos        = snap.m_pos.copy()
        self.missile.vel        = snap.m_vel.copy()
        self.missile._accel     = snap.m_accel.copy()
        self.missile.g_load     = snap.m_g_load
        self.missile.active     = snap.m_active
        self.missile.intercepted= snap.m_intercepted
        # BUG FIX: restore trail dưới dạng deque đúng kiểu
        self.missile.trail      = deque((v.copy() for v in snap.m_trail), maxlen=TRAIL_LEN)
        self.missile.Vc         = snap.m_Vc
        self.target.pos         = snap.t_pos.copy()
        self.target.vel         = snap.t_vel.copy()
        self.target._accel      = snap.t_accel.copy()
        self.target._t          = snap.t_t
        self.target.trail       = deque((v.copy() for v in snap.t_trail), maxlen=TRAIL_LEN)
        self._flash_timer       = snap.flash_timer

    def _scrub(self, delta: int):
        """Di chuyển con trỏ history ±delta steps."""
        if not self._history:
            return
        # Bắt đầu scrub → tự động pause
        if not self._scrubbing:
            self._scrubbing = True
            self.paused = True
            # Đặt con trỏ về cuối nếu đang live
            if self._hist_idx < 0:
                self._hist_idx = len(self._history) - 1

        new_idx = max(0, min(len(self._history) - 1, self._hist_idx + delta))
        self._hist_idx = new_idx
        self._restore_snapshot(new_idx)

    def _exit_scrub(self):
        """Thoát chế độ scrub, quay về live."""
        self._scrubbing = False
        self._hist_idx  = len(self._history) - 1

    # ── Event Handling ────────────────────────────────────────────────────────
    def handle_events(self) -> bool:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            if event.type == pygame.KEYDOWN:
                k = event.key
                if k == pygame.K_ESCAPE:
                    return False

                elif k == pygame.K_SPACE:
                    self.paused = not self.paused

                elif k == pygame.K_r:
                    self.reset()

                # Time scale
                elif k == pygame.K_UP:
                    self.time_scale = min(self.time_scale * 2.0, 32.0)
                elif k == pygame.K_DOWN:
                    self.time_scale = max(self.time_scale * 0.5, 0.5)

                # Scrub history ← →
                # ← / →          : SCRUB_STEP frame (xem chậm từng khung hình)
                # Shift+← / →   : SCRUB_STEP_FAST frames (nhảy nhanh)
                # Ctrl +← / →   : SCRUB_STEP_MED frames
                elif k == pygame.K_LEFT:
                    mods = pygame.key.get_mods()
                    if mods & pygame.KMOD_SHIFT:
                        step = SCRUB_STEP_FAST
                    elif mods & pygame.KMOD_CTRL:
                        step = SCRUB_STEP_MED
                    else:
                        step = SCRUB_STEP
                    self._scrub(-step)
                elif k == pygame.K_RIGHT:
                    mods = pygame.key.get_mods()
                    if mods & pygame.KMOD_SHIFT:
                        step = SCRUB_STEP_FAST
                    elif mods & pygame.KMOD_CTRL:
                        step = SCRUB_STEP_MED
                    else:
                        step = SCRUB_STEP
                    # Nếu đang ở cuối history → thoát scrub
                    if self._scrubbing and self._hist_idx >= len(self._history) - 1:
                        self._exit_scrub()
                        self.paused = False
                    else:
                        self._scrub(+step)

                # Navigation gain
                elif k == pygame.K_n:
                    self.nav_gain = min(self.nav_gain + 0.5, NAV_GAIN_MAX)
                    self.missile.nav_gain = self.nav_gain
                elif k == pygame.K_m:
                    self.nav_gain = max(self.nav_gain - 0.5, 1.0)
                    self.missile.nav_gain = self.nav_gain

                # Target mode
                elif k == pygame.K_1:
                    self.target_mode = TargetMode.STRAIGHT
                    self.target.mode = TargetMode.STRAIGHT
                elif k == pygame.K_2:
                    self.target_mode = TargetMode.MANEUVERING
                    self.target.mode = TargetMode.MANEUVERING

                # Zoom mô phỏng (3D & Plane) (+ / -)
                elif k == pygame.K_KP_PLUS or k == pygame.K_EQUALS:
                    for panel in self.panels:
                        panel.world_range = max(100, panel.world_range * 0.9)
                    self.camera.scale *= 1.1
                elif k == pygame.K_KP_MINUS or k == pygame.K_MINUS:
                    for panel in self.panels:
                        panel.world_range = min(10000, panel.world_range * 1.1)
                    self.camera.scale *= 0.9

                # Target velocity components (Q/E/A/D/W/S)
                # Q/E: Up/Down (Z)
                elif k == pygame.K_q:
                    self.target.vel.z -= 0.5
                    self.target.speed = self.target.vel.norm()
                elif k == pygame.K_e:
                    self.target.vel.z += 0.5
                    self.target.speed = self.target.vel.norm()

                # A/D: Right(+Y)/Left(-Y)
                elif k == pygame.K_a:
                    self.target.vel.y -= 0.5
                    self.target.speed = self.target.vel.norm()
                elif k == pygame.K_d:
                    self.target.vel.y += 0.5
                    self.target.speed = self.target.vel.norm()

                # W/S: Forward(+X)/Backward(-X)
                elif k == pygame.K_w:
                    self.target.vel.x += 0.5
                    self.target.speed = self.target.vel.norm()
                elif k == pygame.K_s:
                    self.target.vel.x -= 0.5
                    self.target.speed = self.target.vel.norm()

                # Target speed tổng ([ / ]) 
                elif k == pygame.K_LEFTBRACKET:
                    self.target_speed = max(0.0, self.target_speed - 1.0)
                    self.target.speed = self.target_speed
                elif k == pygame.K_RIGHTBRACKET:
                    self.target_speed += 1.0
                    self.target.speed = self.target_speed

                # Help
                elif k == pygame.K_h:
                    self.show_help = not self.show_help

            # Mouse button events for orbit camera
            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1: # Left click
                    mx, my = event.pos
                    # Only rotate if mouse is in the 3D area (left side)
                    if mx < MAIN_W:
                        self._dragging_cam = True
                        self._last_mouse_pos = (mx, my)

            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    self._dragging_cam = False

            elif event.type == pygame.MOUSEMOTION:
                if self._dragging_cam:
                    mx, my = event.pos
                    dx = mx - self._last_mouse_pos[0]
                    dy = my - self._last_mouse_pos[1]
                    
                    # Sensitivity: roughly 0.5 degrees per pixel
                    new_yaw = self.camera.yaw - dx * 0.002
                    new_pitch = self.camera.pitch - dy * 0.002
                    self.camera.set_angles(new_yaw, new_pitch)
                    self._last_mouse_pos = (mx, my)

        return True

    # ── Update ────────────────────────────────────────────────────────────────
    def update(self, dt_real: float):
        if self.paused or self._scrubbing:
            return
        dt = dt_real * self.time_scale
        dt = min(dt, 0.05)
        self.elapsed_sim += dt

        if not self.missile.intercepted:
            self.target.update(dt)
            self.missile.update(self.target, dt)

        # Flash timer luôn đếm ngược (kể cả sau intercept)
        if self._flash_timer > 0:
            self._flash_timer -= dt

        # Kích hoạt flash & tự động pause khi mới intercept
        if self.missile.intercepted and self.elapsed_sim > 0.1:
            if not getattr(self, '_flash_set', False):
                self._flash_timer = 2.5
                self._flash_set   = True
                self.paused       = True  # Tự động dừng simulation

        # BUG FIX: chỉ lưu snapshot khi simulation đang chạy (không phải sau intercept kết thúc)
        self._frame_counter += 1
        if self._frame_counter >= SNAPSHOT_EVERY:
            self._frame_counter = 0
            self._save_snapshot()

    # ── 3D Rendering helpers ──────────────────────────────────────────────────
    def _proj(self, v: Vector3D) -> tuple:
        return self.camera.project(v)

    def _draw_grid(self):
        """Vẽ lưới 3D động theo camera scale."""
        step = 500
        # Tính toán lưới dựa trên camera scale hiện tại
        limit = int(3000 / (self.camera.scale * 10))
        limit = max(3000, min(limit, 8000))
        rng = range(-limit, limit + 1, step)
        
        for xi in rng:
            p1 = self._proj(Vector3D(xi, -limit, 0))
            p2 = self._proj(Vector3D(xi,  limit, 0))
            pygame.draw.line(self.screen, (*C_GRID, 80), p1, p2, 1)
        for yi in rng:
            p1 = self._proj(Vector3D(-limit, yi, 0))
            p2 = self._proj(Vector3D( limit, yi, 0))
            pygame.draw.line(self.screen, (*C_GRID, 80), p1, p2, 1)
        
        p_bot = self._proj(Vector3D(0, 0,    0))
        p_top = self._proj(Vector3D(0, 0, 1500))
        pygame.draw.line(self.screen, (60, 200, 80, 80), p_bot, p_top, 1)

    def _draw_trail(self, trail, color_tail: tuple, color_head: tuple,
                    width_max: int = 3):
        """Vẽ quỹ đạo gradient màu + độ dày tăng dần + glow ở đầu mới."""
        pts = list(trail)          # snapshot deque thread-safe
        n   = len(pts)
        if n < 2:
            return

        surf = self._trail_surf
        surf.fill((0, 0, 0, 0))   # xóa trong suốt

        r0, g0, b0 = color_tail
        r1, g1, b1 = color_head
        glow_thresh = 0.80         # 20% cuối trail được glow

        for i in range(1, n):
            frac = i / n           # 0 = đầu cũ nhất, 1 = mới nhất
            # Gradient màu
            r = int(r0 + (r1 - r0) * frac)
            g = int(g0 + (g1 - g0) * frac)
            b = int(b0 + (b1 - b0) * frac)
            # Alpha: từ 25 → 255
            a = int(25 + frac * 230)
            # Độ dày: 1px (cũ) → width_max px (mới)
            w = max(1, int(frac * width_max + 0.5))

            p0 = self._proj(pts[i - 1])
            p1 = self._proj(pts[i])
            pygame.draw.line(surf, (r, g, b, a), p0, p1, w)

            # Glow layer cho đoạn cuối trail
            if frac > glow_thresh:
                glow_frac = (frac - glow_thresh) / (1.0 - glow_thresh)
                ga = int(glow_frac * 70)
                gw = w + 2
                pygame.draw.line(surf, (r1, g1, b1, ga), p0, p1, gw)

        self.screen.blit(surf, (0, 0))

    def _draw_arrow(self, origin: Vector3D, direction: Vector3D,
                    scale: float, color: tuple, width=2):
        tip = origin + direction.normalized() * scale
        p0  = self._proj(origin)
        p1  = self._proj(tip)
        pygame.draw.line(self.screen, color, p0, p1, width)
        pygame.draw.circle(self.screen, color, p1, 4)

    def _draw_object(self, pos: Vector3D, radius: int,
                     color_body: tuple, color_glow: tuple, label: str):
        px, py = self._proj(pos)
        glow = pygame.Surface((radius*6, radius*6), pygame.SRCALPHA)
        pygame.draw.circle(glow, (*color_glow, 40), (radius*3, radius*3), radius*3)
        self.screen.blit(glow, (px - radius*3, py - radius*3))
        pygame.draw.circle(self.screen, color_body, (px, py), radius)
        pygame.draw.circle(self.screen, (255, 255, 255), (px, py), radius, 1)
        txt = self.font_sm.render(label, True, color_body)
        self.screen.blit(txt, (px + radius + 4, py - 8))

    def _draw_los_line(self):
        if not self.missile.active or self.missile.intercepted:
            return
        p0 = self._proj(self.missile.pos)
        p1 = self._proj(self.target.pos)
        # Tái dùng trail_surf để tránh alloc thêm
        self._trail_surf.fill((0, 0, 0, 0))
        pygame.draw.line(self._trail_surf, (*C_LOS_LINE, 55), p0, p1, 1)
        self.screen.blit(self._trail_surf, (0, 0))

    def _draw_pip(self):
        """Vẽ điểm va chạm dự tính (PIP)."""
        if not self.missile.active or self.missile.intercepted:
            return
        
        r = (self.target.pos - self.missile.pos).norm()
        Vc = self.missile.Vc
        time_to_go = r / (Vc + 1.0) if Vc > 0.1 else r / (self.missile.speed + 1.0)
        
        # PIP = vị trí mục tiêu sau time_to_go giây (giả định vận tốc không đổi)
        pip = self.target.pos + self.target.vel * time_to_go
        px, py = self._proj(pip)
        
        # Vẽ dấu X nhỏ hoặc vòng tròn mờ
        pygame.draw.line(self.screen, (255, 255, 255, 100), (px-5, py-5), (px+5, py+5), 1)
        pygame.draw.line(self.screen, (255, 255, 255, 100), (px+5, py-5), (px-5, py+5), 1)
        pygame.draw.circle(self.screen, (255, 255, 255, 50), (px, py), 10, 1)
        
        txt = self.font_sm.render(f"PIP ({time_to_go:.1f}s)", True, (200, 200, 200))
        self.screen.blit(txt, (px + 10, py - 10))

    def _draw_intercept_flash(self):
        if self._flash_timer <= 0:
            return
        px, py = self._proj(self.missile.pos)
        alpha_frac = self._flash_timer / 2.5
        r = int((1 - alpha_frac) * 80 + 20)
        surf = pygame.Surface((MAIN_W, HEIGHT), pygame.SRCALPHA)
        pygame.draw.circle(surf, (255, 240, 60, int(alpha_frac * 180)), (px, py), r)
        pygame.draw.circle(surf, (255, 140, 40, int(alpha_frac * 100)), (px, py), r + 20)
        self.screen.blit(surf, (0, 0))
        msg = self.font_lg.render("INTERCEPTED!", True, C_INTERCEPT)
        self.screen.blit(msg, (MAIN_W//2 - msg.get_width()//2, HEIGHT//2 - 60))

    # ── HUD (góc trái 3D view) ────────────────────────────────────────────────
    def _draw_hud(self):
        r    = self.target.pos - self.missile.pos
        dist = r.norm()
        v_rel = self.target.vel - self.missile.vel
        Vc   = -(r.dot(v_rel)) / (dist + 1e-9)

        mv = self.missile.vel
        tv = self.target.vel

        lines = [
            ("── SIMULATION ──────────",   C_TEXT_HEAD),
            (f"  Time (sim) : {self.elapsed_sim:7.2f} s", C_TEXT_VAL),
            (f"  Time scale : {self.time_scale:7.3f}x",  C_TEXT_VAL),
            (f"  {'[PAUSED]' if self.paused else '[RUNNING]'}",
             C_TEXT_WARN if self.paused else (80, 220, 80)),
            ("",                           C_TEXT_VAL),
            ("── GUIDANCE ────────────",   C_TEXT_HEAD),
            (f"  Nav gain N : {self.nav_gain:7.1f}",     C_TEXT_VAL),
            (f"  Max accel  : {self.max_accel:7.1f} m/s²",C_TEXT_VAL),
            (f"  Camera Rot : Y:{math.degrees(self.camera.yaw):.0f}° P:{math.degrees(self.camera.pitch):.0f}°", C_TEXT_VAL),
            ("",                           C_TEXT_VAL),
            ("── MISSILE ─────────────",   C_TEXT_HEAD),
            (f"  Speed      : {self.missile.speed:7.1f} m/s", C_TEXT_VAL),
            (f"  Pos X/Y/Z  : {self.missile.pos.x:+6.0f}/{self.missile.pos.y:+6.0f}/{self.missile.pos.z:+6.0f}", C_TEXT_VAL),
            (f"  Vel X/Y/Z  : {mv.x:+6.1f}/{mv.y:+6.1f}/{mv.z:+6.1f}", C_TEXT_VAL),
            (f"  |Vel|      : {mv.norm():7.1f} m/s", C_TEXT_VAL),
            (f"  Accel |a|  : {self.missile._accel.norm():7.1f} m/s²", C_TEXT_VAL),
            (f"  G-Load     : {self.missile.g_load:7.2f} G", (255, 100, 100) if self.missile.g_load > 12 else C_TEXT_VAL),
            ("",                           C_TEXT_VAL),
            ("── TARGET ──────────────",   C_TEXT_HEAD),
            (f"  Mode       : {self.target.mode.name[:11]:11s}", C_TEXT_VAL),
            (f"  Speed      : {self.target.speed:7.1f} m/s", C_TEXT_VAL),
            (f"  Pos X/Y/Z  : {self.target.pos.x:+6.0f}/{self.target.pos.y:+6.0f}/{self.target.pos.z:+6.0f}", C_TEXT_VAL),
            (f"  Vel X/Y/Z  : {tv.x:+6.1f}/{tv.y:+6.1f}/{tv.z:+6.1f}", C_TEXT_VAL),
            (f"  |Vel|      : {tv.norm():7.1f} m/s", C_TEXT_VAL),
            ("",                           C_TEXT_VAL),
            ("── ENGAGEMENT ──────────",   C_TEXT_HEAD),
            (f"  Distance   : {dist:7.1f} m",
             (255, 120, 80) if dist < 200 else C_TEXT_VAL),
            (f"  Closing Vc : {Vc:7.1f} m/s",
             C_TEXT_VAL if Vc > 0 else C_TEXT_WARN),
            (f"  Time to Go : {dist/(Vc + 1e-6):7.2f} s", C_TEXT_VAL),
        ]

        x, y = 12, 12
        for text, color in lines:
            surf = self.font_sm.render(text, True, color)
            self.screen.blit(surf, (x, y))
            y += 16

    def _draw_controls_hint(self):
        hints = [
            "SPACE:Pause  R:Reset  H:Help  ESC:Quit",
            "←→:Scrub 1frame  Shift+←→:30f  Ctrl+←→:10f",
            "↑↓:TimeScale  +/-:Zoom View  [/]:TargetSpd",
            "N/M:NavGain   1:Straight  2:Maneuvering",
            "A/D:Target Vx  W/S:Target Vy  Q/E:Target Vz",
        ]
        y = HEIGHT - 14 * len(hints) - 6 - 22  # nhường chỗ cho scrubber
        for h in hints:
            s = self.font_sm.render(h, True, (70, 90, 120))
            self.screen.blit(s, (10, y))
            y += 14

    def _draw_help_overlay(self):
        if not self.show_help:
            return
        overlay = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
        overlay.fill((0, 0, 0, 190))
        self.screen.blit(overlay, (0, 0))

        help_lines = [
            "═══════════  KEYBOARD HELP  ═══════════",
            "",
            "SPACE       Tạm dừng / Tiếp tục",
            "R           Reset mô phỏng",
            "H           Đóng / Mở help này",
            "ESC         Thoát",
            "",
            "↑ / ↓       Tăng / Giảm time scale (×2 / ÷2)",
            "← / →       Scrub history",
            "",
            "N / M       Tăng / Giảm hệ số N (±0.5)",
            "+ / -       Phóng to / Thu nhỏ",
            "",
            "── TARGET CONTROL (mục tiêu) ──────────",
            "[ / ]       Tăng / Giảm Speed tổng",
            "W / S       Tiến (+) / Lùi (-) [X]",
            "D / A       Phải (+) / Trái (-) [Y]",
            "E / Q       Lên  (+) / Xuống (-) [Z]",
            "",
            "1 / 2       Straight / Maneuvering",
            "",
            "── VIEW (góc nhìn) ────────────────────",
            "Chuột trái  Xoay Camera 3D",
            "═══════════════════════════════════════",
        ]

        x = WIDTH // 2 - 260
        y = HEIGHT // 2 - len(help_lines) * 11
        for line in help_lines:
            s = self.font_sm.render(line, True, (200, 220, 255))
            self.screen.blit(s, (x, y))
            y += 20

    # ── Scrubber bar ──────────────────────────────────────────────────────────
    def _draw_scrubber(self):
        """Thanh tiến trình / scrub ở đáy màn hình (full width)."""
        bar_h   = 20
        bar_y   = HEIGHT - bar_h
        bar_x   = 0
        bar_w   = MAIN_W   # chỉ nằm trong vùng 3D
        margin  = 4

        # Nền
        bg = pygame.Surface((bar_w, bar_h), pygame.SRCALPHA)
        bg.fill((*C_SCRUB_BG, 200))
        self.screen.blit(bg, (bar_x, bar_y))

        n = len(self._history)
        if n > 1:
            idx = self._hist_idx if self._hist_idx >= 0 else n - 1
            frac = idx / (n - 1)
            fill_w = int((bar_w - margin * 2) * frac)

            # Fill bar
            fill_color = C_SCRUB_REWD if self._scrubbing else C_SCRUB_FILL
            pygame.draw.rect(self.screen, fill_color,
                             (bar_x + margin, bar_y + margin//2,
                              fill_w, bar_h - margin))

            # Đầu đọc (playhead)
            head_x = bar_x + margin + fill_w
            pygame.draw.rect(self.screen, C_SCRUB_HEAD,
                             (head_x - 1, bar_y, 3, bar_h))

            # Nhãn thời gian — BUG FIX: clamp vị trí để text không tràn ra ngoài
            t = self._history[idx].elapsed_sim
            t_total = self._history[-1].elapsed_sim
            lbl = self.font_sm.render(
                f"{'◀◀ SCRUB' if self._scrubbing else '● LIVE'}  "
                f"{t:.2f}s / {t_total:.2f}s  [{idx+1}/{n}]  "
                f"{'← → navigate | → end = resume' if self._scrubbing else '← rewind'}",
                True, C_SCRUB_HEAD if self._scrubbing else C_SCRUB_FILL)
            lbl_x = min(bar_x + margin + fill_w + 6, bar_w - lbl.get_width() - 4)
            lbl_x = max(lbl_x, bar_x + margin)
            self.screen.blit(lbl, (lbl_x,
                                   bar_y + bar_h // 2 - lbl.get_height() // 2))

        # Viền
        pygame.draw.rect(self.screen, C_PANEL_BORDER,
                         (bar_x, bar_y, bar_w, bar_h), 1)

    # ── Divider giữa 3D view và panels ────────────────────────────────────────
    def _draw_divider(self):
        pygame.draw.line(self.screen, C_PANEL_BORDER,
                         (MAIN_W, 0), (MAIN_W, HEIGHT), 2)
        # Nhãn "3D VIEW"
        lbl = self.font_md.render("3D VIEW", True, (80, 110, 160))
        self.screen.blit(lbl, (MAIN_W - lbl.get_width() - 8, 4))

    def draw(self):
        self.screen.fill(BG_COLOR)

        # ── 3D View (bên trái) ──────────────────────────────────────────────
        self._draw_grid()
        self._draw_trail(self.target.trail,
                         C_TARGET_TRAIL_TAIL, C_TARGET_TRAIL_HEAD, width_max=3)
        self._draw_trail(self.missile.trail,
                         C_MISSILE_TRAIL_TAIL, C_MISSILE_TRAIL_HEAD, width_max=4)
        self._draw_los_line()
        self._draw_pip()

        arrow_scale = 250
        self._draw_arrow(self.target.pos, self.target.vel,
                         arrow_scale, C_VEL_ARROW, 2)
        if self.missile.active:
            self._draw_arrow(self.missile.pos, self.missile.vel,
                             arrow_scale, (255, 160, 80), 2)

        self._draw_object(self.target.pos, 9,
                          C_TARGET_BODY, (60, 160, 255), "TGT")
        if self.missile.active:
            self._draw_object(self.missile.pos, 6,
                              C_MISSILE_BODY, (255, 80, 40), "MSL")

        self._draw_intercept_flash()
        self._draw_hud()
        self._draw_controls_hint()
        self._draw_scrubber()

        # ── 3 Mặt phẳng chiếu (bên phải) — hiển thị toàn bộ quỹ đạo ───────
        for panel in self.panels:
            panel.draw(
                self.screen,
                self.missile.pos, self.missile.vel,
                self.missile.trail, self.missile.full_trail,
                self.target.pos,  self.target.vel,
                self.target.trail, self.target.full_trail,
            )

        # ── Divider căn giữa ────────────────────────────────────────────────
        self._draw_divider()

        # ── Overlay "SCRUBBING" banner ────────────────────────────────────────
        if self._scrubbing:
            ov = pygame.Surface((MAIN_W, 30), pygame.SRCALPHA)
            ov.fill((200, 140, 0, 160))
            self.screen.blit(ov, (0, 0))
            msg = self.font_md.render(
                "  ◀◀  SCRUBBING HISTORY  —  ← → to navigate  |  → at end to resume  ▶▶  ",
                True, (255, 240, 180))
            self.screen.blit(msg, (MAIN_W // 2 - msg.get_width() // 2, 6))

        self._draw_help_overlay()
        pygame.display.flip()

    # ── Main Loop ─────────────────────────────────────────────────────────────
    def run(self):
        running = True
        while running:
            dt_real = self.clock.tick(FPS) / 1000.0
            dt_real = min(dt_real, 0.05)
            running = self.handle_events()
            self.update(dt_real)
            self.draw()
        pygame.quit()
        sys.exit()


# ─────────────────────────────────────────────────────────────────────────────
# ENTRY POINT
# ─────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    sim = Simulation()
    sim.run()
