"""
SAM Simulator - Proportional Navigation Guidance
=================================================
A 2D missile intercept simulation using Pure Proportional Navigation (PN).
Implements realistic physics with Gaussian noise, smoke trails, and explosions.

Controls:
  T     - Spawn a random target
  SPACE / F - Launch an interceptor missile
  R     - Reset simulation
"""

import pygame
import math
import random
import sys
from dataclasses import dataclass, field
from typing import List, Optional

# ─────────────────────────── Constants ───────────────────────────
WIDTH, HEIGHT        = 1200, 800
FPS                  = 60
BACKGROUND           = (5, 8, 12)

# Launcher
LAUNCHER_X           = WIDTH // 2
LAUNCHER_Y           = HEIGHT - 60
LAUNCHER_COLOR       = (255, 165, 0)

# Target physics
TARGET_BASE_SPEED    = 3.5          # px/frame
TARGET_NOISE_VEL     = 0.12         # Gaussian σ for speed jitter
TARGET_NOISE_HEAD    = 0.8          # Gaussian σ for heading jitter (degrees)

# Missile physics
MISSILE_SPEED        = 10.0         # px/frame
MISSILE_NAV_CONST    = 4.5          # N – Proportional Navigation constant
MISSILE_MAX_TURN     = 6.0          # degrees/frame – G-force limit
MISSILE_TRAIL_LEN    = 90           # Number of smoke trail points

# Hit detection
HIT_RADIUS           = 14           # pixels

# Particle explosion
PARTICLE_COUNT       = 35
PARTICLE_LIFESPAN    = 45           # frames

# Colours
COL_MISSILE          = (255, 220, 60)
COL_TARGET           = (220, 80,  80)
COL_TARGET_BORDER    = (255, 140, 140)
COL_SMOKE            = (160, 160, 160)
COL_HUD              = (100, 255, 100)
COL_HUD_DIM          = (60,  180, 60)
COL_TITLE            = (60,  255, 60)
COL_SUBTITLE         = (60,  200, 60)
COL_EXPLOSION        = [(255,255,150),(255,200,80),(255,120,30),(200,60,20),(120,30,10)]

# ─────────────────────────── Helpers ─────────────────────────────

def angle_diff(a: float, b: float) -> float:
    """Shortest signed angular difference a→b (degrees)."""
    d = (b - a + 180) % 360 - 180
    return d


def heading_to_vec(deg: float) -> pygame.Vector2:
    """Convert heading in degrees (0=up, CW) to a unit Vector2."""
    rad = math.radians(deg - 90)        # pygame x-axis offset
    return pygame.Vector2(math.cos(rad), math.sin(rad))


def vec_to_heading(v: pygame.Vector2) -> float:
    """Convert Vector2 direction to heading degrees (0=up, CW)."""
    if v.length_squared() < 1e-9:
        return 0.0
    deg = math.degrees(math.atan2(v.y, v.x)) + 90
    return deg % 360


# ─────────────────────────── Terrain ─────────────────────────────

def generate_terrain(width: int, height: int, y_base: int) -> List[tuple]:
    """Generate a jagged mountain silhouette as a polygon."""
    pts = [(0, height)]
    x = 0
    while x <= width:
        y = y_base + random.randint(-40, 15)
        pts.append((x, y))
        x += random.randint(30, 80)
    pts.append((width, height))
    return pts


# ─────────────────────────── Particle ────────────────────────────

class Particle:
    """Single explosion particle with velocity, colour, and fade."""

    def __init__(self, pos: pygame.Vector2, palette: List[tuple]):
        self.pos   = pygame.Vector2(pos)
        angle      = random.uniform(0, 360)
        speed      = random.uniform(1.5, 6.0)
        self.vel   = heading_to_vec(angle) * speed
        self.life  = PARTICLE_LIFESPAN
        self.max_life = PARTICLE_LIFESPAN
        self.color = random.choice(palette)
        self.size  = random.randint(2, 5)

    def update(self):
        self.pos  += self.vel
        self.vel  *= 0.93          # drag
        self.life -= 1

    def draw(self, surf: pygame.Surface):
        alpha = int(255 * (self.life / self.max_life))
        r, g, b = self.color
        # fade to dark
        r = int(r * self.life / self.max_life)
        g = int(g * self.life / self.max_life)
        b = int(b * self.life / self.max_life)
        pygame.draw.circle(surf, (r, g, b), (int(self.pos.x), int(self.pos.y)), self.size)

    @property
    def dead(self) -> bool:
        return self.life <= 0


# ─────────────────────────── Target ──────────────────────────────

class Target:
    """
    A drone / projectile crossing the screen at variable heading.
    Gaussian noise is added each frame to heading and speed to simulate
    wind gusts and engine instability.
    """

    RADIUS = 8
    _id_counter = 0

    def __init__(self):
        Target._id_counter += 1
        self.id      = Target._id_counter
        self.label   = f"T_2_M_{self.id % 10 + 1}"
        self._spawn()

    def _spawn(self):
        side = random.choice(["top", "left", "right"])
        if side == "top":
            self.pos     = pygame.Vector2(random.randint(100, WIDTH - 100), -20)
            self.heading = random.uniform(100, 260)       # generally downward
        elif side == "left":
            self.pos     = pygame.Vector2(-20, random.randint(50, HEIGHT // 2))
            self.heading = random.uniform(-60, 60)        # rightward
        else:  # right
            self.pos     = pygame.Vector2(WIDTH + 20, random.randint(50, HEIGHT // 2))
            self.heading = random.uniform(120, 240)       # leftward

        self.speed   = TARGET_BASE_SPEED + random.uniform(-0.5, 1.5)
        self.vel     = heading_to_vec(self.heading) * self.speed
        self.alive   = True
        self.trail: List[pygame.Vector2] = []

    # ── update ──
    def update(self, dt: float):
        # Gaussian noise on speed and heading each frame
        noisy_speed   = self.speed + random.gauss(0, TARGET_NOISE_VEL)
        self.heading += random.gauss(0, TARGET_NOISE_HEAD)
        self.heading  = self.heading % 360
        self.vel      = heading_to_vec(self.heading) * noisy_speed

        self.trail.append(pygame.Vector2(self.pos))
        if len(self.trail) > 50:
            self.trail.pop(0)

        self.pos += self.vel

        # Kill if off-screen
        margin = 80
        if (self.pos.x < -margin or self.pos.x > WIDTH + margin or
                self.pos.y < -margin or self.pos.y > HEIGHT + margin):
            self.alive = False

    # ── draw ──
    def draw(self, surf: pygame.Surface, font_small):
        if not self.alive:
            return

        # Smoke trail
        for i, pt in enumerate(self.trail):
            a = int(80 * i / max(len(self.trail), 1))
            pygame.draw.circle(surf, (a, a, a), (int(pt.x), int(pt.y)), 2)

        # Body – filled circle with border
        ix, iy = int(self.pos.x), int(self.pos.y)
        pygame.draw.circle(surf, COL_TARGET, (ix, iy), self.RADIUS)
        pygame.draw.circle(surf, COL_TARGET_BORDER, (ix, iy), self.RADIUS, 2)

        # Direction tick
        tip = self.pos + heading_to_vec(self.heading) * (self.RADIUS + 6)
        pygame.draw.line(surf, (255, 80, 80), (ix, iy), (int(tip.x), int(tip.y)), 2)

        # HUD readout
        spd_display  = f"v: {self.speed:.2f} ({self.speed * 0.96:.2f})"
        head_display = f"h: {self.heading - 180:.2f}"
        lbl          = font_small.render(self.label,     True, COL_HUD)
        vl           = font_small.render(spd_display,    True, COL_HUD)
        hl           = font_small.render(head_display,   True, COL_HUD_DIM)
        ox, oy       = ix + 12, iy - 28
        surf.blit(lbl, (ox, oy))
        surf.blit(vl,  (ox, oy + 14))
        surf.blit(hl,  (ox, oy + 27))


# ─────────────────────────── Missile ─────────────────────────────

class Missile:
    """
    Surface-to-Air interceptor using Pure Proportional Navigation.

    PN Algorithm
    ────────────
    LOS angle  = atan2(Δy, Δx) from missile to target.
    LOS rate   = dLOS/dt  (change in LOS angle between frames).
    Commanded lateral acceleration ∝ N × closing_velocity × LOS_rate.

    In 2D discrete simulation we simplify to:
        desired_heading_change = N × LOS_rate
    clamped to max_turn_rate to honour G-force limits.
    """

    LENGTH = 14
    WIDTH  = 4

    def __init__(self, pos: pygame.Vector2, target: "Target"):
        self.pos          = pygame.Vector2(pos)
        self.target       = target
        self.speed        = MISSILE_SPEED
        # Initial heading: point straight at target
        delta             = target.pos - pos
        self.heading      = vec_to_heading(delta)
        self.vel          = heading_to_vec(self.heading) * self.speed
        self.alive        = True
        self.trail: List[pygame.Vector2] = []
        # Store previous LOS angle for rate calculation
        self._prev_los: Optional[float] = None

    # ── PN guidance ──
    def _proportional_navigation(self) -> float:
        """
        Returns the commanded heading correction (degrees) for this frame.

        LOS vector  r⃗ = target.pos − missile.pos
        LOS angle   λ = atan2(r⃗.y, r⃗.x)
        LOS rate    λ̇ = Δλ / Δt   (we use per-frame change)
        Guidance    ψ̇ = N · λ̇
        """
        r_vec    = self.target.pos - self.pos
        los_now  = math.degrees(math.atan2(r_vec.y, r_vec.x))

        if self._prev_los is None:
            self._prev_los = los_now
            return 0.0

        # Rate of change of LOS angle (degrees per frame)
        los_rate       = angle_diff(self._prev_los, los_now)
        self._prev_los = los_now

        # Commanded turn  =  N * LOS_rate
        commanded = MISSILE_NAV_CONST * los_rate
        return commanded

    # ── update ──
    def update(self, dt: float):
        if not self.target.alive:
            # If target died, keep flying straight (or could self-destruct)
            self.alive = False
            return

        # --- Guidance ---
        turn = self._proportional_navigation()
        # Clamp to G-force limit
        turn = max(-MISSILE_MAX_TURN, min(MISSILE_MAX_TURN, turn))
        self.heading = (self.heading + turn) % 360
        self.vel     = heading_to_vec(self.heading) * self.speed

        # --- Kinematics ---
        self.trail.append(pygame.Vector2(self.pos))
        if len(self.trail) > MISSILE_TRAIL_LEN:
            self.trail.pop(0)

        self.pos += self.vel

        # Off-screen → self-destruct
        if (self.pos.x < -50 or self.pos.x > WIDTH + 50 or
                self.pos.y < -50 or self.pos.y > HEIGHT + 50):
            self.alive = False

    # ── draw ──
    def draw(self, surf: pygame.Surface, font_small):
        if not self.alive:
            return

        # Smoke trail – gradient from grey to dark
        n = len(self.trail)
        for i, pt in enumerate(self.trail):
            frac   = i / max(n, 1)
            bright = int(30 + 130 * frac)
            alpha  = int(20 + 160 * frac)
            rad    = max(1, int(3 * (1 - frac) + 1))
            color  = (bright, bright, bright)
            pygame.draw.circle(surf, color, (int(pt.x), int(pt.y)), rad)

        # Draw missile as a rotated triangle (nose pointing along heading)
        angle_rad = math.radians(self.heading - 90)
        cos_a, sin_a = math.cos(angle_rad), math.sin(angle_rad)

        def rot(lx, ly):
            rx = lx * cos_a - ly * sin_a + self.pos.x
            ry = lx * sin_a + ly * cos_a + self.pos.y
            return (int(rx), int(ry))

        nose  = rot(0,  -self.LENGTH)
        left  = rot(-self.WIDTH,  self.LENGTH // 2)
        right = rot( self.WIDTH,  self.LENGTH // 2)
        tail  = rot(0,   self.LENGTH // 3)

        pygame.draw.polygon(surf, COL_MISSILE, [nose, left, tail, right])
        pygame.draw.polygon(surf, (255, 255, 200), [nose, left, tail, right], 1)

        # Engine glow dot
        eng   = rot(0, self.LENGTH // 2)
        pygame.draw.circle(surf, (255, 100, 0), eng, 3)

        # HUD
        spd_display  = f"v: {self.speed:.2f} ({self.speed * 0.96:.2f})"
        head_display = f"h: {(self.heading - 180):.2f}"
        vl = font_small.render(spd_display,  True, COL_HUD)
        hl = font_small.render(head_display, True, COL_HUD_DIM)
        ox, oy = int(self.pos.x) + 14, int(self.pos.y) - 20
        surf.blit(vl, (ox, oy))
        surf.blit(hl, (ox, oy + 13))


# ─────────────────────────── Launcher ────────────────────────────

def draw_launcher(surf: pygame.Surface, font):
    """Draw the launcher base and label."""
    x, y = LAUNCHER_X, LAUNCHER_Y
    # Base pad
    pygame.draw.rect(surf, (80, 80, 80), (x - 30, y + 10, 60, 14), border_radius=4)
    # Turret barrel (pointing upward)
    pygame.draw.rect(surf, LAUNCHER_COLOR, (x - 4, y - 18, 8, 28), border_radius=3)
    # Flame flicker on standby
    if random.random() < 0.3:
        pygame.draw.circle(surf, (255, 160, 30), (x, y - 18), 4)
    # Label
    lbl = font.render("LAUNCHER", True, (180, 180, 180))
    surf.blit(lbl, (x - lbl.get_width() // 2, y + 28))


# ─────────────────────────── Simulation ──────────────────────────

class Simulation:
    def __init__(self):
        pygame.init()
        self.screen  = pygame.display.set_mode((WIDTH, HEIGHT))
        pygame.display.set_caption("SAM Simulator ")
        self.clock   = pygame.time.Clock()

        # Fonts
        self.font_title  = pygame.font.SysFont("Courier New", 28, bold=True)
        self.font_sub    = pygame.font.SysFont("Courier New", 18, bold=True)
        self.font_hud    = pygame.font.SysFont("Courier New", 14)
        self.font_small  = pygame.font.SysFont("Courier New", 11)

        self.terrain = generate_terrain(WIDTH, HEIGHT, HEIGHT - 80)
        self.reset()

    def reset(self):
        self.targets:   List[Target]   = []
        self.missiles:  List[Missile]  = []
        self.particles: List[Particle] = []
        self.kills      = 0
        self.misses     = 0
        Target._id_counter = 0

    # ── spawn helpers ──
    def spawn_target(self):
        self.targets.append(Target())

    def launch_missile(self):
        if not self.targets:
            return
        # Target the nearest living target
        launch_pos = pygame.Vector2(LAUNCHER_X, LAUNCHER_Y)
        target = min(
            [t for t in self.targets if t.alive],
            key=lambda t: (t.pos - launch_pos).length(),
            default=None
        )
        if target:
            self.missiles.append(Missile(launch_pos, target))

    # ── collision ──
    def check_hits(self):
        for missile in self.missiles:
            if not missile.alive:
                continue
            for target in self.targets:
                if not target.alive:
                    continue
                dist = (missile.pos - target.pos).length()
                if dist < HIT_RADIUS:
                    # EXPLOSION
                    mid = (missile.pos + target.pos) / 2
                    for _ in range(PARTICLE_COUNT):
                        self.particles.append(Particle(mid, COL_EXPLOSION))
                    missile.alive = False
                    target.alive  = False
                    self.kills   += 1

    # ── draw HUD overlay ──
    def draw_hud(self):
        s = self.screen
        # Title
        title = self.font_title.render("SAM Simulator", True, COL_TITLE)
        s.blit(title, (WIDTH // 2 - title.get_width() // 2, 18))
        sub1 = self.font_sub.render("Designed and Developed", True, COL_SUBTITLE)
        sub2 = self.font_sub.render("Proportional Navigation Guidance Testing", True, COL_SUBTITLE)
        s.blit(sub1, (WIDTH // 2 - sub1.get_width() // 2, 54))
        s.blit(sub2, (WIDTH // 2 - sub2.get_width() // 2, 76))

        # Stats panel (bottom-left)
        active_t  = sum(1 for t in self.targets  if t.alive)
        active_m  = sum(1 for m in self.missiles if m.alive)
        lines = [
            f"Targets  : {active_t}",
            f"Missiles : {active_m}",
            f"Kills    : {self.kills}",
            f"Launcher X: {LAUNCHER_X}",
        ]
        for i, line in enumerate(lines):
            txt = self.font_hud.render(line, True, COL_HUD)
            s.blit(txt, (12, HEIGHT - 130 + i * 18))

        # Controls reminder (top-left)
        ctrl_lines = ["[T] Spawn Target", "[F/SPC] Launch", "[R] Reset"]
        for i, cl in enumerate(ctrl_lines):
            ct = self.font_small.render(cl, True, (80, 150, 80))
            s.blit(ct, (12, 12 + i * 14))

    # ── main loop ──
    def run(self):
        running = True
        while running:
            dt = self.clock.tick(FPS) / 1000.0   # seconds (frame-rate independent)

            # ── Events ──
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_t:
                        self.spawn_target()
                    if event.key in (pygame.K_SPACE, pygame.K_f):
                        self.launch_missile()
                    if event.key == pygame.K_r:
                        self.reset()
                    if event.key == pygame.K_ESCAPE:
                        running = False

            # ── Update ──
            for t in self.targets:
                t.update(dt)
            for m in self.missiles:
                m.update(dt)
            for p in self.particles:
                p.update()

            self.check_hits()

            # Prune dead objects
            self.targets   = [t for t in self.targets   if t.alive]
            self.missiles  = [m for m in self.missiles  if m.alive]
            self.particles = [p for p in self.particles if not p.dead]

            # ── Draw ──
            self.screen.fill(BACKGROUND)

            # Stars (static seed each frame for consistency)
            rng = random.Random(42)
            for _ in range(80):
                sx = rng.randint(0, WIDTH)
                sy = rng.randint(0, HEIGHT // 2)
                brightness = rng.randint(60, 200)
                pygame.draw.circle(self.screen, (brightness, brightness, brightness), (sx, sy), 1)

            # Terrain
            pygame.draw.polygon(self.screen, (40, 45, 45), self.terrain)
            pygame.draw.lines(self.screen, (70, 80, 80), False,
                              self.terrain[1:-1], 2)

            # Launcher
            draw_launcher(self.screen, self.font_small)

            # Targets & Missiles
            for t in self.targets:
                t.draw(self.screen, self.font_small)
            for m in self.missiles:
                m.draw(self.screen, self.font_small)

            # Particles
            for p in self.particles:
                p.draw(self.screen)

            # HUD
            self.draw_hud()

            pygame.display.flip()

        pygame.quit()
        sys.exit()


# ─────────────────────────── Entry Point ─────────────────────────

if __name__ == "__main__":
    sim = Simulation()
    # Pre-spawn a few targets to start the show
    for _ in range(3):
        sim.spawn_target()
    sim.run()