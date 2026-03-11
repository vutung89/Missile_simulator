# 🚀 SAM Simulator — Proportional Navigation Guidance

> A 2D Surface-to-Air Missile intercept simulation built with Python & Pygame,  
> implementing the **Pure Proportional Navigation (PN)** guidance algorithm.

---

## 📸 Preview

```
[T] Spawn Target    SAM Simulator
[F/SPC] Launch      Designed and Developed by Time Lapse Coder
[R] Reset           Proportional Navigation Guidance Testing

          ·  ·  ·  ·  (target trail)
                ◉  ← Target  v: 3.50  h: -12.34
               /
              △  ← Missile   v: 10.00  h: 45.67
             /
      [LAUNCHER]
  ████████████████  (terrain)
```

---

## 🛠️ Requirements

| Package  | Version   |
|----------|-----------|
| Python   | ≥ 3.8     |
| pygame   | ≥ 2.0     |

Install dependencies:

```bash
pip install pygame
```

---

## ▶️ How to Run

```bash
python sam_simulator.py
```

3 targets are pre-spawned automatically when the simulation starts.

---

## 🎮 Controls

| Key            | Action                              |
|----------------|-------------------------------------|
| `T`            | Spawn a new random target           |
| `F` / `Space`  | Launch a missile at the nearest target |
| `R`            | Reset the entire simulation         |
| `Esc`          | Quit                                |

---

## 🧠 How the Guidance Works

### Pure Proportional Navigation (PN)

The missile steers by tracking the **Line of Sight (LOS)** angle to the target.

```
r⃗  = target.pos − missile.pos          # LOS vector
λ   = atan2(r⃗.y, r⃗.x)                  # LOS angle (degrees)
λ̇   = Δλ / Δt                           # LOS rate (change per frame)
ψ̇   = N · λ̇                             # Commanded heading change
```

- **N** (Navigation Constant) = `4.5` — higher values make the missile more aggressive.
- The commanded turn is **clamped** to `±6°/frame` to simulate real-world G-force limits.
- A zero LOS rate means the missile is on a **collision course** — no correction needed.

---

## 🏗️ Code Structure

```
sam_simulator.py
│
├── class Target      — Drone/projectile with Gaussian noise on speed & heading
├── class Missile     — PN-guided interceptor with smoke trail
├── class Particle    — Single explosion fragment (velocity + fade)
├── class Simulation  — Main loop, event handling, collision detection
│
├── generate_terrain()   — Procedural jagged mountain silhouette
├── draw_launcher()      — Draws the launcher base at screen bottom-center
├── heading_to_vec()     — Converts heading degrees → pygame.Vector2
└── vec_to_heading()     — Converts pygame.Vector2 → heading degrees
```

---

## ⚙️ Tunable Parameters

Open `sam_simulator.py` and adjust the **Constants** section at the top:

| Variable              | Default | Description                                  |
|-----------------------|---------|----------------------------------------------|
| `TARGET_BASE_SPEED`   | `3.5`   | Target cruising speed (px/frame)             |
| `TARGET_NOISE_VEL`    | `0.12`  | Gaussian σ for speed jitter (wind gusts)     |
| `TARGET_NOISE_HEAD`   | `0.8`   | Gaussian σ for heading jitter (degrees)      |
| `MISSILE_SPEED`       | `10.0`  | Missile speed (px/frame)                     |
| `MISSILE_NAV_CONST`   | `4.5`   | N — PN navigation constant (typically 3–5)   |
| `MISSILE_MAX_TURN`    | `6.0`   | Max turn rate per frame (G-force limit)      |
| `MISSILE_TRAIL_LEN`   | `90`    | Smoke trail length (frames retained)         |
| `HIT_RADIUS`          | `14`    | Hit detection radius (pixels)                |
| `PARTICLE_COUNT`      | `35`    | Explosion particle count                     |

---

## ✨ Features

- ✅ **Proportional Navigation** — mathematically correct LOS rate guidance
- ✅ **G-force limiting** — max turn rate clamp per frame
- ✅ **Gaussian noise** — realistic target flight instability
- ✅ **Smoke trails** — gradient-fading persistent trails per missile
- ✅ **Particle explosions** — colour-fade burst on successful intercept
- ✅ **Procedural terrain** — randomised mountain silhouette each run
- ✅ **Live HUD** — velocity and heading readout for every entity
- ✅ **Frame-rate independent** — `dt`-based physics via `pygame.Vector2`
- ✅ **OOP architecture** — clean class hierarchy, easy to extend

---

## 📐 Physics Notes

- All movement uses `pygame.Vector2` for clean 2D vector math.
- Heading convention: **0° = up, clockwise positive** (matches radar/aviation standard).
- The simulation runs at **60 FPS** with frame-rate independent `dt` scaling.
- Targets are removed when they exit the screen boundary (+80 px margin).
- Missiles self-destruct if they leave the screen (+50 px margin).

---

## 📄 License

MIT — free to use, modify, and distribute.

---

*Inspired by real SAM guidance systems. Built for educational purposes.*