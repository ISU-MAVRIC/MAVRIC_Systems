# MAVRIC Rover Control Layout

## Keyboard Layout Visualization

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         MAVRIC ROVER CONTROLS                            │
└─────────────────────────────────────────────────────────────────────────┘

╔═══════════════════════════════════════════════════════════════════════╗
║                    MOVEMENT CONTROLS (WASD)                            ║
║                    (Exclusive with Rotation)                           ║
╚═══════════════════════════════════════════════════════════════════════╝

                              ┌───┐
                              │ W │  Forward
                              └───┘  +Shift = Fast (0.5)
                                     +Ctrl  = Slow (0.1)
                        ┌───┐ ┌───┐ ┌───┐
                        │ A │ │ S │ │ D │
                        └───┘ └───┘ └───┘
                        Left  Back  Right


╔═══════════════════════════════════════════════════════════════════════╗
║                    ROTATION CONTROLS (QE)                              ║
║                    (Exclusive with Movement)                           ║
╚═══════════════════════════════════════════════════════════════════════╝

                        ┌───┐         ┌───┐
                        │ Q │         │ E │
                        └───┘         └───┘
                     Rotate Left   Rotate Right


╔═══════════════════════════════════════════════════════════════════════╗
║                      ARM CONTROLS (Always Active)                      ║
╚═══════════════════════════════════════════════════════════════════════╝

    ┌─────────────────────────────────────────────────────────────┐
    │ SHOULDER                                                     │
    │  ┌───┐         ┌───┐         ┌───┐         ┌───┐          │
    │  │ Y │         │ H │         │ Z │         │ X │          │
    │  └───┘         └───┘         └───┘         └───┘          │
    │  Pitch Up   Pitch Down   Rotate Left  Rotate Right        │
    └─────────────────────────────────────────────────────────────┘

    ┌─────────────────────────────────────────────────────────────┐
    │ ELBOW                                                        │
    │  ┌───┐         ┌───┐                                       │
    │  │ U │         │ J │                                       │
    │  └───┘         └───┘                                       │
    │  Pitch Up   Pitch Down                                     │
    └─────────────────────────────────────────────────────────────┘

    ┌─────────────────────────────────────────────────────────────┐
    │ WRIST                                                        │
    │  ┌───┐         ┌───┐         ┌───┐         ┌───┐          │
    │  │ I │         │ K │         │ C │         │ V │          │
    │  └───┘         └───┘         └───┘         └───┘          │
    │  Pitch Up   Pitch Down   Rotate Left  Rotate Right        │
    └─────────────────────────────────────────────────────────────┘

    ┌─────────────────────────────────────────────────────────────┐
    │ CLAW                                                         │
    │  ┌───┐         ┌───┐                                       │
    │  │ [ │         │ ] │                                       │
    │  └───┘         └───┘                                       │
    │  Open         Close                                        │
    └─────────────────────────────────────────────────────────────┘


╔═══════════════════════════════════════════════════════════════════════╗
║                         EMERGENCY CONTROLS                             ║
╚═══════════════════════════════════════════════════════════════════════╝

                              ┌─────┐
                              │ ESC │  Emergency Stop
                              └─────┘  (Stops all motors)


═══════════════════════════════════════════════════════════════════════

                           CONTROL RULES

1. Movement (WASD) and Rotation (QE) are MUTUALLY EXCLUSIVE
   - Pressing Q or E activates rotation mode
   - Pressing WASD (without Q/E) activates movement mode
   
2. ARM CONTROLS are ALWAYS AVAILABLE
   - Can use arm controls while in movement mode
   - Can use arm controls while in rotation mode
   - Multiple arm joints can be controlled simultaneously

3. MULTIPLE KEY PRESSES are supported
   - W + A = Forward while turning left
   - W + Shift = Fast forward
   - Y + U + I = Move shoulder, elbow, and wrist together
   
4. SAFETY FEATURES
   - ESC stops everything immediately
   - Window blur (lose focus) releases all keys
   - Client disconnect stops all motors

═══════════════════════════════════════════════════════════════════════


                        EXAMPLE COMBINATIONS

   Forward Fast + Right:          W + Shift + D
   Backward Slow + Left:          S + A
   Rotate Right:                  E
   Shoulder Up + Elbow Up:        Y + U
   Movement + Arm:                W + A + Y + U
   Emergency Stop:                ESC


═══════════════════════════════════════════════════════════════════════

                         SPEED REFERENCE

Movement Speeds:
  ├─ Fast (W + Shift):      0.5
  ├─ Normal (W):            0.3
  ├─ Slow (W + Ctrl):       0.1
  ├─ Backward (S):          -0.1
  └─ Turning Reduction:     Normal / 1.5 = 0.2

Rotation Speed:
  └─ In-place rotation:     0.2

Arm Speeds:
  ├─ Shoulder Pitch:        0.4
  ├─ Shoulder Rotation:     0.4
  ├─ Elbow Pitch:           0.5
  ├─ Wrist Pitch:           0.5
  ├─ Wrist Rotation:        0.5
  └─ Claw:                  1.0

═══════════════════════════════════════════════════════════════════════
```

## State Machine Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                         CONTROL STATE                            │
└─────────────────────────────────────────────────────────────────┘

                    ┌────────────────┐
                    │  IDLE STATE    │
                    │  (No keys)     │
                    └───────┬────────┘
                            │
            ┌───────────────┼───────────────┐
            │               │               │
        Q or E          W,A,S,D         Arm Keys
            │               │               │
            ▼               ▼               ▼
    ┌──────────────┐  ┌──────────────┐  ┌──────────────┐
    │  ROTATION    │  │  MOVEMENT    │  │  ARM ACTIVE  │
    │    MODE      │  │    MODE      │  │  (Always OK) │
    └──────────────┘  └──────────────┘  └──────────────┘
            │               │               │
            └───────────────┼───────────────┘
                            │
                         ESC or
                      Window Blur
                            │
                            ▼
                    ┌────────────────┐
                    │ EMERGENCY STOP │
                    │  (All motors   │
                    │   stopped)     │
                    └────────────────┘
```

## Motor Control Flow

```
Keyboard Event → WebSocket → Server → Control Logic → CAN Bus → Motors

Browser JS          Socket.IO    Flask      Python      SparkCAN    Hardware
──────────         ──────────   ─────      ──────      ────────    ────────
  keydown    ───▶   key_down  ───▶  Update ───▶  Set  ───▶  Send  ───▶  Move
                                     State      Speed     CAN msg    Motor

  keyup      ───▶   key_up    ───▶  Update ───▶  Set  ───▶  Send  ───▶  Stop
                                     State      Speed     CAN msg    Motor
```
