# Drone Simulator Setup

## Fastest path (recommended)

1. In Unity menu, click `Tools > Drone Simulator > Create Demo Drone Rig`.
2. Press Play.
3. Use controls:
   - `W / S`: thrust up / down
   - `A / D`: yaw left / right
   - `I / K`: pitch forward / backward
   - `J / L`: roll left / right
   - `M`: toggle Manual / Auto mode
4. In Auto mode, left click in Game/Scene view to set world target.

This creates a full test scene:

- Ready-to-fly drone with 4 rotor transforms
- Checker/grid testing ground
- Waypoint route with poles
- Gates to fly through
- Obstacle pylons
- Start/finish pads
- Auto target marker and follow camera

## Build only the scenario

If you only want the environment (without recreating the drone), use:

`Tools > Drone Simulator > Create Testing Course Only`

Generated materials/textures are saved in:

`Assets/Art/Generated/DroneTesting`

## Manual setup (for your own mesh)

1. Create or select your drone root GameObject.
2. Add the `QuadcopterController` script to the same root.
3. Assign 4 rotor transforms in `Rotor Setup`:
   - Use the real rotor mesh transform positions.
   - Set `yawSpinDirection` alternating by rotor (example: `+1, -1, +1, -1`).

## Recommended custom dynamics values

- `Mass`: 1.0 to 2.5
- `Inertia Tensor Body`: small positive values (example `0.02, 0.04, 0.02`)
- `Linear Drag`: 0.05 to 0.3
- `Angular Drag`: 0.05 to 0.4

## Auto mode notes

- Set `Auto Target Position` in inspector, or assign `Auto Target Transform`.
- Optional: left mouse click sets target point (`Allow Mouse Click Target`).

## Tuning tips

- If the drone is too weak: increase `Max Rotor Thrust`.
- If it oscillates: lower `Attitude Kp` / `Attitude Ki`, or raise `Attitude Kd` / `Angular Drag`.
- If auto mode is too slow: increase `Horizontal Position Kp` or `Auto Max Horizontal Acceleration`.
- If yaw is too weak/strong: tune `Yaw Drag Torque Coefficient` and `Max Yaw Torque`.
