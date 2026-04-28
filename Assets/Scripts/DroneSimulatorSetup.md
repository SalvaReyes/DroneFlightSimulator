# Drone Simulator Setup

## Fastest path

1. In Unity menu, click `Tools > Drone Simulator > Create Demo Drone Rig`.
2. Press Play.
3. Use controls:
   - `W / S`: climb / descend around hover thrust
   - `A / D`: yaw left / right
   - `I / K`: pitch forward / backward
   - `J / L`: roll left / right

This creates a manual-only test scene:

- Ready-to-fly drone with 4 rotor transforms
- Checker/grid testing ground
- Waypoint route with poles
- Gates to fly through
- Obstacle pylons
- Start/finish pads
- Cinemachine follow camera

## Build only the scenario

If you only want the environment without recreating the drone, use:

`Tools > Drone Simulator > Create Testing Course Only`

Generated materials/textures are saved in:

`Assets/Art/Generated/DroneTesting`

## Manual setup for your own mesh

1. Create or select your drone root GameObject.
2. Add the `QuadcopterController` script to the same root.
3. Assign 4 rotor transforms in `Rotor Setup`:
   - Use the real rotor mesh transform positions.
   - Set `yawSpinDirection` alternating by rotor, for example `+1, -1, +1, -1`.

## Recommended dynamics values

- `Mass`: 1.0 to 2.5
- `Inertia Tensor Body`: small positive values, for example `0.02, 0.04, 0.02`
- `Linear Drag`: 0.05 to 0.3
- `Angular Drag`: 0.05 to 0.4
- `Max Rotor Thrust`: high enough that 4 rotors exceed hover thrust by several times

## Tuning tips

- If the drone cannot lift: increase `Max Rotor Thrust`.
- If pitch/roll feels inverted: check each rotor transform position and `yawSpinDirection`.
- If it oscillates: lower `Attitude Angle Gain` or `Angular Rate Gain`, or raise `Angular Drag`.
- If it feels sluggish: increase `Attitude Angle Gain` or `Angular Rate Gain`.
- If yaw is too weak/strong: tune `Yaw Drag Torque Coefficient` and `Max Yaw Torque`.
