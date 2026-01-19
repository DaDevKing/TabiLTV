# Tabi Robot LTV Drive

This is the **LTV (Linear Time-Varying) Drive system** for the Tabi robot. It is built on top of the `TabiSubsystem` and lets the robot drive to a point on the field automatically.

---

## How It Works

- The robot keeps track of its **current position** using **encoders and gyro**.
- You give it a **target point** (X, Y, and heading), and it computes:
  - How far to drive forward (`vx`)
  - How much to turn (`omega`)
- The robot drives toward the target smoothly and stops when it gets close enough.

**Key Classes:**
- `TabiSubsystem` → handles motors, gyro, odometry, and arcade drive.
- `LTVDrive` → calculates speeds to reach a target point.
- `LTVDriveToPointCommand` → a command that uses LTVDrive and stops the robot at the target.

---

## Why LTV is Better than Simple Differential Drive

- **Smarter movement**: The robot moves toward a target instead of just driving for a set time or distance.
- **Adaptive turning**: It turns to face the target while moving.
- **Works for autonomous**: You can use it in autos to drive precise paths.
- **Stops accurately**: It knows when it reaches the point and stops, unlike driving by time which may overshoot.

---

## How to Use

1. Create a new LTV command with a target:
```java
new LTVDriveToPointCommand(new Pose2d(2.0, 0.0, new Rotation2d()), m_tabiSubsystem);
