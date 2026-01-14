# Shooter Subsystem

The Shooter subsystem is a typical example of a subsystem on our robot that controls the low level operation of one piece of the robots capabilities

## Design goals

1. Reach target shooting RPM as fast as possible after each shot
2. Accurately set hood angle for variable-distance shots
3. Reliably detect when a ball has been fired so it can be removed from the robot state machine

---

## Design Constraints

The most important constraints shaping this subsystem were:

* **Zero tolerance for false negatives** in shot detection
  Missing a shot event breaks the robot’s state machine and causes cascading failures.
* **Very fast RPM recovery**
  The shooter must regain target velocity immediately after firing to enable multi-ball bursts.
* **Tuning speed and reliability during events**
  Any solution had to be understandable and adjustable in the pits without rewriting code.

---

## Shot Detection Strategy

Several approaches were evaluated and rejected:

* **RPM error alone**
  Too sensitive to target changes; indistinguishable from commanded RPM transitions.
* **Feeder motor voltage spikes**
  Not observable due to the feeder’s high torque.
* **Energy-based modeling (flywheel + motor input)**
  Too complex and fragile for competition use.
* **Rolling window of RPM errors**
  Worked, but added unnecessary state and complexity.
* **Beam break sensor**
  Increased wiring complexity and failure risk on a moving turret.

### Final Approach: First-Derivative Error Test

The final solution uses the **first derivative of shooter RPM error**:

* Detects a shot by observing a sudden *maximum error spike*
* Locks out further detection until RPM returns within a defined threshold
* Eliminates false negatives while remaining robust to target changes

This approach provided the best balance of reliability, simplicity, and tunability.
