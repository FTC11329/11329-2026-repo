  # FTC code for 11329 ICE robotics competition season (2025 - 2026)

## Context

This codebase was developed for FTC competition robots operating under strict real-time,
hardware, and match-time constraints. The system prioritizes reliability, fast iteration during events, 
and predictable behavior over theoretical optimality.

## Codebase Structure

The robot software is organized around a centralized update loop that coordinates multiple stateful subsystems:

- `subsystems/`  
  Hardware-facing components responsible for low-level control and sensor interaction (shooter, indexer, intake, drive, etc.)

- `Robot.java`  
  Central coordination layer. All subsystems are updated here, and higher-level behaviors such as shoot-on-the-fly, intake unjamming, and telemetry are orchestrated from this file.

- `teleops/`  
  Operator control logic. Translates driver inputs into robot state transitions and commands.

- `autos/`  
  Autonomous routines and sequences executed during the autonomous period.

- `util/`  
  Shared utilities including control abstractions, interpolation logic, and helper classes.

- `Constants.java`  
  Centralized configuration and tuning parameters.

## Execution Flow

At runtime, the robot operates as follows:

1. Operator or autonomous logic (in `teleops/` or `autos/`) determines the desired robot behavior.
2. High-level commands and state flags are passed to `Robot.java`.
3. `Robot.java` coordinates subsystem updates each loop cycle, handling:
    - State transitions
    - Fault recovery (e.g., intake unjam)
    - Combined behaviors (e.g., shoot-on-the-fly)
4. Subsystems execute low-level control logic using shared utilities from `util/`.

## Highlighted Subsystems

The following components represent my primary contributions and are the best entry points for review:

### Shooter Subsystem (`subsystems/shooter/`)

Implements closed-loop shooter velocity control using PID feedback combined with feedforward modeling.

Key aspects:
- Controls the 2 servos and motor that comprise the robots shooter
- Velocity control with feedforward + PID blending
- Detects if balls have been shot based on the 1st derivative test

### Shooter Interpolation Utilities (`util/shooterInterpolation/`)

Encapsulates shooter tuning data and interpolation logic.

Key aspects:
- Structured representation of test data
- Enum-based configuration for clarity
- Linear interpolation tree for runtime setpoint selection

### Robot Coordination Layer (`Robot.java`)

Acts as the central integration point for all subsystems. Higher-level robot behaviors are composed here rather than inside individual subsystems.

Key responsibilities:
- Coordinating subsystem updates
- Implementing compound behaviors such as shoot-on-the-fly and intake unjamming
- Publishing debugging telemetry during matches
- Computing shared feedforward terms used by multiple subsystems

## Design Decisions & Tradeoffs

- Chose state-machine-based coordination to improve debuggability during competitions
- Prioritized conservative control tuning over aggressive response to avoid instability under load
- Reduced the frequency of certain sensor calls, improving loop times, but reducing reaction time

## Future Improvements

- Add integration testing for new state logic using simulated inputs
- Improve separation between hardware abstraction and control logic
- Unify configuration and tuning parameters across subsystems
