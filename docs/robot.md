# Robot Coordination Layer (`Robot.java`)

## Motivation

In previous seasons, each op mode:

* Initialized every subsystem independently
* Reimplemented multi-subsystem logic
* Made testing slow and repetitive

There was no shared place to store **robot-level behavior**, making iteration costly and error-prone.

---

## Design Goals

* One centralized update loop
* Minimal but powerful abstraction
* Easy creation of temporary test op modes
* All multi-subsystem behavior defined in one place

---

## Architecture Overview

`Robot.java` acts as the **integration layer**:

* Owns subsystem initialization and updates
* Receives high-level intent from `teleops/` and `autos/`
* Implements behaviors that span multiple subsystems:

    * Shoot-on-the-fly
    * Intake unjamming
    * Shared feedforward calculations
    * Telemetry publishing

This structure allows op modes to remain thin and declarative.

---

## Key Iterations & Tradeoffs

### Shoot-On-The-Fly (SOTF)

Progression of approaches:

1. Virtual robot pose → virtual goal
2. Considered acceleration compensation (rejected due to inconsistency)
3. Started without iterations
4. Considered closed-form solutions

Each iteration balanced accuracy against predictability under real-world noise.

### Additional Tradeoffs

* Shoot zone detection vs manual trigger
* Shooter idle at 0.3 power vs full stop
* Intake spin-up delays to prevent rapid state oscillation

---

## Competition Failure & Fix

Early SOTF implementations failed to account for high acceleration and rapid direction changes, leading to missed shots at speed.

### Resolution

* Expanded kinematic considerations
* Added guards against unstable conditions
* Tuned conservatively to prioritize hit consistency over theoretical accuracy

---

## Why This Matters

This file is the **spine of the codebase**.
It enables fast iteration, cleaner testing, and reliable integration under competition pressure.
