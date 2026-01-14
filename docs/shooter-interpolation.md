## Control Architecture

* Closed-loop **velocity control** using feedforward + PID blending
* Hood angle setpoints managed alongside RPM targets
* High enough P value that shooter gives full power when recovering

---

## Competition Failure & Fix

During competition, we discovered a critical issue in the open-source PID controller we were using:
**the integral term was not reset when the target setpoint changed**.

This caused integral windup and resulted in severe overshoot during autonomous shooting.

### Resolution

* Temporarily removed I term to continue with competition
* Patched the PID implementation locally to reset integral state on target changes
* Submitted a bug report upstream to the library maintainers

This fix significantly improved autonomous consistency.

---

# Shooter Interpolation Utilities

## Motivation

To score reliably from anywhere on the field—and while moving—we needed a way to map **robot pose → ideal shooter RPM and hood angle**.

Manual aiming or discrete presets were insufficient for:

* Full-field shooting
* Autonomous operation
* Shoot-on-the-fly under defense

---

## Design Constraints

* **Minimal driver cognitive load**
* **Support for autonomous aiming**
* **Continuous solution space**
* **Fast runtime evaluation**

---

## Explored Alternatives

* **Physics-based ballistic simulation**
  Accounted for drag and projectile motion, but:

    * Too complex
    * Poor real-world correlation
    * Returned *possible* shots, not *optimal* shots

* **Discrete preset shots**

    * Prevented shoot-on-the-fly
    * Vulnerable to defense
    * Incomplete field coverage

---

## Final Design

We adopted an **empirical interpolation-based approach**:

* Shooter test data collected experimentally
* Stored in a structured, versioned format
* Accessed through:

    * Enum-based configuration
    * A linear interpolation tree for runtime selection

This approach traded theoretical elegance for **predictable, tunable performance**, which proved far more valuable in competition.

---

## Non-Obvious Decision

The hardest—but most impactful—decision was **abandoning the ballistic simulation** after investing significant time in it.

Letting go of a technically impressive solution in favor of a simpler, more reliable one was critical to overall robot performance.

---

## Competition Adjustments

During events, mechanical changes caused previously ideal shots to drift.

### Mitigation Strategy

* Introduced temporary compensation terms
* Allowed fast between-match correction without regenerating the full dataset
* Preserved match readiness under time pressure
