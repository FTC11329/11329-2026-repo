package org.firstinspires.ftc.teamcode.util.shooterInterpolation;

public class ShooterState {
    public final double rpm;
    public final double hoodAngle;
    public final double timeInFlight;

    ShooterState(double rpm, double hoodAngle, double timeInFlight) {
        this.rpm = rpm;
        this.hoodAngle = hoodAngle;
        this.timeInFlight = timeInFlight;
    }
}
