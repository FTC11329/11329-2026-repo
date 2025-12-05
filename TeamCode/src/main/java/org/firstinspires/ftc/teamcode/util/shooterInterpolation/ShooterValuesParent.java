package org.firstinspires.ftc.teamcode.util.shooterInterpolation;

public interface ShooterValuesParent {

    ShooterTreeInterpolation shooterMap = new ShooterTreeInterpolation();

    // linearly interpolates between the last and next distance
    // to get the shooter state at that distance
    // Shooter state includes RPM, angle, and Time of Flight
    ShooterState get(double distance);
}
