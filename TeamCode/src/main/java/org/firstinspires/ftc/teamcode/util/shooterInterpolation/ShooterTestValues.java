package org.firstinspires.ftc.teamcode.util.shooterInterpolation;

public class ShooterTestValues {
    private final shooterTreeInterpolation shooterMap = new shooterTreeInterpolation();

    public void createShooterMap() {
        // Distance units must have a consistent unit
        shooterMap.put(20.0, new ShooterState(1600, 10));  // close
        shooterMap.put(60.0, new ShooterState(2700, 21));  // far
    }

    public ShooterState get(double distance) {
        return shooterMap.get(distance);
    }
}

