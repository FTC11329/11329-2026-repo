package org.firstinspires.ftc.teamcode.util.shooterInterpolation;

public class ShooterTestValues {
    private final shooterTreeInterpolation shooterMap = new shooterTreeInterpolation();

    public void createShooterMap() {
        // Distance units must have a consistent unit
        shooterMap.put(60, new ShooterState(2900, 14.1));  // close
        shooterMap.put(62, new ShooterState(2900, 14.6));  // far
    }

    public ShooterState get(double distance) {
        return shooterMap.get(distance);
    }
}

