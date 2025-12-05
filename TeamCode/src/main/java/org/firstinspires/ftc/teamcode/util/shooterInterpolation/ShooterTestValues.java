package org.firstinspires.ftc.teamcode.util.shooterInterpolation;

public class ShooterTestValues {
    private final ShooterTreeInterpolation shooterMap = new ShooterTreeInterpolation();

    public ShooterTestValues() {
        // Distance units must have a consistent unit
        shooterMap.put(17.33, new ShooterState(1868, 16.486, 0.54));
        shooterMap.put(21.98, new ShooterState(2020, 19.7, 0.7));
        shooterMap.put(24.4, new ShooterState(2000, 24.2, 0.5));
        shooterMap.put(29.38, new ShooterState(2016, 25.7, 0.5));
        shooterMap.put(34.67, new ShooterState(2132, 28.7, 0.55));
        shooterMap.put(40.96, new ShooterState(2216, 41.3, 0.4));
        shooterMap.put(46.1, new ShooterState(2196, 40.7, 0.52));
        shooterMap.put(49.2, new ShooterState(2264, 40.7, 0.57));
        shooterMap.put(55.1, new ShooterState(2332, 42.2, 0.64));
        shooterMap.put(60.7, new ShooterState(2484, 43.7, 0.61));
        shooterMap.put(65.6, new ShooterState(2480, 45, 0.53));
        shooterMap.put(71.6, new ShooterState(2336, 35, 0.44));
        shooterMap.put(75.8, new ShooterState(2420, 35, 0.47));
        shooterMap.put(82.7, new ShooterState(2500, 32.3, 0.58));
        shooterMap.put(90.5, new ShooterState(2620, 35.3, 0.62));
        shooterMap.put(95.2, new ShooterState(3076, 40.1, 0)); //0
        shooterMap.put(99.8, new ShooterState(3112, 38, 0)); //0
        shooterMap.put(108.75, new ShooterState(3236, 38.6, 0)); //0
        shooterMap.put(113.16, new ShooterState(3236, 38.6, 0)); //0
        shooterMap.put(119.01, new ShooterState(3400, 41.6, 0.75));
        shooterMap.put(125.89, new ShooterState(3396, 42.2, 0.72));
        shooterMap.put(135.2, new ShooterState(3484, 42.2, 0.78));
        shooterMap.put(140.6, new ShooterState(3520, 44.6, 0.72));
        shooterMap.put(151.5, new ShooterState(3608, 43.1, 0.78));
    }

    public ShooterState get(double distance) {
        return shooterMap.get(distance);
    }
}

