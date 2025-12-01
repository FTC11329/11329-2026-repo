package org.firstinspires.ftc.teamcode.util.shooterInterpolation;

import java.util.Map;
import java.util.TreeMap;

public class shooterTreeInterpolation {

    private final TreeMap<Double, ShooterState> map = new TreeMap<>();
    //sorted map of distance from target and optimal shooter state

    public void put(double distance, ShooterState state) {
        map.put(distance, state);
    } //fills the map with the example points

    public ShooterState get(double distance) {
        // If exact entry exists, return it
        if (map.containsKey(distance)) {
            return map.get(distance);
        }

        // Get nearest lower and higher points
        Map.Entry<Double, ShooterState> lower = map.floorEntry(distance);
        Map.Entry<Double, ShooterState> upper = map.ceilingEntry(distance);

        // If out of range, use the closest endpoint
        if (lower == null) return upper.getValue();
        if (upper == null) return lower.getValue(); //todo: find a better way of dealing with edge cases

        double lowerDistance = lower.getKey();
        double upperDistance = upper.getKey();
        ShooterState lowerShooterState = lower.getValue();
        ShooterState upperShooterState = upper.getValue();

        // Linear interpolation weight
        double t = (distance - lowerDistance) / (upperDistance - lowerDistance);

        // Interpolate RPM and hood angle
        double rpm = lowerShooterState.rpm + (upperShooterState.rpm - lowerShooterState.rpm) * t;
        double hood = lowerShooterState.hoodAngle + (upperShooterState.hoodAngle - lowerShooterState.hoodAngle) * t;

        return new ShooterState(rpm, hood);
    }
}

