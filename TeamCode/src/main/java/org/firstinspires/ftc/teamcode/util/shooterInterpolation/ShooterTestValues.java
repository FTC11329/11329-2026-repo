package org.firstinspires.ftc.teamcode.util.shooterInterpolation;

import com.bylazar.configurables.annotations.Configurable;

import java.util.List;

@Configurable
public class ShooterTestValues implements ShooterValuesParent {
    private final ShooterTreeInterpolation shooterMap = new ShooterTreeInterpolation();
    public ShooterTestValues() {
        // Distance units must have a consistent unit
        shooterMap.put(26, new ShooterState(2100, 16, 0.76));
        shooterMap.put(36, new ShooterState(2050, 21, 0.594));
        shooterMap.put(46, new ShooterState(2120, 24, 0.646));
        shooterMap.put(56, new ShooterState(2200, 30.5, 0.635));
        shooterMap.put(66, new ShooterState(2270, 34, 0.656));
        shooterMap.put(80.5, new ShooterState(2400,36,0.626));
        shooterMap.put(90.5, new ShooterState(2550,37,0.76));
        shooterMap.put(100.5, new ShooterState(2600,37,0.706));
        shooterMap.put(110.5, new ShooterState(2700,38,0.732));
        shooterMap.put(121.5, new ShooterState(2870,38, 0.834));
        shooterMap.put(131.5, new ShooterState(3030,39.3,0.922));
        shooterMap.put(141.6, new ShooterState(3100,36.5,0.916));
        shooterMap.put(151.5, new ShooterState(3240,38,0.924));
        shooterMap.put(161.5, new ShooterState(3400,38,0.928));
        shooterMap.put(171.5, new ShooterState(3530,40.5, 0.980));
        shooterMap.put(181.5, new ShooterState(3610,40.5, 0.993));
        shooterMap.put(191.5, new ShooterState(3710,41.5, 1.098));
        shooterMap.put(201.5, new ShooterState(3810,41.5, 1.020));
    }

    public ShooterState get(double distance) {
        return shooterMap.get(distance);
    }

    public List<Double> getDistances(){
        return shooterMap.getKeys();
    }
}

