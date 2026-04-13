package org.firstinspires.ftc.teamcode.util.shooterInterpolation;

import com.bylazar.configurables.annotations.Configurable;

import java.util.List;

@Configurable
public class ShooterTestValues implements ShooterValuesParent {
    private final ShooterTreeInterpolation shooterMap = new ShooterTreeInterpolation();

    private final double CLOSE_ADDITION = 0; // - shooter farther
    private final double ADDITION = 0; // - shooter farther
    private final double FAR_ADDITION = 3; // - shooter farther
    private final double RPM_MULTIPLYER = 1; // + shooter farther
    private final double FAR_ANGLE_ADDITION = 0; // + shooter farther
    public ShooterTestValues() {
        // Distance units must have a consistent unit
        shooterMap.put(25 + ADDITION + CLOSE_ADDITION, new ShooterState(2100 * RPM_MULTIPLYER, 16, .736));
        shooterMap.put(35 + ADDITION + CLOSE_ADDITION, new ShooterState(2050 * RPM_MULTIPLYER, 21, .524));
        shooterMap.put(45 + ADDITION + CLOSE_ADDITION, new ShooterState(2120 * RPM_MULTIPLYER, 24, .576));
        shooterMap.put(55. + ADDITION + CLOSE_ADDITION, new ShooterState(2200 * RPM_MULTIPLYER, 30.5, .588));
        shooterMap.put(65. + ADDITION + CLOSE_ADDITION, new ShooterState(2270 * RPM_MULTIPLYER, 34, .628));
        shooterMap.put(75. + ADDITION + CLOSE_ADDITION, new ShooterState(2400 * RPM_MULTIPLYER,36,.644));
        shooterMap.put(85. + ADDITION, new ShooterState(2550 * RPM_MULTIPLYER,37,.666));
        shooterMap.put(95. + ADDITION, new ShooterState(2600 * RPM_MULTIPLYER,37,.742));
        shooterMap.put(105 + ADDITION, new ShooterState(2700 * RPM_MULTIPLYER,38,.662));
        shooterMap.put(115 + FAR_ADDITION, new ShooterState(2870 * RPM_MULTIPLYER,38+ FAR_ANGLE_ADDITION,0.736));
        shooterMap.put(125. + FAR_ADDITION, new ShooterState(3030 * RPM_MULTIPLYER,39.3+ FAR_ANGLE_ADDITION,.810));
        shooterMap.put(135. + FAR_ADDITION, new ShooterState(3100 * RPM_MULTIPLYER,36.5+ FAR_ANGLE_ADDITION,.934));
        shooterMap.put(145. + FAR_ADDITION, new ShooterState(3240 * RPM_MULTIPLYER,38+ FAR_ANGLE_ADDITION,.884));
        shooterMap.put(155. + FAR_ADDITION, new ShooterState(3400 * RPM_MULTIPLYER,38+ FAR_ANGLE_ADDITION,.902));
        shooterMap.put(165.+ FAR_ADDITION, new ShooterState(3530 * RPM_MULTIPLYER,40.5+ FAR_ANGLE_ADDITION,.890));
    }

    public ShooterState get(double distance) {
        return shooterMap.get(distance);
    }

    public List<Double> getDistances(){
        return shooterMap.getKeys();
    }
}

