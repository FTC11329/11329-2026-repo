package org.firstinspires.ftc.teamcode.util.shooterInterpolation;

import com.bylazar.configurables.annotations.Configurable;

import java.util.List;

@Configurable
public class ShooterTestValues implements ShooterValuesParent {
    private final ShooterTreeInterpolation shooterMap = new ShooterTreeInterpolation();

    private final double CLOSE_ADDITION = 0; // - shooter farther
    private final double ADDITION = 0; // - shooter farther
    private final double FAR_ADDITION = 0; // - shooter farther
    private final double RPM_MULTIPLYER = 0.87; // >1 shooter farther
    private final double FAR_ANGLE_ADDITION = 0; // + shooter farther
    public ShooterTestValues() {
        // Distance units must have a consistent unit
        shooterMap.put(25 + ADDITION + CLOSE_ADDITION, new ShooterState(2500 * RPM_MULTIPLYER, 16, .736));
        shooterMap.put(35 + ADDITION + CLOSE_ADDITION, new ShooterState(2390 * RPM_MULTIPLYER, 22, .524));
        shooterMap.put(45 + ADDITION + CLOSE_ADDITION, new ShooterState(2450 * RPM_MULTIPLYER, 24.6, .576));
        shooterMap.put(55. + ADDITION + CLOSE_ADDITION, new ShooterState(2580 * RPM_MULTIPLYER, 28.5, .588));
        shooterMap.put(65. + ADDITION + CLOSE_ADDITION, new ShooterState(2790 * RPM_MULTIPLYER, 30.9, .628));
        shooterMap.put(75. + ADDITION + CLOSE_ADDITION, new ShooterState(2878 * RPM_MULTIPLYER,32.8,.644));
        shooterMap.put(85. + ADDITION, new ShooterState(3080 * RPM_MULTIPLYER,34,.666));
        shooterMap.put(95. + ADDITION, new ShooterState(3150 * RPM_MULTIPLYER,38,.742));
        shooterMap.put(105 + ADDITION, new ShooterState(3340 * RPM_MULTIPLYER,42,.662));
        shooterMap.put(115 + FAR_ADDITION, new ShooterState(3362 * RPM_MULTIPLYER,40.5+ FAR_ANGLE_ADDITION,0.736));
        shooterMap.put(125. + FAR_ADDITION, new ShooterState(3540 * RPM_MULTIPLYER,37.5+ FAR_ANGLE_ADDITION,.810));
        shooterMap.put(135. + FAR_ADDITION, new ShooterState(3650 * RPM_MULTIPLYER,36+ FAR_ANGLE_ADDITION,.934));
        shooterMap.put(145. + FAR_ADDITION, new ShooterState(3750 * RPM_MULTIPLYER,38.5+ FAR_ANGLE_ADDITION,.884));
        shooterMap.put(155. + FAR_ADDITION, new ShooterState(3900 * RPM_MULTIPLYER,40+ FAR_ANGLE_ADDITION,.902));
        shooterMap.put(165.+ FAR_ADDITION, new ShooterState(4000 * RPM_MULTIPLYER,42+ FAR_ANGLE_ADDITION,.890));
    }

    public ShooterState get(double distance) {
        return shooterMap.get(distance);
    }

    public List<Double> getDistances(){
        return shooterMap.getKeys();
    }
}

