package org.firstinspires.ftc.teamcode.util.ShootOnTheFly;

public class HoodAngleCompensation {
    double rpmRatio;
    public HoodAngleCompensation() {}
    public double hoodAngleCompensation(double targetRPM, double actualRPM, double hoodDeg) {

        double hoodRad = Math.toRadians(hoodDeg);

        rpmRatio = targetRPM / actualRPM;


        double correctedRad =
                Math.atan(rpmRatio * Math.tan(hoodRad));

        double deltaDeg =
                Math.toDegrees(correctedRad - hoodRad);

        if (Math.abs(deltaDeg) < .7) deltaDeg = 0;

        deltaDeg = clamp(deltaDeg, -8.0, 8.0);

        return deltaDeg;
    }
    public double getRpmRatio() {
        return rpmRatio;
    }
    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
