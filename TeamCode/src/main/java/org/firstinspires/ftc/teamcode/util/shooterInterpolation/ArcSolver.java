package org.firstinspires.ftc.teamcode.util.shooterInterpolation;

public class ArcSolver {
    private static final double G = 386.09;       // gravity, in/s²
    private static final double TARGET_Y = 43.75; // goal lip (38.75) + ball radius (2.5), inches

    // Polynomial coefficients - all functions of nominal distance d (inches)
    // v0: shooter exit velocity (in/s)
    private static final double[] P_V0    = {-0.000056910, 0.01737570, -0.807701, 186.81829};
    // tof: time of flight (s)
    private static final double[] P_TOF   = {-0.000000608, 0.00019491, -0.015893, 1.008565};
    // hood: mechanical hood angle (degrees)
    private static final double[] P_HOOD  = { 0.000024388, -0.00903308, 1.113230, -7.669342};
    // v0/rpm ratio (unitless)
    private static final double[] P_RATIO = {-0.00000047598, 0.000041307, 0.083919};
    private static final double D_MIN     = 10.0;
    private static final double D_MAX     = 189.5;   // hood hits 45° here — hard ceiling
    private static final double RPM_MAX   = 3800.0;
    private static final double HOOD_MAX  = 45.0;

    private double bisectD(double robotX, double shooterH) {
        double dLo = D_MIN, dHi = D_MAX;
        double fLo = residual(dLo, robotX, shooterH);
        double fHi = residual(dHi, robotX, shooterH);

        if (fLo * fHi > 0) {
            for (int step = 1; step <= 20; step++) {
                double nLo = Math.max(1.0,  dLo - step * 8);
                double nHi = Math.min(D_MAX, dHi + step * 8); // never expand past D_MAX
                double fNL = residual(nLo, robotX, shooterH);
                double fNH = residual(nHi, robotX, shooterH);
                if (fNL * fNH <= 0) {
                    dLo = nLo; dHi = nHi; fLo = fNL; fHi = fNH;
                    break;
                }
                if (step == 20) return Math.abs(fLo) < Math.abs(fHi) ? dLo : dHi;
            }
        }

        for (int i = 0; i < 50; i++) {
            double dMid = (dLo + dHi) / 2.0;
            double fMid = residual(dMid, robotX, shooterH);
            if (fLo * fMid <= 0) { dHi = dMid; fHi = fMid; }
            else                  { dLo = dMid; fLo = fMid; }
        }
        return (dLo + dHi) / 2.0;
    }


    public boolean canShoot(double robotX, double shooterH) {
        double d    = bisectD(robotX, shooterH);
        double tof  = polyEval3(P_TOF,   d);
        double hood = polyEval3(P_HOOD,  d);
        double vx   = robotX / tof;
        double vy   = (TARGET_Y - shooterH + 0.5 * G * tof * tof) / tof;
        double v0   = Math.sqrt(vx * vx + vy * vy);
        double rpm  = v0 / polyEval2(P_RATIO, d);
        return rpm <= RPM_MAX && hood <= HOOD_MAX;
    }


    private static double polyEval3(double[] c, double x) {
        // c = {a3, a2, a1, a0}  →  a3*x³ + a2*x² + a1*x + a0
        return ((c[0] * x + c[1]) * x + c[2]) * x + c[3];
    }

    private static double polyEval2(double[] c, double x) {
        // c = {a2, a1, a0}
        return (c[0] * x + c[1]) * x + c[2];
    }

    /**
     * Computes the residual for bisection:
     * physics v0 needed to reach (robotX, TARGET_Y) in tof(d) seconds
     * minus the v0 the shooter produces at nominal distance d.
     * We want this == 0.
     */
    private double residual(double d, double robotX, double shooterH) {
        double tof = polyEval3(P_TOF, d);
        if (tof <= 0) return Double.MAX_VALUE;

        double vx        = robotX / tof;
        double dy        = TARGET_Y - shooterH;
        double vy        = (dy + 0.5 * G * tof * tof) / tof;
        double v0_needed = Math.sqrt(vx * vx + vy * vy);
        double v0_poly   = polyEval3(P_V0, d);

        return v0_needed - v0_poly;
    }

    /**
     * Given robot horizontal distance and shooter height,
     * returns the interpolated ShooterState (rpm, hoodAngle, tof).
     */
    public ShooterState calculateShooterState(double robotX, double shooterH) {
        double dLo = 10.0, dHi = 175.0;
        double fLo = residual(dLo, robotX, shooterH);
        double fHi = residual(dHi, robotX, shooterH);

        // If no sign change, clamp to nearest valid endpoint
        if (fLo * fHi > 0) {
            for (int step = 1; step <= 20; step++) {
                double newLo = Math.max(1.0,   dLo - step * 8);
                double newHi = Math.min(200.0, dHi + step * 8);
                double fNewLo = residual(newLo, robotX, shooterH);
                double fNewHi = residual(newHi, robotX, shooterH);
                if (fNewLo * fNewHi <= 0) {
                    dLo = newLo; dHi = newHi;
                    fLo = fNewLo; fHi = fNewHi;
                    break;
                }
                // If still no bracket after expansion, clamp to nearest
                if (step == 20) return stateFromD((Math.abs(fLo) < Math.abs(fHi)) ? dLo : dHi, robotX, shooterH);
            }
        }

        // Bisect - 50 iterations would give ~1e-14 precision, costs ~microseconds
        for (int i = 0; i < 30; i++) {
            double dMid = (dLo + dHi) / 2.0;
            double fMid = residual(dMid, robotX, shooterH);
            if (fLo * fMid <= 0) { dHi = dMid; fHi = fMid; }
            else                  { dLo = dMid; fLo = fMid; }
        }

        return stateFromD((dLo + dHi) / 2.0, robotX, shooterH);
    }

    private ShooterState stateFromD(double d, double robotX, double shooterH) {
        double tof   = polyEval3(P_TOF,   d);
        double hood  = polyEval3(P_HOOD,  d);
        double vx    = robotX / tof;
        double dy    = TARGET_Y - shooterH;
        double vy    = (dy + 0.5 * G * tof * tof) / tof;
        double v0    = Math.sqrt(vx * vx + vy * vy);
        double ratio = polyEval2(P_RATIO, d);
        double rpm   = v0 / ratio;

        return new ShooterState(rpm, hood, tof);
    }
}
