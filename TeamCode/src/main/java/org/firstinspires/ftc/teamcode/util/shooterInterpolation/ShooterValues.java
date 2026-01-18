package org.firstinspires.ftc.teamcode.util.shooterInterpolation;

public class ShooterValues {
    public static ShooterState getShooterState(double distance) {
        double x = distance;
        double rpm =   2062  +  -15*x      + 0.262*x*x    + -7.85E-04*x*x*x;
        double angle = -13.2 + 1.36*x      + -0.0116*x*x  + 2.83E-05*x*x*x;
        double time =  0.866 + -7.23E-03*x + 7.47E-05*x*x + -1.39E-07*x*x*x;

        ShooterState state = new ShooterState(rpm, angle, time);
        return state;
    }
}
