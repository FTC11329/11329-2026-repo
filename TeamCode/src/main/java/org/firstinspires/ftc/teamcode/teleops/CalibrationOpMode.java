package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.RobotSide;

import java.util.ArrayList;

@TeleOp(name = "Calibration OpMode", group = "zgroup")
public class CalibrationOpMode extends OpMode {
    Turret turret;
    Follower follower;
    @Override
    public void init() {
        turret = new Turret(hardwareMap, 0, RobotSide.Blue);
        follower = org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower(hardwareMap);
    }
    ArrayList<Ray> rays = new ArrayList<>();

    @Override
    public void loop() {
        follower.updatePose();
        if (gamepad1.aWasPressed()) {
            Pose pose = follower.getPose();

            double turretAngleRad = Math.toRadians(turret.getAngle());
            double globalAngle = pose.getHeading() + turretAngleRad;

            double dx = Math.cos(globalAngle);
            double dy = Math.sin(globalAngle);

            rays.add(new Ray(pose.getX(), pose.getY(), dx, dy));
        }
        if (rays.size() >= 2) {
            double[] goal = estimateIntersection(rays);

            if (goal != null) {
                telemetry.addData("Goal X", Math.round(goal[0] * 10) / 10.0);
                telemetry.addData("Goal Y", Math.round(goal[1] * 10) / 10.0);
            }
        }
        for (Ray ray : rays) {
            telemetry.addData("ray", ray.toString());
        }
    }
    public double[] estimateIntersection(ArrayList<Ray> rays) {
        double A = 0, B = 0, C = 0, D = 0, E = 0;

        for (Ray r : rays) {
            double nx = -r.dy; // normal vector
            double ny = r.dx;

            A += nx * nx;
            B += nx * ny;
            C += ny * ny;
            D += nx * (nx * r.x + ny * r.y);
            E += ny * (nx * r.x + ny * r.y);
        }

        double det = A * C - B * B;

        if (Math.abs(det) < 1e-6) return null; // bad geometry

        double x = (D * C - B * E) / det;
        double y = (A * E - B * D) / det;

        return new double[]{x, y};
    }
}

class Ray {
    double x, y;
    double dx, dy;

    Ray(double x, double y, double dx, double dy) {
        this.x = x;
        this.y = y;
        this.dx = dx;
        this.dy = dy;
    }

    @Override
    public String toString() {
        return "Origin: (" + Math.round(x * 10) / 10.0 + ", " + Math.round(y * 10) / 10.0 + "), Slope: " + Math.round(dx * 10) / 10.0 + " / " + Math.round(dy * 10) / 10.0;
    }
}
