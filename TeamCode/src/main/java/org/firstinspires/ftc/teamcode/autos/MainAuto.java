package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.AutoPhase;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.RobotSide;

public class MainAuto {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private AutoPhase pathState;
    Robot robot;

    private Path scorePreload;
    private Path shoot1, shoot2, shoot3, end;
    private PathChain collect1, collect2, collect3;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;

    private final Pose startPose = new Pose(120, 128, Math.toRadians(-144)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(100, 100, Math.toRadians(50));
    //Initial
    private final Pose collectI1 = new Pose(100, 85, Math.toRadians(0));
    //Final
    private final Pose collectF1 = new Pose(130, 85, Math.toRadians(0));
    private final Pose collectI2 = new Pose(100, 60, Math.toRadians(0));
    private final Pose collectF2 = new Pose(130, 60, Math.toRadians(0));
    private final Pose collectI3 = new Pose(100, 35, Math.toRadians(0));
    private final Pose collectF3 = new Pose(130, 35, Math.toRadians(0));

    RobotSide robotSide;
    Telemetry telemetry;
    HardwareMap hardwareMap;

    public MainAuto(HardwareMap hardwareMap, Telemetry telemetry, RobotSide robotSide) {
        this.robotSide = robotSide;
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }

    /** This method is called once at the init of the OpMode. **/
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        robot = new Robot(telemetry, hardwareMap, RobotSide.Blue);

    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(AutoPhase.scorePreload);
    }

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = follower.linearPathBuilder(startPose, scorePose);
        shoot1 = follower.linearPathBuilder(collectF1, scorePose);
        shoot2 = follower.linearPathBuilder(collectF2, scorePose);
        shoot3 = follower.linearPathBuilder(collectF3, scorePose);
        collect1 = new PathChain(follower.linearPathBuilder(scorePose, collectI1), follower.linearPathBuilder(collectI1, collectF1));
        collect2 = new PathChain(follower.linearPathBuilder(scorePose, collectI2), follower.linearPathBuilder(collectI2, collectF2));
        collect3 = new PathChain(follower.linearPathBuilder(scorePose, collectI3), follower.linearPathBuilder(collectI3, collectF3));

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case scorePreload:
                follower.followPath(scorePreload);
                setPathState(AutoPhase.shootPreload);
                break;
            case shootPreload:
                if(pathTimer.getElapsedTime() < 1000) {
                    robot.getMotif();
                }
                if(follower.getErrorDistance(scorePose) < 1) {
                    if (robot.shootQueue()){
                        if (pathTimer.getElapsedTime() >= 1000){
                            follower.followPath(collect1);
                            setPathState(AutoPhase.collectArtifacts1);
                        }
                    }
                }
                break;
            case collectArtifacts1:
                if(follower.getChainIndex() > 0) {
                    robot.startIntake();
                }
                if (!follower.isBusy()){
                    robot.QBall(new BallColor[] {BallColor.Purple, BallColor.Green, BallColor.Green});
                    robot.stopIntake();
                    follower.followPath(shoot1);
                    setPathState(AutoPhase.scoreArtifacts1);
                }
                break;
            case scoreArtifacts1:
                if(follower.getErrorDistance(scorePose) < 1) {
                    if (robot.shootQueue()){
                        follower.followPath(collect2);
                        setPathState(AutoPhase.collectArtifacts2);
                    }
                }
                break;
            case collectArtifacts2:
                if(follower.getChainIndex() > 0) {
                    robot.startIntake();
                }
                if (!follower.isBusy()){
                    robot.QBall(new BallColor[] {BallColor.Green, BallColor.Purple, BallColor.Green});
                    robot.stopIntake();
                    robot.stopIntake();
                    follower.followPath(shoot2);
                    setPathState(AutoPhase.scoreArtifacts2);
                }
                break;
            case scoreArtifacts2:
                if(follower.getErrorDistance(scorePose) < 1) {
                    if (robot.shootQueue()){
                        follower.followPath(collect3);
                        setPathState(AutoPhase.collectArtifacts3);
                    }
                }
                break;
            case collectArtifacts3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(follower.getChainIndex() > 0) {
                    robot.startIntake();
                }
                if (!follower.isBusy()){
                    robot.QBall(new BallColor[] {BallColor.Green, BallColor.Green, BallColor.Purple});
                    robot.stopIntake();
                    follower.followPath(shoot3);
                    setPathState(AutoPhase.scoreArtifacts3);
                }
                break;
            case scoreArtifacts3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(follower.getErrorDistance(scorePose) < 1) {
                    if (robot.shootQueue()){
                        follower.followPath(end);
                        setPathState(AutoPhase.end);
                    }
                }
                break;
        }
    }
    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(AutoPhase pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }


    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        robot.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** We do not use this because everything should automatically disable **/
    public void stop() {}
}
