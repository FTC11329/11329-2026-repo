
package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.RobotSide;
import org.firstinspires.ftc.teamcode.util.AutoEnums.BlueCloseAutoPhases;

@Autonomous(name = "BlueCloseAuto", group = "zautos")
public class BlueCloseAuto extends OpMode {
	private Robot robot;
	private Timer pathTimer, actionTimer, opmodeTimer;
	private BlueCloseAutoPhases pathState;
	private boolean prepareToShoot = false;
	private boolean shoot = false;
	private boolean lastShoot = false;
	
	private double shootTime = 5.0; // seconds to shoot all 3 balls
	private double shootPower = 0.5; // power to move while shooting

	// POSES *******************************~

	private Pose startPose = new Pose(65, 36, Math.toRadians(90));
    private Pose endShoot1Pose = new Pose(12, 12, Math.toRadians(90));

    private Pose intake1StartPose = new Pose(12, 30, Math.toRadians(90));
    private Pose intake1EndPose = new Pose(12, 48, Math.toRadians(90));
    private Pose pushGateEndPose = new Pose(0, 52, Math.toRadians(90));
    private Pose startShoot2Pose = new Pose(36, 36, Math.toRadians(135));
    private Pose endShoot2Pose = new Pose(12, 12, Math.toRadians(135));

    private Pose intake2StartPose = new Pose(-12, 30, Math.toRadians(90));
    private Pose intake2EndPose = new Pose(-12, 48, Math.toRadians(90));
    private Pose startShoot3Pose = new Pose(24, 24, Math.toRadians(135));
    private Pose endShoot3Pose = new Pose(12, 12, Math.toRadians(135));

    private Pose intake3Pose = new Pose(-36, 30, Math.toRadians(90));
    private Pose intake3EndPose = new Pose(-36, 48, Math.toRadians(90));
    private Pose startShoot4Pose = new Pose(12, 12, Math.toRadians(135));

    private Pose toSTunnelControlPoint = new Pose(-24, 30, Math.toRadians(0));
    private Pose startSTunnelPose = new Pose(-30, 60, Math.toRadians(110));
    private Pose endSTunnelPose = new Pose(-50, 60, Math.toRadians(135));
	private Pose startShoot5Pose = new Pose(12, 12, Math.toRadians(135));
	private Pose endShoot5Pose = new Pose(24, 24, Math.toRadians(135));

	private Pose endPose = new Pose(0, 48, Math.toRadians(135));

	// PATHS *******************************~

	private Path shootPath1;

	private PathChain firstMovement;
	private Path moveToIntake1;
	private Path grabIntake1;
	private Path pushGate1;
	private Path moveToShoot2;

	private Path shootPath2;

	private PathChain secondMovement;
	private Path moveToIntake2;
	private Path grabIntake2;
	private Path moveToShoot3;

	private Path shootPath3;

	private PathChain thirdMovement;
	private Path moveToIntake3;
	private Path grabIntake3;
	private Path moveToShoot4;

	private PathChain sTunnelMovement;
	private Path moveToSTunnel;;
	private Path intakeSTunnel;
	private Path moveToShoot5;

	private Path shootPath5;

	private Path endAutoPath;


	public void buildPaths() {
		shootPath1 = robot.follower.linearPathBuilder(startPose, shoo));
	}

	public void autonomousPathUpdate() {
		switch (pathState) {
			case run:
				prepareToShoot = true;
				shoot = true;
				setPathState(pathState);
				robot.follower.followPath(shootPath1);
				robot.follower.setMaxPower(shootPower);
				break;
			case shoot1:
				if (pathTimer.getElapsedTimeSeconds() > shootTime) {
					prepareToShoot = false;
					shoot = false;
					robot.follower.followPath(firstMovement);
					robot.follower.setMaxPower(1.0);
					setPathState(BlueCloseAutoPhases.firstMovement);
				}
				break;
			case moveToIntake1:
				if (robot.follower.getErrorY(intake1StartPose) < 1) {
					robot.intakeManual();
					setPathState(BlueCloseAutoPhases.intaking1);
				}
				break;
			case intaking1:
				if (robot.follower.getPose().getX() > 18) {
					robot.stopIntake();
					prepareToShoot = true;
					shoot = true;
					setPathState(BlueCloseAutoPhases.shootPath2);
				}
				break;
			case goToShoot2:
				if (robot.follower.getErrorDistance(startShoot2Pose) < 1.5) {
					robot.follower.followPath(shootPath2);
					robot.follower.setMaxPower(shootPower);

					setPathState(BlueCloseAutoPhases.moveToIntake2);
				}
				break;
			case shoot2:
				if (pathTimer.getElapsedTimeSeconds() > shootTime) {
					prepareToShoot = false;
					shoot = false;
					robot.follower.followPath(secondMovement);
					robot.follower.setMaxPower(1.0);
					setPathState(BlueCloseAutoPhases.moveToIntake2);
				}
				break;
			case moveToIntake2:
				if (robot.follower.getErrorY(intake2StartPose) < 1) {
					robot.intakeManual();
					setPathState(BlueCloseAutoPhases.intaking2);
				}
				break;
			case intaking2:
				if (pathTimer.getElapsedTimeSeconds() > 1.5) {
					prepareToShoot = true;
					setPathState(BlueCloseAutoPhases.goToShoot3);
				}
				break;
			case goToShoot3:
				if (robot.follower.getErrorDistance(startShoot3Pose) < 5) {
					robot.stopIntake();
					shoot = true;
					robot.follower.followPath(shootPath3);
					robot.follower.setMaxPower(shootPower);
					setPathState(BlueCloseAutoPhases.shootPath3);
				}
				break;
			case shoot3:
				if (pathTimer.getElapsedTimeSeconds() > shootTime) {
					prepareToShoot = false;
					shoot = false;
					robot.follower.followPath(thirdMovement);
					robot.follower.setMaxPower(1.0);
					setPathState(BlueCloseAutoPhases.moveToIntake3);
				}
				break;
			case moveToIntake3:
				if (robot.follower.getErrorY(intake3Pose) < 1) {
					robot.intakeManual();
					setPathState(BlueCloseAutoPhases.intaking3);
				}
				break;
			case intaking3:
				if (pathTimer.getElapsedTimeSeconds() > 2.5) {
					prepareToShoot = true;
					setPathState(BlueCloseAutoPhases.goToShoot4);
				}
				break;
			case goToShoot4:
				if (robot.follower.getErrorDistance(startShoot4Pose) < 5) {
					robot.stopIntake();
					shoot = true;
					setPathState(BlueCloseAutoPhases.shootPath4);
				}
				break;
			case shoot4:
				if (pathTimer.getElapsedTimeSeconds() > shootTime) {
					prepareToShoot = false;
					shoot = false;
					robot.follower.followPath(sTunnelMovement);
					setPathState(BlueCloseAutoPhases.moveToSTunnel);
				}
				break;
			case moveToSTunnel:
				if (robot.follower.getErrorY(startSTunnelPose) < 1) {
					robot.intakeManual();
					setPathState(BlueCloseAutoPhases.grabbingSTunnel);
				}
				break;
			case intakingSTunnel:
				if (robot.follower.getErrorDistance(endSTunnelPose) < 3) {
					robot.stopIntake();
					prepareToShoot = true;
					shoot = true;
					setPathState(BlueCloseAutoPhases.shootPath5);
				}
				break;
			case shoot5:
				if (pathTimer.getElapsedTimeSeconds() > shootTime) {
					prepareToShoot = false;
					shoot = false;
					robot.follower.followPath(endAutoPath);
					setPathState(BlueCloseAutoPhases.endAuto);
				}
				break;

		}
	}

	/** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
	public void setPathState(BlueCloseAutoPhases pState) {
		pathState = pState;
		pathTimer.resetTimer();
	}

	/** This method is called once at the init of the OpMode. **/
	@Override
	public void init() {
		pathTimer = new Timer();
		opmodeTimer = new Timer();
		opmodeTimer.resetTimer();
		robot = new Robot(telemetry, hardwareMap, RobotSide.Blue);
		robot.follower.setStartingPose(startPose);
		buildPaths();
	}
	/** This method is called continuously after Init while waiting for "play". **/
	@Override
	public void init_loop() {}
	/** This method is called once at the start of the OpMode.
	 * It runs all the setup actions, including building paths and starting the path system **/
	@Override
	public void start() {
		opmodeTimer.resetTimer();
		setPathState(0);
	}
	/** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
	@Override
	public void loop() {
		// These loop the movements of the robot, these must be called continuously in order to work
		if (prepareToShoot) {
			robot.prepareShooter();
		} else {
			robot.casualShooterModeOn();
		}
		if (shoot) {
			robot.shootQueue(false);
		} else if (lastShoot && !shoot) {
			robot.stopIndexer();
		}
		lastShoot = shoot;
		robot.follower.update();
		autonomousPathUpdate();

		// Feedback to Driver Hub for debugging
		telemetry.addData("path state", pathState);
		telemetry.addData("x", robot.follower.getPose().getX());
		telemetry.addData("y", robot.follower.getPose().getY());
		telemetry.addData("heading", robot.follower.getPose().getHeading());
		telemetry.update();
	}

	/** We do not use this because everything should automatically disable **/
	@Override
	public void stop() {}
}

