
package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.FuturePose;
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
	
	private double shootTime = 2.5; // seconds to shoot all 3 balls
	private double shootPower = 0.5; // power to move while shooting
	private double maxPower = 0.7; // power to move while shooting
	private double intaketime = 0.8; // seconds to fully intake balls after reaching intakeEnd

	// POSES *******************************~

	private Pose startPose = new Pose(62.5, 36, Math.toRadians(90));
    private Pose endShoot1Pose = new Pose(12, 12, Math.toRadians(90));

    private Pose intake1StartPose = new Pose(12, 32, Math.toRadians(90));
    private Pose intake1EndPose = new Pose(12, 44.6, Math.toRadians(90));
    private Pose pushGateEndPose = new Pose(0, 53.8, Math.toRadians(100));
    private Pose startShoot2Pose = new Pose(36, 36, Math.toRadians(135));
    private Pose endShoot2Pose = new Pose(12, 12, Math.toRadians(135));

    private Pose intake2StartPose = new Pose(-12, 32, Math.toRadians(90));
    private Pose intake2EndPose = new Pose(-12, 47, Math.toRadians(90));
    private Pose startShoot3Pose = new Pose(24, 24, Math.toRadians(135));
    private Pose endShoot3Pose = new Pose(12, 12, Math.toRadians(135));

    private Pose intake3StartPose = new Pose(-36, 32, Math.toRadians(90));
    private Pose intake3EndPose = new Pose(-36, 47, Math.toRadians(90));
    private Pose shoot4Pose = new Pose(12, 12, Math.toRadians(135));

    private Pose toSTunnelControlPoint = new Pose(-24, 30, Math.toRadians(0));
    private Pose startSTunnelPose = new Pose(-30, 60, Math.toRadians(110));
    private Pose endSTunnelPose = new Pose(-50, 60, Math.toRadians(135));
	private Pose startShoot5Pose = new Pose(12, 12, Math.toRadians(135));
	private Pose endShoot5Pose = new Pose(24, 24, Math.toRadians(135));

	private Pose endPose = new Pose(0, 48, Math.toRadians(135));

	// PATHS *******************************~

	private Path shootPath1;

	private PathChain firstMovement;

	private Path shootPath2;

	private PathChain secondMovement;

	private Path shootPath3;

	private PathChain thirdMovement;

	private PathChain sTunnelMovement;

	private Path shootPath5;

	private Path endAutoPath;


	public void buildPaths() {
		shootPath1 = robot.follower.linearPathBuilder(startPose, endShoot1Pose);

		firstMovement = robot.follower.pathBuilder()
				.addPath(robot.follower.linearPathBuilder(endShoot1Pose, intake1StartPose))
				.addPath(robot.follower.linearPathBuilder(intake1StartPose, intake1EndPose))
				.addPath(robot.follower.linearPathBuilder(intake1EndPose, pushGateEndPose))
				.addPath(robot.follower.linearPathBuilder(pushGateEndPose, startShoot2Pose))
				.build();

		shootPath2 = robot.follower.linearPathBuilder(startShoot2Pose, endShoot2Pose);

		secondMovement = robot.follower.pathBuilder()
				.addPath(robot.follower.linearPathBuilder(endShoot2Pose, intake2StartPose))
				.addPath(robot.follower.linearPathBuilder(intake2StartPose, intake2EndPose))
				.addPath(robot.follower.linearPathBuilder(intake2EndPose, startShoot3Pose))
				.build();

		shootPath3 = robot.follower.linearPathBuilder(startShoot3Pose, endShoot2Pose);

		thirdMovement = robot.follower.pathBuilder()
				.addPath(robot.follower.linearPathBuilder(endShoot3Pose, intake3StartPose))
				.addPath(robot.follower.linearPathBuilder(intake3StartPose, intake3EndPose))
				.addPath(robot.follower.linearPathBuilder(intake3EndPose, shoot4Pose))
				.build();

		sTunnelMovement = robot.follower.pathBuilder()
				.addPath(new BezierCurve(shoot4Pose, toSTunnelControlPoint, startSTunnelPose))
				.addPath(robot.follower.linearPathBuilder(startSTunnelPose, endSTunnelPose))
				.addPath(robot.follower.linearPathBuilder(endSTunnelPose, startShoot5Pose))
				.build();

		shootPath5 = robot.follower.linearPathBuilder(startShoot5Pose, endShoot5Pose);

		endAutoPath = robot.follower.linearPathBuilder(endShoot5Pose, endPose);


	}

	public void autonomousPathUpdate() {
		switch (pathState) {
			case run:
				prepareToShoot = true;
				robot.follower.followPath(shootPath1);
				robot.follower.setMaxPower(shootPower);
				setPathState(BlueCloseAutoPhases.prepShot1);
				break;
			case prepShot1:
				if (pathTimer.getElapsedTimeSeconds() > 2) {
					shoot = true;
					setPathState(BlueCloseAutoPhases.shoot1);
				}
				break;
			case shoot1:
				if (pathTimer.getElapsedTimeSeconds() > shootTime) {
					prepareToShoot = false;
					shoot = false;
					robot.intakeManual();
					robot.follower.followPath(firstMovement);
					robot.follower.setMaxPower(maxPower);
					setPathState(BlueCloseAutoPhases.moveToIntake1);
				}
				break;
			case moveToIntake1:
				if (robot.follower.getCurrentPathNumber() == 2) {
					setPathState(BlueCloseAutoPhases.intaking1);
				}
				break;
			case intaking1:
				if (robot.follower.getCurrentPathNumber() == 2) {
					setPathState(BlueCloseAutoPhases.stillIntaking1);
				}
				break;
			case stillIntaking1:
				if (pathTimer.getElapsedTimeSeconds() > intaketime) {
					prepareToShoot = true;
					shoot = true;
					setPathState(BlueCloseAutoPhases.goToShoot2);
				}
				break;
			case goToShoot2:
				if (robot.follower.getErrorDistance(startShoot2Pose) < 1.5) {
					robot.follower.followPath(shootPath2);
					robot.follower.setMaxPower(shootPower);

					setPathState(BlueCloseAutoPhases.shoot2);
				}
				break;
			case shoot2:
				if (pathTimer.getElapsedTimeSeconds() > shootTime) {
					prepareToShoot = false;
					shoot = false;
					robot.intakeManual();
					robot.follower.followPath(secondMovement);
					robot.follower.setMaxPower(maxPower);
					setPathState(BlueCloseAutoPhases.moveToIntake2);
				}
				break;
			case moveToIntake2:
				if (robot.follower.getErrorY(intake2StartPose) < 1) {
					setPathState(BlueCloseAutoPhases.intaking2);
				}
				break;
			case intaking2:
				if (robot.follower.getCurrentPathNumber() == 2) {
					setPathState(BlueCloseAutoPhases.stillIntaking2);
				}
				break;
			case stillIntaking2:
				if (pathTimer.getElapsedTimeSeconds() > intaketime) {
					shoot = true;
					prepareToShoot = true;
					setPathState(BlueCloseAutoPhases.goToShoot3);
				}
				break;
			case goToShoot3:
				if (robot.follower.getErrorDistance(startShoot3Pose) < 5) {
					robot.follower.followPath(shootPath3);
					robot.follower.setMaxPower(shootPower);
					setPathState(BlueCloseAutoPhases.shoot3);
				}
				break;
			case shoot3:
				if (pathTimer.getElapsedTimeSeconds() > shootTime) {
					prepareToShoot = false;
					shoot = false;
					robot.intakeManual();
					robot.follower.followPath(thirdMovement);
					robot.follower.setMaxPower(maxPower);
					setPathState(BlueCloseAutoPhases.moveToIntake3);
				}
				break;
			case moveToIntake3:
				if (robot.follower.getErrorY(intake3StartPose) < 1) {
					setPathState(BlueCloseAutoPhases.intaking3);
				}
				break;
			case intaking3:
				if (robot.follower.getCurrentPathNumber() == 2) {
					setPathState(BlueCloseAutoPhases.stillIntaking3);
				}
				break;
			case stillIntaking3:
				if (pathTimer.getElapsedTimeSeconds() > intaketime) {
					prepareToShoot = true;
					shoot = true;
					setPathState(BlueCloseAutoPhases.goToShoot4);
				}
				break;
			case goToShoot4:
				if (robot.follower.getErrorDistance(shoot4Pose) < 1.5) {
					setPathState(BlueCloseAutoPhases.shoot4);
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
					setPathState(BlueCloseAutoPhases.intakingSTunnel);
				}
				break;
			case intakingSTunnel:
				if (robot.follower.getCurrentPathNumber() == 2) {
					setPathState(BlueCloseAutoPhases.stillIntaking4);
				}
				break;
			case stillIntaking4:
				if (pathTimer.getElapsedTimeSeconds() > intaketime) {
					setPathState(BlueCloseAutoPhases.shoot5);
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
		setPathState(BlueCloseAutoPhases.run);
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
		}
		if (lastShoot && !shoot) {
			robot.stopIndexer();
		}
		lastShoot = shoot;
		robot.update();
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

