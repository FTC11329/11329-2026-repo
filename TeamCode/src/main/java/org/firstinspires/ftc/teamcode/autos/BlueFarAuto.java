
package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.RobotSide;
import org.firstinspires.ftc.teamcode.util.AutoEnums.BlueFarAutoPhases;

@Autonomous(name = "Blue Far Auto", group = "zautos")
public class BlueFarAuto extends OpMode {
	private Robot robot;
	private Timer pathTimer, actionTimer, opmodeTimer;
	private BlueFarAutoPhases pathState;
	private boolean prepareToShoot = false;
	private boolean shoot = false;
	private boolean lastShoot = false;
	
	private double shootTime = 2.5; // seconds to shoot all 3 balls
	private double maxPower = 0.7; // power to move while shooting
	private double intaketime = 0.8; // seconds to fully intake balls after reaching intakeEnd

	// POSES *******************************~

	private Pose startPose = new Pose(-60, 12, Math.toRadians(90));
    private Pose shootPose = new Pose(-60, 12, Math.toRadians(135));


    private Pose intakeHumanStartPose = new Pose(-60, 48, Math.toRadians(135));
    private Pose intakeHumanEndPose = new Pose(-62, 60, Math.toRadians(90));

    private Pose intake2StartPose = new Pose(-36, 32, Math.toRadians(90));
    private Pose intake2EndPose = new Pose(-36, 58, Math.toRadians(90));

    private Pose intake3StartPose = new Pose(-12, 32, Math.toRadians(90));
    private Pose intake3EndPose = new Pose(-12, 58, Math.toRadians(90));

	private Pose endPose = new Pose(0, 48, Math.toRadians(135));

	// PATHS *******************************~

	private PathChain firstMovement;

	private PathChain secondMovement;

	private PathChain thirdMovement;

	private Path endAutoPath;


	public void buildPaths() {

		firstMovement = robot.follower.pathBuilder()
				.addPath(robot.follower.linearPathBuilder(startPose, intakeHumanStartPose))
				.addPath(robot.follower.linearPathBuilder(intakeHumanStartPose, intakeHumanEndPose))
				.addPath(robot.follower.linearPathBuilder(intakeHumanEndPose, shootPose))
				.build();

		secondMovement = robot.follower.pathBuilder()
				.addPath(robot.follower.linearPathBuilder(shootPose, intake2StartPose))
				.addPath(robot.follower.linearPathBuilder(intake2StartPose, intake2EndPose))
				.addPath(robot.follower.linearPathBuilder(intake2EndPose, shootPose))
				.build();


		thirdMovement = robot.follower.pathBuilder()
				.addPath(robot.follower.linearPathBuilder(shootPose, intake3StartPose))
				.addPath(robot.follower.linearPathBuilder(intake3StartPose, intake3EndPose))
				.addPath(robot.follower.linearPathBuilder(intake3EndPose, shootPose))
				.build();

		endAutoPath = robot.follower.linearPathBuilder(shootPose, endPose);


	}

	public void autonomousPathUpdate() {
		switch (pathState) {
			case run:
				robot.follower.setMaxPower(maxPower);
				prepareToShoot = true;
				setPathState(BlueFarAutoPhases.prepShot1);
				break;
			case prepShot1:
				if (pathTimer.getElapsedTimeSeconds() > 2) {
					shoot = true;
					setPathState(BlueFarAutoPhases.shoot1);
				}
				break;
			case shoot1:
				if (pathTimer.getElapsedTimeSeconds() > shootTime) {
					prepareToShoot = false;
					shoot = false;
					robot.intakeManual();
					robot.follower.followPath(firstMovement);
					setPathState(BlueFarAutoPhases.intaking1);
				}
				break;
			case intaking1:
				if (robot.follower.getCurrentPathNumber() == 2) {
					prepareToShoot = true;
					shoot = true;
					setPathState(BlueFarAutoPhases.stillIntaking1);
				}
				break;
			case stillIntaking1:
				if (pathTimer.getElapsedTimeSeconds() > intaketime) {
//					robot.stopIntake();
					setPathState(BlueFarAutoPhases.goToShoot2);
				}
				break;
			case goToShoot2:
				if (robot.follower.getErrorDistance(shootPose) < 1.5) {
					setPathState(BlueFarAutoPhases.shoot2);
				}
				break;
			case shoot2:
				if (pathTimer.getElapsedTimeSeconds() > shootTime) {
					prepareToShoot = false;
					shoot = false;
					robot.intakeManual();
					robot.follower.followPath(secondMovement);
					robot.follower.setMaxPower(maxPower);
					setPathState(BlueFarAutoPhases.intaking2);
				}
				break;
			case intaking2:
				if (robot.follower.getCurrentPathNumber() == 2) {
					shoot = true;
					prepareToShoot = true;
					setPathState(BlueFarAutoPhases.stillIntaking2);
				}
				break;
			case stillIntaking2:
				if (pathTimer.getElapsedTimeSeconds() > intaketime) {
//					robot.stopIntake();
					setPathState(BlueFarAutoPhases.goToShoot3);
				}
				break;
			case goToShoot3:
				if (robot.follower.getErrorDistance(shootPose) < 1.5) {
					setPathState(BlueFarAutoPhases.shoot3);
				}
				break;
			case shoot3:
				if (pathTimer.getElapsedTimeSeconds() > shootTime) {
					prepareToShoot = false;
					shoot = false;
					robot.intakeManual();
					robot.follower.followPath(thirdMovement);
					robot.follower.setMaxPower(maxPower);
					setPathState(BlueFarAutoPhases.moveToIntake3);
				}
				break;
			case moveToIntake3:
				if (robot.follower.getErrorY(intake3StartPose) < 1) {
//					robot.manualIntake();
					setPathState(BlueFarAutoPhases.intaking3);
				}
				break;
			case intaking3:
				if (robot.follower.getCurrentPathNumber() == 2) {
					prepareToShoot = true;
					shoot = true;
					setPathState(BlueFarAutoPhases.stillIntaking3);
				}
				break;
			case stillIntaking3:
				if (pathTimer.getElapsedTimeSeconds() > intaketime) {
//					robot.stopIntake();
					setPathState(BlueFarAutoPhases.goToShoot4);
				}
				break;
			case goToShoot4:
				if (robot.follower.getErrorDistance(shootPose) < 1.5) {
					setPathState(BlueFarAutoPhases.shoot4);
				}
				break;
			case shoot4:
				if (pathTimer.getElapsedTimeSeconds() > shootTime) {
					prepareToShoot = false;
					shoot = false;
					robot.follower.followPath(endAutoPath);
					setPathState(BlueFarAutoPhases.endAuto);
				}
				break;

		}
	}

	/** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
	public void setPathState(BlueFarAutoPhases pState) {
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
		setPathState(BlueFarAutoPhases.run);
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

