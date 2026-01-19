
package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.AutoEnums.FarAutoPhases;
import org.firstinspires.ftc.teamcode.util.EndValuesStorer;
import org.firstinspires.ftc.teamcode.util.FancyButton;
import org.firstinspires.ftc.teamcode.util.RobotSide;

@Autonomous(name = "Red Far Auto", group = " autos")
public class RedFarAuto extends OpMode {
	private Robot robot;
	private Timer pathTimer, actionTimer, opmodeTimer;
	private FarAutoPhases pathState;
	private boolean stopAuto = false;
	private boolean prepareToShoot = false;
	private boolean shoot = false;
	private boolean lastShoot = false;
	private FancyButton gamepadStop;

	private double shootTime = 4; // seconds to shoot all 3 balls
	private double maxPower = 1; // max power to move
	private double intakePower = 0.6; // max power to intake
	private double intaketime = 1; // seconds to fully intake balls after reaching intakeEnd

	// POSES *******************************~

	private Pose startPose = new Pose(-63.65, -16.8, Math.toRadians(270));
    private Pose shootPose = new Pose(-60, -12, Math.toRadians(270));

    private Pose intakeHumanStartPose = new Pose(-55.2, -60.3, Math.toRadians(232));
    private Pose intakeHumanEndPose = new Pose(-62, -59, Math.toRadians(275));

    private Pose intake2StartPose = new Pose(-36, -32, Math.toRadians(270));
    private Pose intake2EndPose = new Pose(-36, -54, Math.toRadians(270));

    private Pose intake3StartPose = new Pose(-12, -32, Math.toRadians(270));
    private Pose intake3EndPose = new Pose(-12, -54, Math.toRadians(270));

	private Pose endPose = new Pose(0, -36, Math.toRadians(270));

	// PATHS *******************************~

	private PathChain toWall1;
	private Path toShoot1;

	private PathChain secondMovement;

	private PathChain thirdMovement;

	private Path endAutoPath;


	public void buildPaths() {

		toWall1 = robot.follower.pathBuilder()
				.addPath(robot.follower.linearPathBuilder(startPose, intakeHumanStartPose))
				.addPath(robot.follower.linearPathBuilder(intakeHumanStartPose, intakeHumanEndPose))
				.build();

		toShoot1 = robot.follower.linearPathBuilder(intakeHumanEndPose, shootPose);

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
				setPathState(FarAutoPhases.prepShot1);
				break;
			case prepShot1:
				if (pathTimer.getElapsedTimeSeconds() > 1.5) {
					shoot = true;
					setPathState(FarAutoPhases.shoot1);
				}
				break;
			case shoot1:
				if (pathTimer.getElapsedTimeSeconds() > shootTime) {
					prepareToShoot = false;
					shoot = false;
					robot.doIntake();
					robot.follower.followPath(toWall1);
					setPathState(FarAutoPhases.intaking1);
				}
				break;
			case intaking1:
				if (robot.follower.getErrorDistance(intakeHumanEndPose) < 0.75 || pathTimer.getElapsedTimeSeconds() > 2.5) {
					prepareToShoot = true;
					shoot = true;
					setPathState(FarAutoPhases.stillIntaking1);
				}
				break;
			case stillIntaking1:
				if (pathTimer.getElapsedTimeSeconds() > 0.5) {
//					robot.stopIntake();
					robot.follower.followPath(toShoot1);
					setPathState(FarAutoPhases.goToShoot2);
				}
				break;
			case goToShoot2:
				if (robot.follower.getErrorDistance(shootPose) < 1.5) {
					setPathState(FarAutoPhases.shoot2);
				}
				break;
			case shoot2:
				if (pathTimer.getElapsedTimeSeconds() > shootTime) {
					prepareToShoot = false;
					shoot = false;
					robot.doIntake();
					robot.follower.followPath(secondMovement);
					robot.follower.setMaxPower(maxPower);
					setPathState(FarAutoPhases.intaking2);
				}
				break;
			case intaking2:
				if (robot.follower.getCurrentPathNumber() == 2) {
					shoot = true;
					prepareToShoot = true;
					setPathState(FarAutoPhases.stillIntaking2);
				}
				break;
			case stillIntaking2:
				if (pathTimer.getElapsedTimeSeconds() > intaketime) {
//					robot.stopIntake();
					setPathState(FarAutoPhases.goToShoot3);
				}
				break;
			case goToShoot3:
				if (robot.follower.getErrorDistance(shootPose) < 1.5) {
					setPathState(FarAutoPhases.shoot3);
				}
				break;
			case shoot3:
				if (pathTimer.getElapsedTimeSeconds() > shootTime) {
					prepareToShoot = false;
					shoot = false;
					robot.doIntake();
					robot.follower.followPath(thirdMovement);
					robot.follower.setMaxPower(maxPower);
					setPathState(FarAutoPhases.moveToIntake3);
				}
				break;
			case moveToIntake3:
				if (robot.follower.getErrorY(intake3StartPose) < 1) {
//					robot.manualIntake();
					setPathState(FarAutoPhases.intaking3);
				}
				break;
			case intaking3:
				if (robot.follower.getCurrentPathNumber() == 2) {
					prepareToShoot = true;
					shoot = true;
					setPathState(FarAutoPhases.stillIntaking3);
				}
				break;
			case stillIntaking3:
				if (pathTimer.getElapsedTimeSeconds() > intaketime) {
//					robot.stopIntake();
					setPathState(FarAutoPhases.goToShoot4);
				}
				break;
			case goToShoot4:
				if (robot.follower.getErrorDistance(shootPose) < 1.5) {
					setPathState(FarAutoPhases.shoot4);
				}
				break;
			case shoot4:
				if (pathTimer.getElapsedTimeSeconds() > shootTime) {
					prepareToShoot = false;
					shoot = false;
					robot.follower.followPath(endAutoPath);
					setPathState(FarAutoPhases.endAuto);
				}
				break;

		}
	}

	/** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
	public void setPathState(FarAutoPhases pState) {
		pathState = pState;
		pathTimer.resetTimer();
	}

	/** This method is called once at the init of the OpMode. **/
	@Override
	public void init() {
		pathTimer = new Timer();
		opmodeTimer = new Timer() ;
		opmodeTimer.resetTimer();
		robot = new Robot(telemetry, hardwareMap, RobotSide.Red, 0, 0);
		robot.follower.setStartingPose(startPose);
		gamepadStop = new FancyButton(FancyButton.PressType.Toggle);
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
		setPathState(FarAutoPhases.run);
	}
	/** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
	@Override
	public void loop() {
		gamepadStop.checkStatus(gamepad1.ps);
		// These loop the movements of the robot, these must be called continuously in order to work
		if (robot.follower.getFollowingPathChain()) {
			if (false) {
				switch (robot.follower.getCurrentPathNumber()) {
					case 0:
						robot.follower.setMaxPower(maxPower);
						break;
					case 1:
						robot.follower.setMaxPower(maxPower);
						break;
					case 3:
						robot.follower.setMaxPower(maxPower);
						break;
				}
			} else {
				switch (robot.follower.getCurrentPathNumber()) {
					case 0:
						robot.follower.setMaxPower(maxPower);
						break;
					case 1:
						robot.follower.setMaxPower(intakePower);
						break;
					case 2:
						robot.follower.setMaxPower(maxPower);
						break;
				}
			}
		}

		if (prepareToShoot) {
			robot.prepareShooter();
		} else {
			robot.casualShooterModeOn();
		}
//		robot.autoShoot(true);
		robot.update();
		if (opmodeTimer.getElapsedTimeSeconds() < 29) {
			autonomousPathUpdate();
		} else {
			if (!stopAuto) {
				robot.follower.breakFollowing();
				robot.turret.setTargetDeg(robot.turret.getAngle());
				robot.shooter.setTargetRPM(0);
				robot.shooter.setHoodDeg(5);
				robot.drivetrain.stop();
				stopAuto = true;
			}
		}

		// Feedback to Driver Hub for debugging
		telemetry.addData("path state", pathState);
		telemetry.addData("x", robot.follower.getPose().getX());
		telemetry.addData("y", robot.follower.getPose().getY());
		telemetry.addData("heading", robot.follower.getPose().getHeading());
		telemetry.update();
	}

	@Override
	public void stop() {
		EndValuesStorer endValuesStorer = new EndValuesStorer();
		endValuesStorer.saveEndValues(robot.getCurrentPose().getX(), robot.getCurrentPose().getY(), robot.getCurrentPose().getHeading(), robot.turret.getTicks(), robot.indexer.getEncoderPercentage());
	}
}

