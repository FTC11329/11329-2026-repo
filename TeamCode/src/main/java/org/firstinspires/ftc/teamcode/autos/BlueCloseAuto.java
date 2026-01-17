
package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.math.Vector;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.util.EndValuesStorer;
import org.firstinspires.ftc.teamcode.util.RobotSide;
import org.firstinspires.ftc.teamcode.util.AutoEnums.CloseAutoPhases;

@Autonomous(name = "Blue Close Auto", group = "  autos")
public class BlueCloseAuto extends OpMode {
	private Robot robot;
	private Timer pathTimer, actionTimer, opmodeTimer;
	private CloseAutoPhases pathState;
	private boolean prepareToShoot = false;
	private boolean shoot = false;
	private boolean lastShoot = false;
	private boolean stopAuto = false;

	private double shootTime = 2.8; // seconds to shoot all 3 balls
	private double shootPower = 0.5; // power to move while shooting
	private double maxPower = 1; // max power to move
	private double intakePower = 0.6; // max power to intake
	private double toGatePower = 0.75; // max power to move to gate
	private double intaketime = 1; // seconds to fully intake balls after reaching intakeEnd

	// POSES *******************************~

	private Pose startPose = new Pose(62.5, 36, Math.toRadians(90));
    private Pose endShoot1Pose = new Pose(12, 12, Math.toRadians(90));

    private Pose intake1StartPose = new Pose(12, 32, Math.toRadians(90));
    private Pose intake1EndPose = new Pose(12, 48, Math.toRadians(90));
    private Pose pushGateStartPose = new Pose(6, 48.3, Math.toRadians(90));
    private Pose pushGateEndPose = new Pose(4, 53.2, Math.toRadians(90));
    private Pose startShoot2Pose = new Pose(36, 36, Math.toRadians(135));
    private Pose endShoot2Pose = new Pose(12, 12, Math.toRadians(135));

    private Pose intake2StartPose = new Pose(-12, 32, Math.toRadians(90));
    private Pose intake2EndPose = new Pose(-12, 54, Math.toRadians(90));
    private Pose startShoot3Pose = new Pose(24, 24, Math.toRadians(135));
    private Pose endShoot3Pose = new Pose(12, 12, Math.toRadians(135));

    private Pose intake3StartPose = new Pose(-36, 32, Math.toRadians(90));
    private Pose intake3EndPose = new Pose(-36, 54, Math.toRadians(90));
    private Pose shoot4Pose = new Pose(12, 12, Math.toRadians(135));

    private Pose toSTunnelControlPoint = new Pose(-24, 30, Math.toRadians(0));
    private Pose startSTunnelPose = new Pose(-27, 59.42, Math.toRadians(155));
    private Pose endSTunnelPose = new Pose(-53, 63, Math.toRadians(180));
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
				.addPath(robot.follower.linearPathBuilder(intake1EndPose, pushGateStartPose))
				.addPath(robot.follower.linearPathBuilder(pushGateStartPose, pushGateEndPose))
				.addPath(robot.follower.linearPathBuilder(pushGateEndPose, startShoot2Pose))
				.setConstantHeadingInterpolation(startShoot2Pose.getHeading())
				.build();

		shootPath2 = robot.follower.linearPathBuilder(startShoot2Pose, endShoot2Pose);

		secondMovement = robot.follower.pathBuilder()
				.addPath(robot.follower.linearPathBuilder(endShoot2Pose, intake2StartPose))
				.addPath(robot.follower.linearPathBuilder(intake2StartPose, intake2EndPose))
				.addPath(robot.follower.linearPathBuilder(intake2EndPose, startShoot3Pose))
				.setConstantHeadingInterpolation(startShoot3Pose)
				.build();

		shootPath3 = robot.follower.linearPathBuilder(startShoot3Pose, endShoot2Pose);

		thirdMovement = robot.follower.pathBuilder()
				.addPath(robot.follower.linearPathBuilder(endShoot3Pose, intake3StartPose))
				.addPath(robot.follower.linearPathBuilder(intake3StartPose, intake3EndPose))
				.addPath(robot.follower.linearPathBuilder(intake3EndPose, shoot4Pose))
				.setConstantHeadingInterpolation(shoot4Pose)
				.build();

		sTunnelMovement = robot.follower.pathBuilder()
				.addPath(new BezierCurve(shoot4Pose, toSTunnelControlPoint, startSTunnelPose))
				.setLinearHeadingInterpolation(shoot4Pose, startSTunnelPose)
				.addPath(robot.follower.linearPathBuilder(startSTunnelPose, endSTunnelPose, 0.7))
				.addPath(robot.follower.linearPathBuilder(endSTunnelPose, startShoot5Pose))
				.setConstantHeadingInterpolation(startShoot5Pose)
				.build();

		shootPath5 = robot.follower.linearPathBuilder(startShoot5Pose, endShoot5Pose);

		endAutoPath = robot.follower.linearPathBuilder(endShoot5Pose, endPose);


	}

	public void autonomousPathUpdate() {
		switch (pathState) {
			case run:
				robot.intakeManual();
				prepareToShoot = true;
				robot.follower.followPath(shootPath1);
				robot.follower.setMaxPower(shootPower);
				setPathState(CloseAutoPhases.prepShot1);
				break;
			case prepShot1:
				if (pathTimer.getElapsedTimeSeconds() > 1.75) {
					shoot = true;
					setPathState(CloseAutoPhases.shoot1);
				}
				break;
			case shoot1:
				if (pathTimer.getElapsedTimeSeconds() > shootTime) {
					prepareToShoot = false;
					shoot = false;
					robot.follower.followPath(firstMovement);
					robot.follower.setMaxPower(maxPower);
					setPathState(CloseAutoPhases.moveToIntake1);
				}
				break;
			case moveToIntake1:
				if (robot.follower.getCurrentPathNumber() == 2) {
//					robot.intakeManual();
					setPathState(CloseAutoPhases.intaking1);
				}
				break;
			case intaking1:
				if (robot.follower.getCurrentPathNumber() == 2) {
					setPathState(CloseAutoPhases.stillIntaking1);
					prepareToShoot = true;
					shoot = true;
				}
				break;
			case stillIntaking1:
				if (pathTimer.getElapsedTimeSeconds() > intaketime) {
//					robot.stopIntake();
					setPathState(CloseAutoPhases.goToShoot2);
				}
				break;
			case goToShoot2:
				if (robot.follower.getErrorDistance(startShoot2Pose) < 1.5) {
					robot.follower.followPath(shootPath2);
					robot.follower.setMaxPower(shootPower);

					setPathState(CloseAutoPhases.shoot2);
				}
				break;
			case shoot2:
				if (pathTimer.getElapsedTimeSeconds() > shootTime) {
					prepareToShoot = false;
					shoot = false;
					robot.intakeManual();
					robot.follower.followPath(secondMovement);
					robot.follower.setMaxPower(maxPower);
					setPathState(CloseAutoPhases.moveToIntake2);
				}
				break;
			case moveToIntake2:
				if (robot.follower.getErrorY(intake2StartPose) < 1) {
//					robot.intakeManual();
					setPathState(CloseAutoPhases.intaking2);
				}
				break;
			case intaking2:
				if (robot.follower.getCurrentPathNumber() == 2) {
					setPathState(CloseAutoPhases.stillIntaking2);
					shoot = true;
					prepareToShoot = true;
				}
				break;
			case stillIntaking2:
				if (pathTimer.getElapsedTimeSeconds() > intaketime) {
//					robot.stopIntake();
					setPathState(CloseAutoPhases.goToShoot3);
				}
				break;
			case goToShoot3:
				if (robot.follower.getErrorDistance(startShoot3Pose) < 5) {
					robot.follower.followPath(shootPath3);
					robot.follower.setMaxPower(shootPower);
					setPathState(CloseAutoPhases.shoot3);
				}
				break;
			case shoot3:
				if (pathTimer.getElapsedTimeSeconds() > shootTime) {
					prepareToShoot = false;
					shoot = false;
					robot.intakeManual();
					robot.follower.followPath(thirdMovement);
					robot.follower.setMaxPower(maxPower);
					setPathState(CloseAutoPhases.moveToIntake3);
				}
				break;
			case moveToIntake3:
				if (robot.follower.getErrorY(intake3StartPose) < 1) {
//					robot.intakeManual();
					setPathState(CloseAutoPhases.intaking3);
				}
				break;
			case intaking3:
				if (robot.follower.getCurrentPathNumber() == 2) {
					prepareToShoot = true;
					shoot = true;
					setPathState(CloseAutoPhases.stillIntaking3);
				}
				break;
			case stillIntaking3:
				if (pathTimer.getElapsedTimeSeconds() > intaketime) {
//					robot.stopIntake();
					setPathState(CloseAutoPhases.goToShoot4);
				}
				break;
			case goToShoot4:
				if (robot.follower.getErrorDistance(shoot4Pose) < 1.5) {
					setPathState(CloseAutoPhases.shoot4);
				}
				break;
			case shoot4:
				if (pathTimer.getElapsedTimeSeconds() > shootTime) {
					prepareToShoot = false;
					shoot = false;
					robot.follower.followPath(sTunnelMovement);
					setPathState(CloseAutoPhases.moveToSTunnel);
				}
				break;
			case moveToSTunnel:
				if (robot.follower.getErrorY(startSTunnelPose) < 1) {
					robot.intakeManual();
					setPathState(CloseAutoPhases.intakingSTunnel);
				}
				break;
			case intakingSTunnel:
				if (robot.follower.getCurrentPathNumber() == 2) {
					prepareToShoot = true;
					setPathState(CloseAutoPhases.stillIntaking4);
				}
				break;
			case stillIntaking4:
				if (pathTimer.getElapsedTimeSeconds() > intaketime) {
					robot.spitIntake();
					setPathState(CloseAutoPhases.outtake);
				}
				break;
			case outtake:
				if (pathTimer.getElapsedTimeSeconds() > Constants.Intake.spitTime) {
					shoot = true;
					robot.intakeManual();
					setPathState(CloseAutoPhases.shoot5);
				}
			case shoot5:
				if (pathTimer.getElapsedTimeSeconds() > shootTime) {
					prepareToShoot = false;
					shoot = false;
					robot.follower.followPath(endAutoPath);
					setPathState(CloseAutoPhases.endAuto);
				}
				break;

		}
	}

	/** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
	public void setPathState(CloseAutoPhases pState) {
		pathState = pState;
		pathTimer.resetTimer();
	}

	/** This method is called once at the init of the OpMode. **/
	@Override
	public void init() {
		pathTimer = new Timer();
		opmodeTimer = new Timer();
		opmodeTimer.resetTimer();
		robot = new Robot(telemetry, hardwareMap, RobotSide.Blue, 0, 0);
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
		robot.follower.setStartingPose(startPose);
		setPathState(CloseAutoPhases.run);
	}
	/** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
	@Override
	public void loop() {
		// These loop the movements of the robot, these must be called continuously in order to work

		if (robot.follower.getFollowingPathChain()) {
			if (robot.follower.getCurrentPathChain() == firstMovement) {
				switch (robot.follower.getCurrentPathNumber()) {
					case 0:
						robot.follower.setMaxPower(maxPower);
						break;
					case 1:
						robot.follower.setMaxPower(intakePower);
						break;
					case 2:
						robot.follower.setMaxPower(toGatePower);
						break;
					case 3:
						robot.follower.setMaxPower(toGatePower);
						break;
				}
			} else if (robot.follower.getCurrentPathChain() == secondMovement) {
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
			} else if (robot.follower.getCurrentPathChain() == thirdMovement) {
				switch (robot.follower.getCurrentPathNumber()) {
					case 0:
						robot.follower.setMaxPower(maxPower);
						break;
					case 1:
						robot.follower.setMaxPower(maxPower);
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
				Pose curPose = robot.getCurrentPose();
				Vector curVel = robot.follower.getVelocity();
				robot.follower.followPath(robot.follower.linearPathBuilder(curPose, curPose.plusVector(curVel, -0.000001)));
				robot.turret.setTargetDeg(robot.turret.getAngle());
				robot.shooter.setTargetRPM(0);
				robot.shooter.setHoodDeg(5);
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

