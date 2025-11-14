package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.RobotSide;

import java.util.ArrayList;

public class Robot {
    ArrayList<BallColor> queuedBalls = new ArrayList<>();
    int oneBallCase = 0;
    int burst = 3;
    Stilts stilts;
    Intake intake;
    Indexer indexer;
    Shooter shooter;
    Follower follower;
    Drivetrain drivetrain;
    Turret turret;
    ElapsedTime time;
    RobotSide robotSide;
    public Robot(HardwareMap hardwareMap, RobotSide robotSide) {
        this.robotSide = robotSide;
        stilts = new Stilts(hardwareMap);
        intake = new Intake(hardwareMap);
        indexer = new Indexer(hardwareMap);
        shooter = new Shooter(hardwareMap);
//        follower = new Follower(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);
        time = new ElapsedTime();

    }
    double startTime;
    //FNCTION TO SHOOT one ball
    public boolean singleBallShoot(BallColor colorOfBall) {
        switch (oneBallCase) {
            case 0:
                if (indexer.SpinTill(colorOfBall)) {
                    oneBallCase =  1;
                }
                return false;
            case 1:
                turret.turnTo(Constants.PlacholdereDouble);
                shooter.setHoodDeg(Constants.PlacholdereDouble);
                if (shooter.spinUp()) {
                    startTime = time.milliseconds();
                    oneBallCase = 3-1;
                }
                return false;
            case 3-1:
                indexer.ballToShooter();
                if (time.milliseconds() - startTime < 300) {
                    shooter.lightSpin();
                    oneBallCase = 0;
                    return true;
                }
                return false;
        }
        return false;
    }
    public void shootQueue(){
        if (queuedBalls.get(0) == null) {
            return;
        }
        if (singleBallShoot(queuedBalls.get(0))) {
            queuedBalls.remove(0);
        }
    }
    public void aFunctionThatQsBalls(BallColor qdColor) {
        queuedBalls.add(qdColor);
    }

    //function to intake and SpinDexer
    public void Intakenspindex(){
    }

    public void update() {

    }

    //FNCTION TO SHOOT obe ball
    public void singleball{
        indexer.SpinTill(BallColor.None);

    }
    //FNCTION TO SHOOT green ball
    public void greenball{
        indexer.SpinTill(BallColor.Green);

    }
    //FNCTION TO SHOOT purple ball
    public void purpleball{
        indexer.SpinTill(BallColor.Purple);
    }
    //FNCTION TO SHOOT 3 ball
    public void threeRoundBurst{

    }
    //function to intake and spindexer
    public void Intakenspindex{
    }
}
