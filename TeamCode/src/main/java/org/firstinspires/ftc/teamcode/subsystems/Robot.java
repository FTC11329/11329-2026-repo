package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.RobotSide;

public class Robot {
    Stilts stilts;
    Intake intake;
    Indexer indexer;
    Shooter shooter;
    Follower follower;
    Drivetrain drivetrain;

    RobotSide robotSide;
    public Robot(HardwareMap hardwareMap, RobotSide robotSide) {
        this.robotSide = robotSide;
        stilts = new Stilts(hardwareMap);
        intake = new Intake(hardwareMap);
        indexer = new Indexer(hardwareMap);
        shooter = new Shooter(hardwareMap);
//        follower = new Follower(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);

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
