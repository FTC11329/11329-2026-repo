package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.ColorFunctions;
import org.firstinspires.ftc.teamcode.util.ScanPhase;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Indexer {
    // declaring motor variables
    CRServo spindexer1;
    CRServo spindexer2;
    DcMotorEx transfer;
    RevColorSensorV3 colorSensor;
    ScanPhase scanPhase = ScanPhase.pre;

    BallColor color;
    int scanIndex = 0;

    double lastIndexerPower;
    double lastTransferPower;
    ElapsedTime scanTimer;

    public Indexer(HardwareMap hardwaremap){
        spindexer1 = hardwaremap.get(CRServo.class, "spindexer1");
        spindexer2 = hardwaremap.get(CRServo.class, "spindexer2");
        spindexer1.setDirection(DcMotorSimple.Direction.FORWARD);
        spindexer2.setDirection(DcMotorSimple.Direction.FORWARD);

        transfer = hardwaremap.get(DcMotorEx.class, "transfer");
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setCurrentAlert(4, CurrentUnit.AMPS);

        colorSensor = hardwaremap.get(RevColorSensorV3.class, "spindexerColorSensor");
        scanTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        scanTimer.reset();
    }

    public void setIndexerPower(double set) {
        if (lastIndexerPower != set) {
            lastIndexerPower = set;
            spindexer1.setPower(set);
            spindexer2.setPower(set);
        }

    }

    public void setIndexerToShooterPower(double set){
        transfer.setPower(set);
    }

    public NormalizedRGBA getColor(){
        return colorSensor.getNormalizedColors();
    }
    public double getDistance(){
        return colorSensor.getDistance(DistanceUnit.INCH);
    }

    // Spins the indexer until the correct color is in front of the sensor then stops the spindexer
    public boolean spinUntil(BallColor ballColor) {
        setIndexerPower(Constants.Indexer.spindexPower);
        NormalizedRGBA currentColor = getColor();

        return ColorFunctions.toColor(currentColor, getDistance()) == ballColor;
    }

    public List<BallColor> scanIndexer(List<BallColor> balls){
        color = ColorFunctions.toColor(getColor(), getDistance());
        BallColor ballColor = balls.get(balls.size() - 1);
        switch (scanPhase) {
            case pre:
                if (color != BallColor.None){
                    scanPhase = ScanPhase.ball;
                    if (scanTimer.time() < Constants.PlacholdereDouble){ //This is SMALL
                        balls = Arrays.asList(balls.get(1), balls.get(2), color);
                    }
                    else if (scanTimer.time() < Constants.PlacholdereDouble){ // This is BIG
                        balls = Arrays.asList(balls.get(1), color);
                    } else {
                        balls = Arrays.asList(color);
                    }
                }
                break;
            case ball:
                if (color != ballColor){
                    scanPhase = ScanPhase.pre;
                    scanTimer.reset();
                }
                break;
        }
        return balls;
    }
    //ToDo: Loading
    public void setTransferPower(double set){
        if (lastTransferPower != set){
            lastTransferPower = set;
            transfer.setPower(set);
        }
    }

    public void transfer(boolean start){
        setTransferPower(start ? Constants.Indexer.transferPower : 0);
    }

    public void index(boolean start){
        setIndexerPower(start ? Constants.Indexer.spindexPower : 0);
    }

    public void stopIndexer(){
        setIndexerPower(0);
    }

    public void setMotor(double set) {
        transfer.setPower(set);
    }
}