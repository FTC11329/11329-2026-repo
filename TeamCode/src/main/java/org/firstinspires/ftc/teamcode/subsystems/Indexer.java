package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.ColorFunctions;

public class Indexer {
    // declaring motor variables
    CRServo spindexer1;
    CRServo spindexer2;
    DcMotorEx transfer;
    RevColorSensorV3 colorSensor;

    public Indexer(HardwareMap hardwaremap){
        spindexer1 = hardwaremap.get(CRServo.class, "spindexer1");
        spindexer2 = hardwaremap.get(CRServo.class, "spindexer2");
        spindexer1.setDirection(DcMotorSimple.Direction.FORWARD);
        spindexer2.setDirection(DcMotorSimple.Direction.FORWARD);

        transfer = hardwaremap.get(DcMotorEx.class, "transfer");
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setCurrentAlert(4, CurrentUnit.AMPS);

        colorSensor = hardwaremap.get(RevColorSensorV3.class, "spindexerColorSensor");
    }

    public void setIndexerPower(double set){
        spindexer1.setPower(set);
        spindexer2.setPower(set);
    }

    public void setIndexerToShooterPower(double set){
        transfer.setPower(set);
    }

    public NormalizedRGBA getColor(){
        return colorSensor.getNormalizedColors();
    }

    // Spins the indexer until the correct color is in front of the sensor then stops the spindexer
    public boolean spinUntil(BallColor ballColor) {
        setIndexerPower(Constants.Indexer.spindexPower);
        NormalizedRGBA currentColor = getColor();

        return ColorFunctions.toColor(currentColor) == ballColor;
    }
    //ToDo: Loading
    public void startTransfer(){
        setMotor(0.9);
    }

    public void stopTransfer(){
        setMotor(0);
    }

    public void startIndexer(){
        setIndexerPower(0.9);
    }

    public void stopIndexer(){
        setIndexerPower(0);
    }

    public void setMotor(double set) {
        transfer.setPower(set);
    }
}