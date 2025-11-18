package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.ColorFunctions;

public class Indexer {
    // declaring motor variables
    CRServo spindexer1;
    CRServo spindexer2;
    DcMotorEx indexerToShooterMotor;
    RevColorSensorV3 colorSensor;

    public Indexer(HardwareMap hardwaremap){
        spindexer1 = hardwaremap.get(CRServo.class, "spindexer1");
        spindexer2 = hardwaremap.get(CRServo.class, "spindexer2");
        indexerToShooterMotor = hardwaremap.get(DcMotorEx.class, "spindexerPrimingMotor");
        colorSensor = hardwaremap.get(RevColorSensorV3.class, "spindexerColorSensor");
    }

    public void setSpindexerPower(double set){
        spindexer1.setPower(set);
        spindexer2.setPower(set);
    }

    public void setIndexerToShooterPower(double set){
        indexerToShooterMotor.setPower(set);
    }

    public NormalizedRGBA getColor(){
        return colorSensor.getNormalizedColors();
    }

    // Spins the indexer until the correct color is in front of the sensor then stops the spindexer
    public boolean spinTill(BallColor ballColor) {
        setSpindexerPower(Constants.Indexer.spindexPower);
        NormalizedRGBA currentColor = getColor();

        if (ColorFunctions.toColor(currentColor) == ballColor) {
            setSpindexerPower(0);
            return true;
        } else {
            return false;
        }
    }
    //ToDo: Loading
    public void ballToShooter(){

    }

    public void setServo(double set) {
        spindexer1.setPower(set);
        spindexer2.setPower(set);
    }

    public void setMotor(double set) {
        indexerToShooterMotor.setPower(set);
    }
}