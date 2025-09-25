package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptAprilTag;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.ColorFunctions;

public class Indexer {
    // declaring motor variables
    DcMotorEx spindexer;
    CRServo indexerToShooter;
    RevColorSensorV3 colorSensor;

    public Indexer(HardwareMap hardwaremap){
        spindexer = hardwaremap.get(DcMotorEx.class, "spindexer");
        indexerToShooter = hardwaremap.get(CRServo.class, "spindexerServo");
        colorSensor = hardwaremap.get(RevColorSensorV3.class, "spindexerColorSensor");
    }

    public void setSpindexerPower(double set){
        spindexer.setPower(set);
    }
    public void moveToShooterPower(double set){
        indexerToShooter.setPower(set);
    }

    public NormalizedRGBA getColor(){
        return colorSensor.getNormalizedColors();
    }

    // Spins the indexer until the correct color is in front of the sensor then stops the spindexer
    public boolean SpinTill(BallColor ballColor) {
        setSpindexerPower(Constants.Indexer.spindexPower);
        NormalizedRGBA currentColor = getColor();

        if (ColorFunctions.toColor(currentColor) == ballColor) {
            setSpindexerPower(0);
            return true;
        } else {
            return false;
        }
    }
}