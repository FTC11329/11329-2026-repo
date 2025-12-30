package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.ColorFunctions;
import org.firstinspires.ftc.teamcode.util.ScanPhase;

public class Indexer {
    // declaring motor variables
    CRServo spindexer1;
    CRServo spindexer2;
    DcMotorEx feeder;
    RevColorSensorV3 colorSensor;
    ScanPhase scanPhase = ScanPhase.pre;

    int scanIndex = 0;

    double lastIndexerPower;
    double lastTransferPower;
    Timer lastColorTime;

    public Indexer(HardwareMap hardwaremap){
        spindexer1 = hardwaremap.get(CRServo.class, "spindexer1");
        spindexer2 = hardwaremap.get(CRServo.class, "spindexer2");
        spindexer1.setDirection(DcMotorSimple.Direction.FORWARD);
        spindexer2.setDirection(DcMotorSimple.Direction.FORWARD);

        feeder = hardwaremap.get(DcMotorEx.class, "transfer");
        feeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feeder.setDirection(DcMotorSimple.Direction.REVERSE);
        feeder.setCurrentAlert(4, CurrentUnit.AMPS);

        colorSensor = hardwaremap.get(RevColorSensorV3.class, "spindexerColorSensor");
        lastColorTime = new Timer();
    }

    public void setIndexerPower(double set) {
        if (lastIndexerPower != set) {
            lastIndexerPower = set;
            spindexer1.setPower(set);
            spindexer2.setPower(set);
        }

    }

    public void setIndexerToShooterPower(double set){
        feeder.setPower(set);
    }

    public NormalizedRGBA getColorRGBA(){
        return colorSensor.getNormalizedColors();
    }
    public BallColor getColor(){
        return ColorFunctions.toColor(colorSensor.getNormalizedColors(), colorSensor.getDistance(DistanceUnit.INCH));
    }
    public double getDistance(){
        return colorSensor.getDistance(DistanceUnit.INCH);
    }

    // Spins the indexer until the correct color is in front of the sensor then stops the spindexer
    public boolean spinUntil(BallColor ballColor) {
        setIndexerPower(Constants.Indexer.scanningPower);
        NormalizedRGBA currentColor = getColorRGBA();
        double distance = getDistance();
        BallColor color = getColor();

        if (ballColor == BallColor.Any && color != BallColor.None) {
            return true;
        }
        return ColorFunctions.toColor(currentColor, distance) == ballColor;
    }

    BallColor lastColor = BallColor.None;

    boolean countedEmpty1 = false;
    boolean countedEmpty2 = false;
/*
    todo remove after 2nd comp
    // Inserts at index 0 and wraps size to 3 max
    private void insertAtFrontWrapped(List<BallColor> balls, BallColor color) {
        balls.add(0, color);
        if (balls.size() > 3) {
            balls.remove(3); // remove oldest
        }
    }

    // Doron if you want to go back to your code that is fine
    public List<BallColor> scanIndexer(List<BallColor> balls) {
        BallColor color = getColor();

        // === Sensor sees empty ===
        if (color == BallColor.None) {
            double t = lastColorTime.getElapsedTimeSeconds();

            // First empty slot after 0.8 sec
            if (t >= 0.8 && !countedEmpty1) {
                insertAtFrontWrapped(balls, BallColor.None);
                countedEmpty1 = true;
            }

            // Second empty slot after 1.6 sec
            if (t >= 1.6 && !countedEmpty2) {
                insertAtFrontWrapped(balls, BallColor.None);
                countedEmpty2 = true;
            }

            return balls;
        }

        // === Sensor sees real color ===
        lastColorTime.resetTimer();
        countedEmpty1 = false;
        countedEmpty2 = false;

        // New ball detected if color changed or last was empty
        if (color != lastColor) {
            insertAtFrontWrapped(balls, color);
        }

        return balls;
    }

//        Doron's way
//        BallColor ballColor = balls.isEmpty() ? BallColor.None : balls.get(balls.size() - 1);
//        switch (scanPhase) {
//            case pre:
//                if (color != BallColor.None){
//                    scanPhase = ScanPhase.ball;
//                    if (scanTimer.time() < Constants.Indexer.secondsFor1){ //This is SMALL
//                        balls = balls.size() >= 3 ?
//                                Arrays.asList(balls.get(1), balls.get(2), color) :
//                                (balls.size() >= 2 ? Arrays.asList(balls.get(1), color) : Arrays.asList(color));
//
//                    } else if (scanTimer.time() < Constants.Indexer.secondsFor2){ // This is BIG
//                        balls = balls.size() >= 2 ? Arrays.asList(balls.get(1), color) : Arrays.asList(color);
//                    } else {
//                        balls = Arrays.asList(color);
//                    }
//                }
//                break;
//            case ball:
//                if (color != ballColor){
//                    scanPhase = ScanPhase.hole;
//                    scanTimer.reset();
//                }
//                break;
//            case hole:
//                if (scanTimer.time() > 0.1) {
//                    if (color == BallColor.None) {
//                        scanPhase = ScanPhase.pre;
//                    } else {
//                        scanPhase = ScanPhase.ball;
//                    }
//
//                }
//                break;
//        }
//        return balls;
     */
    public void setTransferPower(double set){
        if (lastTransferPower != set){
            lastTransferPower = set;
            feeder.setPower(set);
        }
    }

    public void transfer(boolean start){
        setTransferPower(start ? Constants.Indexer.transferPower : 0);
    }

    public void spinIndexer(boolean start){
        setIndexerPower(start ? Constants.Indexer.spindexPower : 0);
    }

    public void stopIndexer(){
        setIndexerPower(0);
    }

    public void setMotor(double set) {
        feeder.setPower(set);
    }

    public void update(double distanceToGoal) {
        if (spindexer1.getPower() == 0) {
            return;
        }
        if (distanceToGoal > Constants.Indexer.farDistance) {
            setIndexerPower(Constants.Indexer.farSpindexPower);
        } else {
            setIndexerPower(Constants.Indexer.spindexPower);
        }
    }
}