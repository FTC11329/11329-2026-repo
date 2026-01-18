package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.ColorFunctions;
import org.firstinspires.ftc.teamcode.util.IndexerEnums;

public class SmartIndexerButEvenNewer {

    ServoImplEx spindexer1;
    ServoImplEx spindexer2;
    DcMotorEx feeder;
    DcMotorEx encoder;
    RevColorSensorV3 colorSensor;

    BallColor[] ballCells = new BallColor[3];
    private BallColor transferColor = BallColor.None;

    public IndexerEnums currentIndexerState = IndexerEnums.intake0;
    public double lastIndexerPos = 0.67676767; // not actually the start pos
    double lastTransferPower;
    int updatingEncoderPos;

    boolean startShooting = false;
    boolean shooting = false;
    boolean endShootingNext = false;
    boolean endShootingNextSecond = false;
    boolean allowIntaking = true;

    int queuedFirst = -1;
    int queuedSecond = -1;
    int queuedThird = -1;
    public SmartIndexerButEvenNewer(HardwareMap hardwareMap) {
        this(hardwareMap, new BallColor[]{BallColor.None, BallColor.None, BallColor.None}, 0);
    }

    public SmartIndexerButEvenNewer(HardwareMap hardwareMap, BallColor[] ballCells, double startIndexerPos) {
        spindexer1 = hardwareMap.get(ServoImplEx.class, "spindexer1");
        spindexer2 = hardwareMap.get(ServoImplEx.class, "spindexer2");
        spindexer1.setPwmRange(new PwmControl.PwmRange(542, 2450)); // probably the wrong way to do this but it works
        spindexer2.setPwmRange(new PwmControl.PwmRange(542, 2450));
        spindexer1.setDirection(Servo.Direction.REVERSE);
        spindexer2.setDirection(Servo.Direction.REVERSE);
        setIndexerPos(IndexerEnums.intake0);


        feeder = hardwareMap.get(DcMotorEx.class, "transfer");
        feeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feeder.setDirection(DcMotorSimple.Direction.REVERSE);

        encoder = hardwareMap.get(DcMotorEx.class, "intake");
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "spindexerColorSensor");
        this.ballCells = ballCells;
    }

    // DO NOT USE THESE FUNCTIONS TO MOVE THE INDEXER DURING NORMAL OPERATION. USE UPDATE() INSTEAD
    private void setIndexerPos(IndexerEnums indexerEnum) {
        currentIndexerState = indexerEnum;
        setIndexerPos(convertEnumToPercentOfRot(indexerEnum));
    }
    private void setIndexerPos(double set) {
        if (lastIndexerPos != set) {
            lastIndexerPos = set;
            spindexer1.setPosition(set);
            spindexer2.setPosition(set);
        }
    }
    private void setIndexerToShooterPower(double set) {
        if (lastTransferPower != set) {
            lastTransferPower = set;
            feeder.setPower(set);
        }
    }

    private void spinTransferWheel(boolean set) {
        setIndexerToShooterPower(set ? Constants.Indexer.transferPower : 0);
    }

    public boolean allowIntaking() {
        return allowIntaking;
    }

    public NormalizedRGBA getColorRGBA(){
        return colorSensor.getNormalizedColors();
    }

    public double getDistance(){
        return colorSensor.getDistance(DistanceUnit.INCH);
    }

    public BallColor getColor(){
        return ColorFunctions.toColor(getColorRGBA(), getDistance());
    }
    public boolean isHasBallsFull() {
        for (BallColor color : ballCells) {
            if (color == BallColor.None) {
                return false;
            }
        }
        return true;
    }

    public boolean isHasBallsEmpty() {
        for (BallColor color : ballCells) {
            if (color != BallColor.None) {
                return false;
            }
        }
        return true;
    }

    private void clearBallCells() {
        ballCells[0] = BallColor.None;
        ballCells[1] = BallColor.None;
        ballCells[2] = BallColor.None;
    }

    public double convertEnumToPercentOfRot(IndexerEnums indexerPos) {
        switch (indexerPos) {
            case shoot0:
                return 0;
            case shoot1:
                return 0.3333;
            case shoot2:
                return 0.6667;
            case intake0:
                return 0.5;
            case intake1:
                return 0.8333;
            case intake2:
                return 0.1667;
        }
        throw new RuntimeException("Shouldn't error ever, in the indexer");
    }

    public void setBallCellAtIntakeToColor(BallColor ballColor) {
        switch (currentIndexerState) {
            case intake0:
                ballCells[0] = ballColor;
                break;
            case intake1:
                ballCells[1] = ballColor;
                break;
            case intake2:
                ballCells[2] = ballColor;
                break;
            default:
                throw new RuntimeException("tried to add a ball while not at intake position");

        }
    }

    public BallColor[] getBallCells() {
        return ballCells;
    }

    public double getEncoderPercentage() {
        return (updatingEncoderPos / 4096.0) + 0.5;
    }

    public boolean isAtPosition() {
        return (Math.abs(getEncoderPercentage() - lastIndexerPos) < 0.02);
    }

    public void shootAll() {
        if (!shooting) {
            startShooting = true;
        }
    }


    public void update(boolean intaking, boolean readyToShoot) {
        updatingEncoderPos = encoder.getCurrentPosition(); //updates this variable on tick so we are not calling multiple times in one tick

        if (startShooting && readyToShoot) { 
            shooting = true; // makes sure things don't run this loop
        }

        BallColor curColor = intaking && !shooting ? getColor() : BallColor.None;

        if (isAtPosition() && !IndexerEnums.isAShootEnum(currentIndexerState) && !shooting && curColor != BallColor.None) {
            setBallCellAtIntakeToColor(curColor);

            int nextIndex = IndexerEnums.getIndex(currentIndexerState) + 1;

            if (nextIndex <= 2) {
                setIndexerPos(IndexerEnums.getEnum(nextIndex, true));
            } else {
                // out of bounds, meaning ball cells is full
            }
        }

        // move to highest full index
        if (startShooting && readyToShoot) {
            switch (currentIndexerState) {
                case intake0:
                    // intake 0 goes to shoot 1, 2; intake 0
                    queuedFirst = 1;
                    queuedSecond = 2;
                    break;
                case intake1:
                    // intake 1 goes to shoot 0; intake 0
                    queuedFirst = 3;
                    queuedSecond = -1;
                    break;
                case intake2:
                    // intake 2 goes to shoot 0 reverse; intake 0
                    queuedFirst = 3;
                    queuedSecond = -1;
                    break;

            }
            spinTransferWheel(true);

            if (queuedFirst == 3) {
                setIndexerPos(1);
            } else {
                setIndexerPos(IndexerEnums.getEnum(queuedFirst, false));
            }
            startShooting = false;
            shooting = true;
        }

        // once at highest full index, shoot while going back to intake
        if (shooting && !endShootingNext && isAtPosition()) {
            if (queuedSecond != -1) {
                setIndexerPos(IndexerEnums.getEnum(queuedSecond, false));
            }
            endShootingNext = true;
        }

        // once back at intaking, stop shooting
        if (endShootingNext && isAtPosition()) {
            endShootingNext = false;
            endShootingNextSecond = true;
            setIndexerPos(IndexerEnums.intake0);
        }

        if (endShootingNextSecond && isAtPosition()) {
            clearBallCells();
            shooting = false;
            endShootingNextSecond = false;
            spinTransferWheel(false);

        }
        if (startShooting) {startShooting = false;}
    }

    public void stop() {
        setIndexerPos(getEncoderPercentage());
    }
}
