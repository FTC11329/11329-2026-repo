package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.ColorFunctions;
import org.firstinspires.ftc.teamcode.util.IndexerEnums;
import org.firstinspires.ftc.teamcode.util.SuperDuperPID;

import static java.lang.Math.PI;
import static java.lang.Math.pow;

public class IndexerState {

    private IndexerEnums indexerPosition = IndexerEnums.intake0;

    public int encoderTickPos = 0;
    private BallColor[] ballCells;
    public SuperDuperPID pidfController;

    private DcMotorEx encoder;
    private CRServo spindexer1;
    private CRServo spindexer2;
    private RevColorSensorV3 colorSensor;
    public boolean atPosition = true;
    private int encoderOffset;
    private double power;
    public boolean stopIndexer = false;

    long now = System.currentTimeMillis();
    public IndexerState(HardwareMap hardwareMap, BallColor[] ballCells, int startIndexerTicks) {
        this.ballCells = ballCells;
        spindexer1 = hardwareMap.get(CRServo.class, "spindexer1");
        spindexer2 = hardwareMap.get(CRServo.class, "spindexer2");
        spindexer1.setDirection(CRServo.Direction.REVERSE);
        spindexer2.setDirection(CRServo.Direction.FORWARD);

        encoder = hardwareMap.get(DcMotorEx.class, "intake");
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "spindexerColorSensor");

        pidfController = new SuperDuperPID();
        encoderOffset = startIndexerTicks;
    }

    static final int STEP_TICKS = 1366 * 3;

    long moveStartTime = 0;
    double totalTimeMs = 0;
    int samples = 0;

    double encoderTarget = 682.6;
    double avgTimeSec;
    boolean moving = false;
    boolean first = true;

    public void averageTime() {
        long now = System.currentTimeMillis();
        double pos = encoder.getCurrentPosition();

        pidfController.update(pos);
        double power = pidfController.run();

        if (!moving) {
            // Start new move
            encoderTarget = pos + STEP_TICKS;
            pidfController.setTargetPosition(encoderTarget);
            moveStartTime = now;
            moving = true;
        }

        // Movement finished
        if (moving && Math.abs(pos - encoderTarget) < Constants.Indexer.indexerTolerance && power == 0 /*&& Math.abs(pidfController.getVelocity()) < 20 */) {
            long moveTime = now - moveStartTime;
            totalTimeMs += moveTime;
            samples++;

            avgTimeSec = (totalTimeMs / samples) / 1000.0;

            moving = false;
        }

        spindexer1.setPower(power);
        spindexer2.setPower(power);
    }

    long lastStepTime = 0;
    boolean high = false;
    long timer = -1;
    double target;

    public void stepMovement() {
        long now = System.currentTimeMillis();

        if (now - lastStepTime >= 3000) {
            high = !high;
            lastStepTime = now;

            pidfController.setTargetPosition(target+=STEP_TICKS);
        }

        pidfController.update(encoder.getCurrentPosition());
        double power = pidfController.run();

        spindexer1.setPower(power);
        spindexer2.setPower(power);
    }

    public void moveToNearest(BallColor tarColor, boolean isAnIntakePosition) {
        // gets nearest index of
        double smallestDistance = Double.MAX_VALUE;
        int smallestIndex = -1;

        for (int index = 0; index < 3; index++) {
            if (ballCells[index] == tarColor || (tarColor == BallColor.Any && ballCells[index] != BallColor.None)) {
                IndexerEnums targetEnum = IndexerEnums.getEnum(index, isAnIntakePosition);

                double targetAngle = convertEnumToAbsoluteAngle(targetEnum);

                double distanceFromAngleToTarget = Math.abs(findSmallestAngleToIndex(targetAngle));

                if (distanceFromAngleToTarget < smallestDistance) {
                    smallestDistance = distanceFromAngleToTarget;
                    smallestIndex = index;
                }
            }
        }
        if (smallestIndex == -1) {
            throw new RuntimeException("moveToNearest ball empty");
        }

        IndexerEnums targetEnum = IndexerEnums.getEnum(smallestIndex, isAnIntakePosition);
        setIndexerTarget(targetEnum);
    }


    public double findSmallestAngleToIndex(double targetAngle) { //todo: always minimize to intake
        return ((targetAngle - getAbsoluteEncoderAngle() + PI) % (2 * PI)) - PI;
    }


    public double getAbsoluteEncoderAngle() {
        return getContinuousEncoderAngle() % (2 * PI);
//        double angle = getContinuousEncoderAngle();
//        if (angle >= 0) {
//            return angle % (2 * PI);
//        } else {
//            return (2 * PI) + (angle % (2 * PI));
//        }
    }

    public double getContinuousEncoderAngle() {
        return getEncoderTicks() / 4096.0 * (2 * PI);
    }

    public double convertEnumToAbsoluteAngle(IndexerEnums indexEnum) {
        switch (indexEnum) {
            case shoot0:
                return PI;
            case shoot1:
                return PI / 3;
            case shoot2:
                return 5 * PI / 3;
            case intake0:
                return 0;
            case intake1:
                return 4 * PI / 3;
            case intake2:
                return 2 * PI / 3;
        }
        return 0;
    }
    public double enumToTicks(IndexerEnums targetIndexerEnum) {
        return pidfController.getTargetPosition() + ((4096.0 / (2 * PI)) * (findSmallestAngleToIndex(convertEnumToAbsoluteAngle(targetIndexerEnum))));
    }
    public void setIndexerTarget(IndexerEnums targetIndexerEnum) {
        if (targetIndexerEnum != indexerPosition) {
            atPosition = false;
            indexerPosition = targetIndexerEnum;

            double targetTicks = enumToTicks(targetIndexerEnum);

            pidfController.setTargetPosition(targetTicks);
        }
    }

    public int getEncoderTicks() {
        return encoderTickPos + encoderOffset;
    }

    public void update() {
        encoderTickPos = encoder.getCurrentPosition();
        pidfController.update(getEncoderTicks());
        power = pidfController.run();

        if (!stopIndexer) {
            spindexer1.setPower(power);
            spindexer2.setPower(power);
        }

        if (Math.abs(pidfController.getError()) < Constants.Indexer.indexerTolerance) {
            atPosition = true;
        }
    }

    public boolean isBallCellsFull() {
        for (BallColor color : ballCells) {
            if (color == BallColor.None) {
                return false;
            }
        }
        return true;
    }

    public boolean isBallCellsEmpty() {
        for (BallColor color : ballCells) {
            if (color != BallColor.None) {
                return false;
            }
        }
        return true;
    }

    public BallColor[] getBallCells() {
        return ballCells;
    }
    private static final long UPDATE_PERIOD_MS = 0;
    private long lastUpdateTime = 0;
    private BallColor cachedColor = BallColor.None;
    // this limits it to checking the intake for a ball to every 100ms for faster performance
    public BallColor getColor() {
        long now = System.currentTimeMillis();

        if (now - lastUpdateTime >= UPDATE_PERIOD_MS) {
            lastUpdateTime = now;
            if (atPosition) {
                cachedColor = ColorFunctions.toColor(getColorRGBA(), getDistance());
            } else {
                cachedColor = BallColor.None;
            }
        }

        return cachedColor;
    }



    public double getDistance() {
        return colorSensor.getDistance(DistanceUnit.INCH);
    }

    public NormalizedRGBA getColorRGBA() {
        return colorSensor.getNormalizedColors();
    }

    public IndexerEnums getIndexerPosition() {
        return indexerPosition;
    }

    public void setBallCells(BallColor[] ballCells) {
        this.ballCells = ballCells;
    }

    public void setBallCellAtIntakeToRightColor() {
        switch (indexerPosition) {
            case intake0:
                ballCells[0] = getColor();
                break;
            case intake1:
                ballCells[1] = getColor();
                break;
            case intake2:
                ballCells[2] = getColor();
                break;
            default:
                throw new RuntimeException("tried to add a ball while not at intake position");

        }
    }

    public void removeBallAtIndex(int index) {
        ballCells[index] = BallColor.None;
    }

    public void spinIndexerOnce() {
        pidfController.update(getEncoderTicks());
        atPosition = false;
        encoderOffset += 4096;
    }

    public void stopIndexer(boolean set) {
        if (set) {
            stopIndexer = true;
            spindexer1.setPower(0);
            spindexer2.setPower(0);
        } else {
            stopIndexer = false;
        }
    }

    public void setIndexerPosition(IndexerEnums indexerPosition) {
        this.indexerPosition = indexerPosition;
    }

    public double getPower(){return power;}

}
