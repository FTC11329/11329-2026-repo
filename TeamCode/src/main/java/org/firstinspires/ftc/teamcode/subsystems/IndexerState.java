package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.ColorFunctions;
import org.firstinspires.ftc.teamcode.util.IndexerEnumsNew;
import org.firstinspires.ftc.teamcode.util.SuperDuperPID;

import static java.lang.Math.PI;

public class IndexerState {

    private IndexerEnumsNew indexerPosition = IndexerEnumsNew.intake0;
    private BallColor[] ballCells;
    public SuperDuperPID pidfController;

    private DcMotorEx encoder;
    private CRServo spindexer1;
    private CRServo spindexer2;
    private RevColorSensorV3 colorSensor;
    public boolean atPosition = true;
    private int encoderOffset;
    private double power;

    public IndexerState(HardwareMap hardwareMap, BallColor[] ballCells, int startIndexerTicks) {
        this.ballCells = ballCells;
        spindexer1 = hardwareMap.get(CRServo.class, "spindexer1");
        spindexer2 = hardwareMap.get(CRServo.class, "spindexer2");
        spindexer1.setDirection(CRServo.Direction.FORWARD);
        spindexer2.setDirection(CRServo.Direction.REVERSE);

        encoder = hardwareMap.get(DcMotorEx.class, "intake");
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "spindexerColorSensor");

        pidfController = new SuperDuperPID(Constants.Indexer.pidfCoefficients);
        encoderOffset = startIndexerTicks;
    }

    public void moveToNearest(BallColor tarColor, boolean isAnIntakePosition) {
        // gets nearest index of
        double smallestDistance = Double.MAX_VALUE;
        int smallestIndex = -1;

        for (int index = 0; index < 3; index++) {
            if (ballCells[index] == tarColor || (tarColor == BallColor.Any && ballCells[index] != BallColor.None)) {
                IndexerEnumsNew targetEnum = IndexerEnumsNew.getEnum(index, isAnIntakePosition);

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

        IndexerEnumsNew targetEnum = IndexerEnumsNew.getEnum(smallestIndex, isAnIntakePosition);
        setIndexerTarget(targetEnum);
    }


    public double findSmallestAngleToIndex(double targetAngle) {
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

    public double convertEnumToAbsoluteAngle(IndexerEnumsNew indexEnum) {
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
    public double enumToTicks(IndexerEnumsNew targetIndexerEnum) {
        return encoder.getCurrentPosition() + ((4096.0 / (2 * PI)) * (findSmallestAngleToIndex(convertEnumToAbsoluteAngle(targetIndexerEnum))));
    }
    public void setIndexerTarget(IndexerEnumsNew targetIndexerEnum) {
        if (targetIndexerEnum != indexerPosition) {
            atPosition = false;
            indexerPosition = targetIndexerEnum;

            double targetTicks = enumToTicks(targetIndexerEnum);

            pidfController.setTargetPosition(targetTicks);
        }
    }

    public int getEncoderTicks() {
        return encoder.getCurrentPosition() + encoderOffset;
    }

    public void update() {
        pidfController.update(getEncoderTicks());
        power = pidfController.run();

        spindexer1.setPower(power);
        spindexer2.setPower(power);

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
    private static final long UPDATE_PERIOD_MS = 100;
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

    public IndexerEnumsNew getIndexerPosition() {
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

    public void setIndexerPosition(IndexerEnumsNew indexerPosition) {
        this.indexerPosition = indexerPosition;
    }

    public double getPower(){return power;}

}
