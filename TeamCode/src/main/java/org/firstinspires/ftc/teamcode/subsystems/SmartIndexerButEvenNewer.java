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
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.ColorFunctions;
import org.firstinspires.ftc.teamcode.util.IndexerEnumsButEvenNewerThisTime;

public class SmartIndexerButEvenNewer {

    ServoImplEx spindexer1;
    ServoImplEx spindexer2;
    DcMotorEx feeder;
    DcMotorEx encoder;
    RevColorSensorV3 colorSensor;

    BallColor[] ballCells = new BallColor[3];
    private BallColor transferColor = BallColor.None;

    public IndexerEnumsButEvenNewerThisTime currentIndexerState = IndexerEnumsButEvenNewerThisTime.intake0;
    public double lastIndexerTarget = 0.676767676767676767676767676767676767676767676767676767676767676767676767676767676767676767676767676767676767;
    double lastTransferPower;
    double encoderOffsetFromAuto = 0;
    int updatingEncoderPos;

    boolean startShooting = false;
    boolean shooting = false;
    boolean endShootingNext = false;
    private boolean allowIntaking = true;
    private boolean doSpit = false;
    Timer spitTimer = new Timer();

    BallColor[] queuedBalls = new BallColor[]{BallColor.None, BallColor.None, BallColor.None};

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
        spitTimer.resetTimer(10000000);
//        spindexer1.setPosition(0);
//        spindexer2.setPosition(0);
        setIndexerPos(IndexerEnumsButEvenNewerThisTime.intake0);


        feeder = hardwareMap.get(DcMotorEx.class, "transfer");
        feeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feeder.setDirection(DcMotorSimple.Direction.REVERSE);

        encoder = hardwareMap.get(DcMotorEx.class, "intake");
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderOffsetFromAuto = startIndexerPos;

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "spindexerColorSensor");
        this.ballCells = ballCells;
    }

    public void start() {
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // DO NOT USE THESE FUNCTIONS TO MOVE THE INDEXER DURING NORMAL OPERATION. USE UPDATE() INSTEAD
    private void setIndexerPos(IndexerEnumsButEvenNewerThisTime indexerEnum) {
        currentIndexerState = indexerEnum;
        setIndexerPos(IndexerEnumsButEvenNewerThisTime.convertEnumToPercentOfRot(indexerEnum));
    }
    private void setIndexerPos(double set) {
        if (lastIndexerTarget != set) {
            lastIndexerTarget = set;
            spindexer1.setPosition(set);
            spindexer2.setPosition(set);
        }
    }
    private void setFeederPower(double set) {
        if (lastTransferPower != set) {
            lastTransferPower = set;
            feeder.setPower(set);
        }
    }

    private void spinTransferWheel(boolean set) {
        setFeederPower(set ? Constants.Indexer.transferPower : 0);
    }

    BallColor[] lastColors = new BallColor[]{BallColor.None, BallColor.None, BallColor.None};
    public boolean allowIntaking() {
        return allowIntaking;
    }
    public boolean doSpit() {
        return doSpit;
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
            case intake3:
                ballCells[0] = ballColor;
            default:
                throw new RuntimeException("shouldnt error, set ball cells");

        }
    }

    public BallColor[] getBallCells() {
        return ballCells;
    }

    public double getEncoderPercentage() {
        return (updatingEncoderPos / 4096.0) + encoderOffsetFromAuto;
    }

    public boolean isAtPosition() {
        return (Math.abs(getEncoderPercentage() - lastIndexerTarget) < Constants.Indexer.indexerTolerance);
    }

    public void shootAll() {
        if (!shooting) {
            startShooting = true;
        }
    }

    // functions to shoot specific colors *********************************************************~

    public boolean isQueuedBallsFull() {
        for (BallColor color : queuedBalls) {
            if (color == BallColor.None) {
                return false;
            }
        }
        return true;
    }

    public boolean isQueuedBallsEmpty() {
        for (BallColor color : queuedBalls) {
            if (color != BallColor.None) {
                return false;
            }
        }
        return true;
    }

    // return 3 if it is full
    public int highestEmptyQueueIndex() {
        int i = 0;
        for (BallColor color : queuedBalls) {
            if (color == BallColor.None) {
                return i;
            }
            i++;
        }
        return 3;
    }

    public void setQueuedBalls(BallColor[] queuedBalls) {
        this.queuedBalls = queuedBalls;
    }

    public void addToQueue(BallColor queueBall) {
        if (!isQueuedBallsFull()) {
            queuedBalls[highestEmptyQueueIndex()] = queueBall;
        }
    }

    public BallColor removeBottomQueue() {
        if (!isHasBallsEmpty()) {
            BallColor removedColor = queuedBalls[0];

            int highestFullIndex = highestEmptyQueueIndex() - 1;
            BallColor[] newList = new BallColor[]{BallColor.None, BallColor.None, BallColor.None};

            for (int i = 0; i < highestFullIndex; i++) {
                newList[i] = queuedBalls[i+1];
            }
            queuedBalls = newList.clone();
            return removedColor;
        }
        return BallColor.None;
    }



    public void update(boolean intaking, boolean readyToShoot) {
        updatingEncoderPos = encoder.getCurrentPosition(); //updates this variable on tick so we are not calling multiple times in one tick

        if (startShooting && readyToShoot) {
            shooting = true; // makes sure things don't run this loop
        }

        if (spitTimer.getElapsedTimeSeconds() > 0.25) {
            doSpit = false;
            spitTimer.resetTimer(10000000);
        }

        if (intaking && !isHasBallsFull() && isAtPosition() && !shooting) {
            BallColor curColor = getColor();

            if (curColor != BallColor.None) {
                // moves and sets ball cells if it sees a color
                setBallCellAtIntakeToColor(curColor);

                int nextIndex = IndexerEnumsButEvenNewerThisTime.getIndex(currentIndexerState) + 1;
                setIndexerPos(IndexerEnumsButEvenNewerThisTime.getEnum(nextIndex));
                if (nextIndex == 3) {
                    allowIntaking = false;
                    doSpit = true;
                    spitTimer.resetTimer();
                }
            }
        }

        // move to highest full index
        if (startShooting && readyToShoot) {
            setIndexerPos(IndexerEnumsButEvenNewerThisTime.intake3);
            spinTransferWheel(true);

            startShooting = false;
            shooting = true;
        }

        // once at highest full index, shoot while going back to intake
        if (shooting && !endShootingNext && isAtPosition()) {
            setIndexerPos(IndexerEnumsButEvenNewerThisTime.intake0);
            endShootingNext = true;
        }

        // once back at intake, stop shooting
        if (endShootingNext && isAtPosition()) {
            allowIntaking = true;
            clearBallCells();
            shooting = false;
            endShootingNext = false;
            spinTransferWheel(false);
        }
    }

    

    public void stop() {
        setIndexerPos(getEncoderPercentage());
    }
}
