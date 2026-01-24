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
    public IndexerEnumsButEvenNewerThisTime deleteme = IndexerEnumsButEvenNewerThisTime.intake1;

    boolean startShooting = false;
    boolean shooting = false;
    boolean dumbShootState1 = false;
    boolean smartShootStage1 = false;
    boolean smartShootStage2 = false;
    private boolean allowIntaking = true;
    private boolean doSpit = false;
    Timer spitTimer = new Timer();
    Timer feedTimer = new Timer();

    BallColor[] queuedBalls = new BallColor[]{BallColor.None, BallColor.None, BallColor.None};

    public SmartIndexerButEvenNewer(HardwareMap hardwareMap) {
        this(hardwareMap, new BallColor[]{BallColor.None, BallColor.None, BallColor.None}, 0);
    }

    public SmartIndexerButEvenNewer(HardwareMap hardwareMap, BallColor[] ballCells, double startIndexerPos) {
        spindexer1 = hardwareMap.get(ServoImplEx.class, "spindexer1");
        spindexer2 = hardwareMap.get(ServoImplEx.class, "spindexer2");
        spindexer1.setPwmRange(new PwmControl.PwmRange(500, 2500)); // probably the wrong way to do this but it works
        spindexer2.setPwmRange(new PwmControl.PwmRange(500, 2500));
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

    public BallColor[] getQueuedBalls() {
        return queuedBalls;
    }

    public double getEncoderPercentage() {
        return Math.abs(updatingEncoderPos / 4096.0) + encoderOffsetFromAuto;
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

    public BallColor removeFrontQueue() {
        if (!isQueuedBallsEmpty()) {
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


    public int findIndexWithColor(BallColor color) {
        return findIndexWithColor(color, true);
    }

    // returns -1 if force is false and there is not a color
    // returns  3 if force is true  and there is not a color
    public int findIndexWithColor(BallColor color, boolean force) {
        // searches in a special order
        int[] searchOrder;
        if (IndexerEnumsButEvenNewerThisTime.isAShootEnum(currentIndexerState)) {
            switch (currentIndexerState) {
                case shoot0:
                    searchOrder = new int[]{0, 2, 1};
                    break;
                case shoot1:
                    searchOrder = new int[]{1, 2, 0};
                    break;
                case shoot2:
                default:
                    searchOrder = new int[]{2, 0, 1};
            }

        } else {
            searchOrder = new int[]{2, 0, 1};
        }

        for (int loop = 0; loop < 3; loop++) {
            int i = searchOrder[loop];
            BallColor colorToCheck;
            colorToCheck = ballCells[i];
            if (colorToCheck == color) {
                return i;
            }
        }
        if (force) {
            return 2;
        } else {
            return -1;
        }
    }


    public void update(boolean intaking, boolean readyToShoot) {
        update(intaking, readyToShoot, false, false);
    }

    public void update(boolean intaking, boolean readyToShoot, boolean doSmartShoot, boolean isFarShot) {
        updatingEncoderPos = encoder.getCurrentPosition(); //updates this variable on tick so we are not calling multiple times in one tick

        // starts shooting if there is anything in queue
        if (!isQueuedBallsEmpty() && doSmartShoot && !shooting) {
            startShooting = true;
        }

        if (startShooting && readyToShoot) {
            shooting = true; // makes sure things don't run this loop in intake
        }

        if (spitTimer.getElapsedTimeSeconds() > Constants.Indexer.spitTime) {
            doSpit = false;
            spitTimer.resetTimer(2000000000);
        }

        intakeLogicUpdate(intaking);
        if (doSmartShoot) {
            smartShootLogicUpdate(readyToShoot);
        } else if (!isFarShot){
            dumbShootLogicUpdate(readyToShoot);
        } else {
            farShootingLogicUpdate(readyToShoot);
        }
    }
    boolean shotTimerStarted = false;
    boolean next = false;
    boolean hasShot = true;
    Timer shotTimer = new Timer();
    private final double SHOT_TIME = .1;

    public void farShootingLogicUpdate(boolean readyToShoot) {
        if (startShooting && readyToShoot) {
            startShooting = false;
            shotTimer.resetTimer();
            spinTransferWheel(true);
        }

        // once at highest full index, shoot while going back to intake
        if (shooting && isAtPosition() && !shotTimerStarted) {
            shotTimerStarted = true;
            allowIntaking = false;
        }
        if (!readyToShoot) {
            hasShot = true;
        }
        if (shooting && shotTimerStarted && shotTimer.getElapsedTimeSeconds() > SHOT_TIME && readyToShoot && isAtPosition() && hasShot) {
            switch (currentIndexerState) {
                case intake1:
                case intake2:
                case intake3:
                    setIndexerPos(IndexerEnumsButEvenNewerThisTime.shoot1);
                    hasShot = false;
                    break;
                case shoot1:
                    setIndexerPos(IndexerEnumsButEvenNewerThisTime.shoot0);
                    hasShot = false;
                    break;
                case shoot0:
                    setIndexerPos(IndexerEnumsButEvenNewerThisTime.shoot2);
                    hasShot = false;
                    break;
                case shoot2:
                    setIndexerPos(IndexerEnumsButEvenNewerThisTime.intake0);
                    break;
                case intake0: //todo: deal with this if this is true immediately
                    shooting = false;
                    spinTransferWheel(false);
                    allowIntaking = true;
                    shotTimerStarted = false;
                    clearBallCells();
                    break;
                default:
                    throw new RuntimeException("far shooting switch");
            }
            shotTimer.resetTimer();
        }
    }
    public void intakeLogicUpdate(boolean intaking) {
        if (intaking && !isHasBallsFull() && isAtPosition() && !shooting) {
            BallColor curColor = getColor();

            if (curColor != BallColor.None) {
                // moves and sets ball cells if it sees a color
                setBallCellAtIntakeToColor(curColor);
                int nextIndex = 0;
                while (nextIndex < 3) {
                    if (ballCells[nextIndex] == BallColor.None) {
                        break;
                    }
                    nextIndex++;
                }

                setIndexerPos(IndexerEnumsButEvenNewerThisTime.getEnum(nextIndex, false));
                if (nextIndex == 3) {
                    allowIntaking = false;
                    doSpit = true;
                    spitTimer.resetTimer();
                }
            }
        }
    }
    public void dumbShootLogicUpdate(boolean readyToShoot) {
        // move to highest full index
        if (startShooting && readyToShoot) {
            setIndexerPos(IndexerEnumsButEvenNewerThisTime.intake3);
            spinTransferWheel(true);

            startShooting = false;
        }

        // once at highest full index, shoot while going back to intake
        if (shooting && !dumbShootState1 && isAtPosition()) {
            setIndexerPos(IndexerEnumsButEvenNewerThisTime.intake0);
            dumbShootState1 = true;
        }

        // once back at intake, stop shooting
        if (dumbShootState1 && isAtPosition()) {
            allowIntaking = true;
            clearBallCells();
            spinTransferWheel(false);
            shooting = false;
            dumbShootState1 = false;
        }
    }
    public void smartShootLogicUpdate(boolean readyToShoot) {
        if (startShooting && readyToShoot) {
            deleteme = IndexerEnumsButEvenNewerThisTime.getEnum(findIndexWithColor(queuedBalls[0]), true);
            setIndexerPos(IndexerEnumsButEvenNewerThisTime.getEnum(findIndexWithColor(queuedBalls[0]), true));
            smartShootStage1 = true;
            startShooting = false;
        }
        if (shooting && smartShootStage1 && feedTimer.getElapsedTimeSeconds() > Constants.Indexer.smartShootSpacingSec && isAtPosition()) {
            spinTransferWheel(true);
            feedTimer.resetTimer();
            smartShootStage1 = false;
            smartShootStage2 = true;
        }
        if (shooting && smartShootStage2 && feedTimer.getElapsedTimeSeconds() > Constants.Indexer.smartFeedSec) {
            //removes the cell of the ball we shot
            spinTransferWheel(false);
            ballCells[findIndexWithColor(queuedBalls[0])] = BallColor.None;
            removeFrontQueue();
            if (isQueuedBallsEmpty()) {
                allowIntaking = true;
                // if no more queue then go to intake
                setIndexerPos(IndexerEnumsButEvenNewerThisTime.intake0);
                shooting = false;
                smartShootStage2 = false;
            } else {
                setIndexerPos(IndexerEnumsButEvenNewerThisTime.getEnum(findIndexWithColor(queuedBalls[0]), true));
                smartShootStage2 = false;
                smartShootStage1 = true;
            }
        }
    }

    public void stop() {
        setIndexerPos(getEncoderPercentage());
    }
}
