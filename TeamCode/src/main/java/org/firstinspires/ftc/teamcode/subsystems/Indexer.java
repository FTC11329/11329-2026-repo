package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.ColorFunctions;
import org.firstinspires.ftc.teamcode.util.FieldShapes;
import org.firstinspires.ftc.teamcode.util.IndexerEnums;
import org.firstinspires.ftc.teamcode.util.ShapeDetection;
import org.firstinspires.ftc.teamcode.util.SmartShootState;


public class Indexer {

    ServoImplEx spindexer1;
    public ServoImplEx spindexer2;
    DcMotorEx feeder;
    DcMotorEx encoder;
    AnalogInput analog2;
    AnalogInput analog3;
    RevColorSensorV3 colorSensor;


    BallColor[] ballCells = new BallColor[3];
    private BallColor transferColor = BallColor.None;

    public IndexerEnums currentIndexerState = IndexerEnums.intake0;
    public double lastIndexerTarget = 0.676767676767676767676767676767676767676767676767676767676767676767676767676767676767676767676767676767676767;
    double lastTransferPower;
    boolean spinTransferWheelVariable = false;
    boolean beenInited = false;
    public double encoderOffsetFromAuto = 0;
    int updatingEncoderPos;
    boolean startShooting = false;
    public boolean shooting = false;
    boolean unjam = false;
    boolean dumbShootState1 = false;
    private boolean allowIntaking = true;
    private boolean forceEndPlug = false;
    private boolean doSpit = false;
    Timer spitTimer = new Timer();
    Timer feedTimer = new Timer();
    public boolean startIndexerPlug = false;
    public boolean indexerPlug = false;
    private Pose lastPosition = new Pose(0,0,0);

    BallColor[] queuedBalls = new BallColor[]{BallColor.None, BallColor.None, BallColor.None};

    public Indexer(HardwareMap hardwareMap) {
        this(hardwareMap, new BallColor[]{BallColor.None, BallColor.None, BallColor.None}, 0);
    }

    public Indexer(HardwareMap hardwareMap, BallColor[] ballCells, double startIndexerPos) {
        encoder = hardwareMap.get(DcMotorEx.class, "intake");
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderOffsetFromAuto = startIndexerPos;

        spindexer1 = hardwareMap.get(ServoImplEx.class, "spindexer1");
        spindexer2 = hardwareMap.get(ServoImplEx.class, "spindexer2");
        spindexer1.setPwmRange(new PwmControl.PwmRange(500, 2500));
        spindexer2.setPwmRange(new PwmControl.PwmRange(500, 2500));
        spindexer1.setDirection(Servo.Direction.REVERSE);
        spindexer2.setDirection(Servo.Direction.REVERSE);
        spitTimer.resetTimer(10000000);
        setIndexerPos(encoderOffsetFromAuto);


        feeder = hardwareMap.get(DcMotorEx.class, "transfer");
        feeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feeder.setDirection(DcMotorSimple.Direction.REVERSE);

        analog2 = hardwareMap.get(AnalogInput.class, "spindexerAnalog2");
        analog3 = hardwareMap.get(AnalogInput.class, "spindexerAnalog3");

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "Color");

        setHasBalls(ballCells);
    }

    public void start() {
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // DO NOT USE THESE FUNCTIONS TO MOVE THE INDEXER DURING NORMAL OPERATION. USE UPDATE() INSTEAD
    public void setIndexerPos(IndexerEnums indexerEnum) {
        currentIndexerState = indexerEnum;
        setIndexerPos(IndexerEnums.convertEnumToPercentOfRot(indexerEnum));
    }
    public void setIndexerPos(double set) {
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
        spinTransferWheelVariable = set;
    }

    // because we will want to change power in between calls to the function above
    private void spinTransferWheelUpdate() {
        if (spinTransferWheelVariable) {
            setFeederPower(Constants.Indexer.transferPower);
        } else {
            setFeederPower(isAtPosition() ? 0 : -0.2);
        }
    }

    BallColor[] lastColors = new BallColor[]{BallColor.None, BallColor.None, BallColor.None};
    public boolean allowIntaking() {
        return allowIntaking;
    }
    public boolean doSpit() {
        return doSpit;
    }

    // DO NOT RUN IN TELEMETRY
    boolean lastGreen = false;
    public boolean isGreen() {
        boolean temp = lastGreen;
        lastGreen = analog2.getVoltage() > 2.9;
        return temp && lastGreen;
    }
    public void reZeroIndexer() {
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderOffsetFromAuto = .833;
    }

    // DO NOT RUN IN TELEMETRY
    boolean lastPurple = false;
    public boolean isPurple() {
        boolean temp = lastPurple;
        lastPurple = analog3.getVoltage() > 2.9;
        return temp && lastPurple;
    }
    public boolean isGreenTelem() {
        return analog2.getVoltage() > 2.9;
    }

    // DO NOT RUN IN TELEMETRY
    public boolean isPurpTelem() {
        return analog3.getVoltage() > 2.9;
    }

//    public BallColor getColor(){
//        if (isGreen()) {
//            return BallColor.Green;
//        } else if (isPurple()) {
//            return BallColor.Purple;
//        } else {
//            return BallColor.None;
//        }
//    }

    BallColor lastColor = BallColor.None;
    public BallColor getColor(){
        BallColor thisColor = ColorFunctions.toColor(colorSensor.getNormalizedColors(), colorSensor.getDistance(DistanceUnit.INCH));
        BallColor returnColor;
        if (thisColor == lastColor) {
            returnColor = thisColor;
        } else {
            returnColor = BallColor.None;
        }
        lastColor = thisColor;
        return returnColor;
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

    public int numberOfBallsInBallCells() {
        int num = 0;
        for (BallColor color : ballCells) {
            if (color != BallColor.None) {
                num++;
            }
        }
        return num;
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

    public void updateEncoder() {
        updatingEncoderPos = -encoder.getCurrentPosition();
    }

    public double getEncoderPercentage() {
        return (updatingEncoderPos / 4096.0) + encoderOffsetFromAuto;
    }

    public boolean isAtPosition(boolean wideTolerance) {
        double tol;
        tol = wideTolerance ? Constants.Indexer.wideIndexerTolerance : Constants.Indexer.indexerTolerance;
        return (Math.abs(getEncoderPercentage() - lastIndexerTarget) < tol);
    }
    public boolean isAtPosition() {
        return isAtPosition(false);
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

    public void emptyQueue() {
        queuedBalls[0] = BallColor.None;
        queuedBalls[1] = BallColor.None;
        queuedBalls[2] = BallColor.None;
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


    // returns the location of any color if the correct is not found
    // else returns 2
    public int findIndexWithColor(BallColor color) {
        // searches in a special order
        int[] searchOrder;
        if (IndexerEnums.isAShootEnum(currentIndexerState)) {
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
            if (colorToCheck == color || (colorToCheck != BallColor.None && color == BallColor.Any)) {
                return i;
            }
        }
        if (color != BallColor.Any) {
            return findIndexWithColor(BallColor.Any);
        } else {
            return 2;
        }
    }

    public void unjam() {
        if (!unjam) {
            unjam = true;
            unjamCounter = 0;
        }
    }
    long lastStuck;
    public boolean isStuck = false;
    public void stuckUpdate() {
        if (isAtPosition()) {
            lastStuck = System.nanoTime();
        } else if ((lastStuck - System.nanoTime()) * 1e-9 > .5) {
            isStuck = true;
        } else {
            isStuck = false;
        }
    }
    //Update **************************************************************************************~
    public void update(boolean intaking, boolean readyToShoot, Pose currentPose) {
        update(intaking, readyToShoot, false, false, currentPose);
    }

    public void update(boolean intaking, boolean readyToShoot, boolean doSmartShoot, boolean isFarShot, Pose currentPose) {
        stuckUpdate();

        updateEncoder(); //updates this variable on tick so we are not calling multiple times in one tick
        // Stops if unjamming
        if (unjam) {
            unjamUpdate();
            spinTransferWheelUpdate();
            return;
        }

        if (startShooting && readyToShoot) {
            shooting = true; // makes sure things don't run this loop in intake
        }

        // Plugging (https://www.youtube.com/@ftc11329)
        if (startIndexerPlug && !indexerPlug) {
            lastPosition = currentPose;
            setIndexerPos(IndexerEnums.shoot0);
            startIndexerPlug = false;
            indexerPlug = true;
        }

        if (indexerPlug && (ShapeDetection.isRobotInside(FieldShapes.farTriangle, currentPose) || ShapeDetection.isRobotInside(FieldShapes.closeTriangle, currentPose))) {
            setIndexerPos(IndexerEnums.intake2);
            indexerPlug = false;
        }

        if (indexerPlug) {
            if (currentPose.distanceFrom(lastPosition) > Constants.Indexer.indexerPlugDistance || forceEndPlug) {
                setIndexerPos(IndexerEnums.intake2);
                indexerPlug = false;
                forceEndPlug = false;
            }
        } else if (forceEndPlug) {
            forceEndPlug = false;
        }

        // Bulk Runs
        intakeLogicUpdate(intaking, readyToShoot);
        if (doSmartShoot) {
            smartShootLogicUpdate(readyToShoot);
        } else {
            dumbShootLogicUpdate(readyToShoot);
        }
        if (isHasBallsFull() && readyToShoot && isAtPosition() && !doSmartShoot) {
            spinTransferWheel(true);
        }
        spinTransferWheelUpdate();
    }
    public boolean isPlugged() {
        return indexerPlug;
    }
    public void forceEndPlug() {
        forceEndPlug = true;
    }
    private int unjamCounter = 0;
    public void unjamUpdate() {
        spinTransferWheel(true);
        if (shotTimer.getElapsedTimeSeconds() > .32){
            setIndexerPos(currentIndexerState == IndexerEnums.intake0 ? IndexerEnums.intake3: IndexerEnums.intake0);
            shotTimer.resetTimer();
            unjamCounter++;
        }
        if (unjamCounter > 2) {
            unjam = false;
            spinTransferWheel(false);
            setIndexerPos(IndexerEnums.intake0);
            clearBallCells();
            allowIntaking = true;
        }
    }
    Timer shotTimer = new Timer();
    public void setHasBalls(BallColor[] set) {
        ballCells = set;
    }
    public void intakeLogicUpdate(boolean intaking, boolean readyToShoot) {
        if (intaking && !isHasBallsFull() && isAtPosition() && !shooting && !indexerPlug) {
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

                if (nextIndex == 2) {
                    // do the thing where we plug the intake until we move x inches
                    startIndexerPlug = true;
                } else {
                    setIndexerPos(IndexerEnums.getEnum(nextIndex, false));
                }

                if (nextIndex == 3) {
                    allowIntaking = false;
                }
            }
        }
        if (!beenInited && encoderOffsetFromAuto >= 0.001 && intaking) {
            setIndexerPos(IndexerEnums.intake0);
            beenInited = true;
        }
    }
    public void spit() {
        doSpit = true;
    }

    boolean dumbShootState2 = false;
    public void dumbShootLogicUpdate(boolean readyToShoot) {
        // move to highest full index
        if (startShooting && readyToShoot) {
            setIndexerPos(IndexerEnums.shoot1);
            spinTransferWheel(true);
            shotTimer.resetTimer();
            startShooting = false;
        }

        // once at highest full index, shoot while going back to intake
        if (shooting && !dumbShootState1 && !dumbShootState2 && isAtPosition()) {
            setIndexerPos(IndexerEnums.intake0);
            dumbShootState1 = true;
        }

        // once back at intake, stop shooting
        if (dumbShootState1 && !dumbShootState2 && isAtPosition()) {
            dumbShootState1 = false;
            dumbShootState2 = true;
            shotTimer.resetTimer();
            if (!unjam) {
                clearBallCells();
            }
        }
        if (dumbShootState2 && !dumbShootState1 && shotTimer.getElapsedTimeSeconds() > .33) {
            spinTransferWheel(false);
            shooting = false;
            allowIntaking = true;
            dumbShootState2 = false;
        }
    }
    SmartShootState smartShootState = SmartShootState.IDLE;
    public void smartShootLogicUpdate(boolean readyToShoot) {
        if (!isAtPosition()) {spinTransferWheel(false);}

        switch (smartShootState) {
            case IDLE:
                if (startShooting && readyToShoot) {
                    startShooting = false;
                    smartShootState = SmartShootState.GO_TO_INDEX;
                }
                break;
            case GO_TO_INDEX:
                spinTransferWheel(false);
                if (isQueuedBallsEmpty()) {
                    allowIntaking = true;
                    setIndexerPos(IndexerEnums.intake0);
                    shooting = false;
                    smartShootState = SmartShootState.IDLE;
                } else {
                    setIndexerPos(IndexerEnums.getEnum(findIndexWithColor(queuedBalls[0]), true));
                    smartShootState = SmartShootState.SHOOT;
                }
                break;
            case SHOOT:
                if (shooting && isAtPosition()) {
                    spinTransferWheel(true);
                    feedTimer.resetTimer();
                    ballCells[findIndexWithColor(queuedBalls[0])] = BallColor.None;
                    removeFrontQueue();
                    smartShootState = SmartShootState.WAIT_TO_SETTLE;
                }
                break;
            case WAIT_TO_SETTLE:
                if (feedTimer.getElapsedTimeSeconds() > Constants.Indexer.smartShootSpacingSec) {
                    smartShootState = SmartShootState.GO_TO_INDEX;
                }
                break;
        }
    }

    public void stop() {
        setIndexerPos(getEncoderPercentage());
        setFeederPower(0);
    }
}
