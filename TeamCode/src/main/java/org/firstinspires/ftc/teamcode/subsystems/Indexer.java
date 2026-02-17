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
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.ColorFunctions;
import org.firstinspires.ftc.teamcode.util.FieldShapes;
import org.firstinspires.ftc.teamcode.util.IndexerEnums;
import org.firstinspires.ftc.teamcode.util.ShapeDetection;

public class Indexer {

    ServoImplEx spindexer1;
    public ServoImplEx spindexer2;
    DcMotorEx feeder;
    DcMotorEx encoder;
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
    public IndexerEnums deleteme = IndexerEnums.intake1;

    boolean startShooting = false;
    boolean shooting = false;
    boolean unjam = false;
    boolean boostPID;
    boolean dumbShootState1 = false;
    boolean smartShootStage1 = false;
    boolean smartShootStage2 = false;
    private boolean allowIntaking = true;
    private boolean forceEndPlug = false;
    private boolean doSpit = false;
    Timer spitTimer = new Timer();
    Timer feedTimer = new Timer();
    private boolean startIndexerPlug = false;
    private boolean indexerPlug = false;
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

        colorSensor = hardwareMap.get(RevColorSensorV3.class, "spindexerColorSensor");
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

    public NormalizedRGBA getColorRGBA(){
        return colorSensor.getNormalizedColors();
    }

    public double getDistance(){
        return colorSensor.getDistance(DistanceUnit.INCH);
    }

    public BallColor getColor(){
        return ColorFunctions.toColor(getColorRGBA(),getDistance());
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
    //Update **************************************************************************************~
    public void update(boolean intaking, boolean readyToShoot, Pose currentPose) {
        update(intaking, readyToShoot, false, false, currentPose);
    }

    public void update(boolean intaking, boolean readyToShoot, boolean doSmartShoot, boolean isFarShot, Pose currentPose) {
        updatingEncoderPos = -encoder.getCurrentPosition(); //updates this variable on tick so we are not calling multiple times in one tick
        // Stops if unjamming
        if (unjam) {
            unjamUpdate();
            spinTransferWheelUpdate();
            return;
        }

        // starts shooting if there is anything in queue
        if (!isQueuedBallsEmpty() && doSmartShoot && !shooting) {
            startShooting = true;
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
        } else if (!isFarShot){
            dumbShootLogicUpdate(readyToShoot);
        } else {
            farShootingLogicUpdate(readyToShoot);
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
        if (shotTimer.getElapsedTimeSeconds() > .2){
            setIndexerPos(currentIndexerState == IndexerEnums.intake0 ? IndexerEnums.intake3: IndexerEnums.intake0);
            shotTimer.resetTimer();
            unjamCounter++;
        }
        if (unjamCounter > 4) {
            unjam = false;
            spinTransferWheel(false);
            setIndexerPos(IndexerEnums.intake0);
            clearBallCells();
            allowIntaking = true;
        }
    }
    boolean shotTimerStarted = false;
    boolean next = false;
    boolean hasShot = true;
    Timer shotTimer = new Timer();
    private final double SHOT_TIME = .2;

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
        if (shooting && shotTimerStarted && shotTimer.getElapsedTimeSeconds() > SHOT_TIME /*&& readyToShoot */ && isAtPosition() && hasShot) {
            shotTimer.resetTimer();
            switch (currentIndexerState) {
                case intake1:
                case intake2:
                case intake3:
                    setIndexerPos(IndexerEnums.shoot1);
                    hasShot = false;
                    break;
                case shoot1:
                    setIndexerPos(IndexerEnums.shoot0);
                    hasShot = false;
                    break;
                case shoot0:
                    setIndexerPos(IndexerEnums.shoot2);
                    hasShot = false;
                    break;
                case shoot2:
                    setIndexerPos(IndexerEnums.intake0);
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
        }
    }
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
            boostPID = true;
        }

        // once at highest full index, shoot while going back to intake
        if (shooting && !dumbShootState1 && !dumbShootState2 && isAtPosition() && shotTimer.getElapsedTimeSeconds() > .2) {
            setIndexerPos(IndexerEnums.intake0);
            dumbShootState1 = true;
            boostPID = false;
        }

        // once back at intake, stop shooting
        if (dumbShootState1 && !dumbShootState2 && isAtPosition()) {
            dumbShootState1 = false;
            dumbShootState2 = true;
            shotTimer.resetTimer();
            clearBallCells();
        }
        if (dumbShootState2 && !dumbShootState1 && shotTimer.getElapsedTimeSeconds() > .33) {
            spinTransferWheel(false);
            shooting = false;
            allowIntaking = true;
            dumbShootState2 = false;
        }
    }
    public void smartShootLogicUpdate(boolean readyToShoot) {
        if (!isAtPosition()) {
            spinTransferWheel(false);
        }
        if (startShooting && readyToShoot) {
            deleteme = IndexerEnums.getEnum(findIndexWithColor(queuedBalls[0]), true);
            setIndexerPos(IndexerEnums.getEnum(findIndexWithColor(queuedBalls[0]), true));
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
                setIndexerPos(IndexerEnums.intake0);
                shooting = false;
                smartShootStage2 = false;
            } else {
                setIndexerPos(IndexerEnums.getEnum(findIndexWithColor(queuedBalls[0]), true));
                smartShootStage2 = false;
                smartShootStage1 = true;
            }
        }
    }

    public void stop() {
        setIndexerPos(getEncoderPercentage());
        setFeederPower(0);
    }
}
