package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.ColorFunctions;
import org.firstinspires.ftc.teamcode.util.IndexerEnums;

import java.util.ArrayList;
import java.util.Arrays;


public class SmartIndexerWNoTRev {
    // declaring motor variables
    Servo spindexer1;
    Servo spindexer2;
    DcMotorEx feeder;
    RevColorSensorV3 colorSensor;

    // ONLY USE STORE STATES FROM THE ENUM IN HERE
    private IndexerEnums currentIndexerState = IndexerEnums.StoreTlBTr012;

    private BallColor beReadyToTransferColor = BallColor.None;
    private BallColor transferColor = BallColor.None;

    private BallColor[] hasBalls = new BallColor[3];

    //This is the balls that the shooter prepares to shoot
    private ArrayList<BallColor> queuedBalls = new ArrayList<>();
    boolean force = true;

    double lastIndexerPos;
    double lastTransferPower;

    boolean inShootingMode = false;
    private boolean allowIntaking = true;

    Timer movingTimer;
    Timer transferTimer;

    //HOW THIS CLASS WORKS:
    // On initialization the indexer starts with index 0 at top left, index 1 in the intake opening (bottem), and index 2 top right
    // When we move the indexer, THIS IS NO LONGER THE CASE. When we rotate the indexer, the holes will stay at the same index but be in differnet spots

    public SmartIndexerWNoTRev(HardwareMap hardwaremap) {
        spindexer1 = hardwaremap.get(Servo.class, "spindexer1");
        spindexer2 = hardwaremap.get(Servo.class, "spindexer2");
        spindexer1.setDirection(Servo.Direction.REVERSE);
        spindexer2.setDirection(Servo.Direction.REVERSE);

        feeder = hardwaremap.get(DcMotorEx.class, "transfer");
        feeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feeder.setDirection(DcMotorSimple.Direction.REVERSE);

        colorSensor = hardwaremap.get(RevColorSensorV3.class, "spindexerColorSensor");
        movingTimer = new Timer();
        transferTimer = new Timer();
        hasBalls[0] = BallColor.None;
        hasBalls[1] = BallColor.None;
        hasBalls[2] = BallColor.None;
        setIndexerPos(currentIndexerState);
    }

    // DO NOT USE THESE FUNCTIONS TO MOVE THE INDEXER DURING NORMAL OPERATION. USE UPDATE() INSTEAD
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

    private void transfer(boolean set) {
        setIndexerToShooterPower(set ? Constants.Indexer.transferPower : 0);
    }

    // Getter functions outside


    public boolean allowIntakeing() {
        return allowIntaking;
    }

    public BallColor[] getHasBalls() {
        return hasBalls;
    }

    public ArrayList<BallColor> getQueuedBalls() {
        return queuedBalls;
    }

    // Moves a ball of color to the transfer
    // if ball of color is not in the has balls and force is false it will move any ball to the transfer
    public void queueColor(BallColor color, boolean force) {
        queuedBalls.add(color);
        this.force = force;
    }

    public void beReadyToShoot(BallColor color) {
        // a stupid way to check if color is in hasBalls
        if (Arrays.stream(hasBalls).anyMatch(c -> c == color)) {
            beReadyToTransferColor = color;
        }
    }

    //Color
    public NormalizedRGBA getColorRGBA(){
        return colorSensor.getNormalizedColors();
    }
    public BallColor getColor(){
        return ColorFunctions.toColor(getColorRGBA(), getDistance());
    }
    public double getDistance(){
        return colorSensor.getDistance(DistanceUnit.INCH);
    }

    // State machine for indexer ********************************~
    
    // Returns true if all slots in hasBalls are filled (not BallColor.None)
    private boolean isHasBallsFull() {
        for (BallColor color : hasBalls) {
            if (color == BallColor.None) {
                return false;
            }
        }
        return true;
    }

    private boolean isHasBallsEmpty() {
        for (BallColor color : hasBalls) {
            if (color != BallColor.None) {
                return false;
            }
        }
        return true;
    }

    // Functions to get current index positions ***************************************~
    // Gets the current index at the bottom and shooter positions based on current state
    public int getCurrentIndexAtBottem(){
        switch (currentIndexerState) {
            case StoreTlBTr012Revrese:
                return 1;
            case StoreTlBTr120:
                return 2;
            case StoreTlBTr201:
                return 0;
            case StoreTlBTr012:
                return 1;
        }
        return -2000000000;
    }

    // Gets the current index at the shooter position based on current state
    public int getCurrentIndexAtShooter(){
        switch (currentIndexerState) {
            case TransferTBlBr012Reverse:
                return 0;
            case TransferTBlBr120:
                return 1;
            case TransferTBlBr201:
                return 2;
        }
        return -2000000000;
    }

    // Functions to get order of indexer states ***************************************~
    // Returns the prefered order of indexer states to store a new ball based on current state
    public IndexerEnums[] getPreferredIndexToStore() {
        switch (currentIndexerState) {
            // From a store Position
            case StoreTlBTr012Revrese:
                return new IndexerEnums[]{
                    IndexerEnums.StoreTlBTr120,
                    IndexerEnums.StoreTlBTr201,
                    IndexerEnums.StoreTlBTr012
                };
            case StoreTlBTr120:
                return new IndexerEnums[]{
                    IndexerEnums.StoreTlBTr201,
                    IndexerEnums.StoreTlBTr012Revrese,
                    IndexerEnums.StoreTlBTr012
                };
            case StoreTlBTr201:
                return new IndexerEnums[]{
                    IndexerEnums.StoreTlBTr120,
                    IndexerEnums.StoreTlBTr012,
                    IndexerEnums.StoreTlBTr012Revrese
                };
            case StoreTlBTr012:
                return new IndexerEnums[] {
                    IndexerEnums.StoreTlBTr201,
                    IndexerEnums.StoreTlBTr120,
                    IndexerEnums.StoreTlBTr012Revrese
                };
            // Transfer States
            case TransferTBlBr012Reverse:
                return new IndexerEnums[] {
                    IndexerEnums.StoreTlBTr120,
                    IndexerEnums.StoreTlBTr012,
                    IndexerEnums.StoreTlBTr201
                };
            case TransferTBlBr120:
                return new IndexerEnums[] {
                    IndexerEnums.StoreTlBTr120,
                    IndexerEnums.StoreTlBTr201,
                    IndexerEnums.StoreTlBTr012Revrese
                };
            case TransferTBlBr201:
                return new IndexerEnums[] {
                    IndexerEnums.StoreTlBTr201,
                    IndexerEnums.StoreTlBTr012,
                    IndexerEnums.StoreTlBTr120
                };
        }
        return null;
    }

    // Returns the prefered order of indexer states to transfer a new ball based on current state
    public IndexerEnums[] getPreferedIndexToTransfer() {
        switch (currentIndexerState) {
            case StoreTlBTr012Revrese:
                return new IndexerEnums[]{
                    IndexerEnums.TransferTBlBr012Reverse,
                    IndexerEnums.TransferTBlBr120,
                    IndexerEnums.TransferTBlBr201
                };
            case StoreTlBTr120:
                return new IndexerEnums[]{
                    IndexerEnums.TransferTBlBr120,
                    IndexerEnums.TransferTBlBr012Reverse,
                    IndexerEnums.TransferTBlBr201
                };
            case StoreTlBTr201:
                return new IndexerEnums[]{
                    IndexerEnums.TransferTBlBr120,
                    IndexerEnums.TransferTBlBr201,
                    IndexerEnums.TransferTBlBr012Reverse
                };
            case StoreTlBTr012:
                return new IndexerEnums[] {
                    IndexerEnums.TransferTBlBr201,
                    IndexerEnums.TransferTBlBr120,
                    IndexerEnums.TransferTBlBr012Reverse
                };
            // Transfer States
            case TransferTBlBr012Reverse:
                return new IndexerEnums[] {
                    IndexerEnums.TransferTBlBr120,
                    IndexerEnums.TransferTBlBr012,
                    IndexerEnums.TransferTBlBr201
                };
            case TransferTBlBr120:
                return new IndexerEnums[] {
                    IndexerEnums.TransferTBlBr120,
                    IndexerEnums.TransferTBlBr201,
                    IndexerEnums.TransferTBlBr012
                };
            case TransferTBlBr201:
                return new IndexerEnums[] {
                    IndexerEnums.TransferTBlBr201,
                    IndexerEnums.TransferTBlBr012,
                    IndexerEnums.TransferTBlBr120
                };
        }
        return null;
    }

    // Functions for converting a Indexer Enum to ball colors ******************************~
    // Returns the ball color at the intake position if it were at given a store state
    public BallColor ballColorAtIntake(IndexerEnums state) {
        switch (state) {
            case StoreTlBTr012Revrese:
                return hasBalls[1];
            case StoreTlBTr120:
                return hasBalls[2];
            case StoreTlBTr201:
                return hasBalls[0];
            case StoreTlBTr012:
                return hasBalls[1];
        }
        return null;
    }

    // Returns the ball color at the top left or top right position if it were at given a store state
    public BallColor[] ballColorsAtReadyToShoot(IndexerEnums state) { //todo This is wrong for now because we dont have T Rev anymore but I dont wanna change it because it will break other logic, will require a large rewrite if we mess with it
        switch (state) {
            case StoreTlBTr012Revrese:
                return new BallColor[]{hasBalls[0], hasBalls[2]};
            case StoreTlBTr120:
                return new BallColor[]{hasBalls[1], hasBalls[0]};
            case StoreTlBTr201:
                return new BallColor[]{hasBalls[2], hasBalls[1]};
            case StoreTlBTr012:
                return new BallColor[]{hasBalls[0], hasBalls[2]};
        }
        return null;
    }
    
    // Returns the ball color at the top position if it were at given a transfer state
    public BallColor ballColorsAtTransferPosition(IndexerEnums state) {
        switch (state) {
            case TransferTBlBr012Reverse:
                return hasBalls[0];
            case TransferTBlBr120:
                return hasBalls[1];
            case TransferTBlBr201:
                return hasBalls[2];
        }
        return null;
    }

    // Functions for deciding a state based on a ball color ******************************~
    // Returns the correct store state to move to for storing a ball of given color that was in the intake
    public IndexerEnums getCorrectStoreStateAfterIntake() {

        // No space to store
        if (isHasBallsFull()) {
            if (isAStoreState(currentIndexerState)) {
                // if we are at a store state then return our current state
                return currentIndexerState;
            } else {
                // if we are not at a store state then return the closest prefered state
                return getPreferredIndexToStore()[0];
            }
        }

        // Gets the prefered order to store
        IndexerEnums[] preferedStoreOrder = getPreferredIndexToStore();

        // Finds the first state in the order where the intake position is empty
        for (IndexerEnums state : preferedStoreOrder) {
            if (ballColorAtIntake(state) == BallColor.None) {
                return state;
            }
        }
        return null;
    }

    // Returns the correct store state to move to for being ready to transfer a ball of given color
    public IndexerEnums getCorrectStoreStateForBeReadyToTransfer(BallColor color) {
        
        // Gets the prefered order to store
        IndexerEnums[] preferedStoreOrder = getPreferredIndexToStore();

        // If hasBalls is not full than there is a spot such that the correct color is ready and there is nothing in the intake slot
        if (!isHasBallsFull()) {
            // Finds the first state in the order where the ball color is at either ready to shoot position
            // AND the intake position is empty
            for (IndexerEnums state : preferedStoreOrder) {
                BallColor[] readyColors = ballColorsAtReadyToShoot(state);
                BallColor intakeColor = ballColorAtIntake(state);
                if (intakeColor == BallColor.None && (readyColors[0] == color || readyColors[1] == color) ) {
                    return state;
                }
            }
        } else {
            // Finds the first state in the order where the ball color is at either ready to shoot position
            for (IndexerEnums state : preferedStoreOrder) {
                BallColor[] readyColors = ballColorsAtReadyToShoot(state);
                if (readyColors[0] == color || readyColors[1] == color) {
                    return state;
                }
            }
        }
        return null;
    }
    
    // Returns the correct transfer state to move to for transferring a ball of given color
    public IndexerEnums getCorrectTransferStateForTransfer(BallColor color) {
        // Gets the prefered order to transfer
        IndexerEnums[] preferredTransferOrder = getPreferedIndexToTransfer();

        // Finds the first state in the order where the ball color is at the transfer position
        for (IndexerEnums state : preferredTransferOrder) {
            if (ballColorsAtTransferPosition(state) == color) {
                // if ball is correct color
                return state;
            } else if (beReadyToTransferColor != BallColor.Any && ballColorsAtTransferPosition(state) != BallColor.None) {
                // if looking for any ball and ball is not none
                return state;
            }
        }
        return null;
    }

    // Functions to decide how long we are moving ***************************************~
    // Returns the time in seconds to move from current state to target state
    public double getTimeToMoveToState(IndexerEnums targetState) {
        // If already at target state, no time needed
        if (currentIndexerState == targetState) {
            return 0;
        }

        // Calculate rotations needed to move to target state
        // Creates list of order of states
        IndexerEnums[] IndexerStatesOrder = new IndexerEnums[]{
            IndexerEnums.StoreTlBTr012Revrese,
            IndexerEnums.TransferTBlBr012Reverse,
            IndexerEnums.StoreTlBTr120,
            IndexerEnums.TransferTBlBr120,
            IndexerEnums.StoreTlBTr201,
            IndexerEnums.TransferTBlBr201,
            IndexerEnums.StoreTlBTr012,
        };

        // Finds the indexes of the current and target states
        int currentStateIndex = -10000000;
        int targetStateIndex = -1000;
        for (int i = 0; i < IndexerStatesOrder.length; i++) {
            if (IndexerStatesOrder[i] == currentIndexerState) {
                currentStateIndex = i;
            }
            if (IndexerStatesOrder[i] == targetState) {
                targetStateIndex = i;
            }
        }

        // Computes distance
        double distance = Math.abs(targetStateIndex - currentStateIndex);

        // Because our list includes both store and transfer states, we divide distance by 2
        return (distance / 2) * Constants.Indexer.timeToMoveIndexer1Index;
    }

    // Resets the moving timer and currentState to be proper and sets the indexer to the correct spot givin a state
    public void setIndexerPos(IndexerEnums state) {
        movingTimer.resetTimer(getTimeToMoveToState(state));
        currentIndexerState = state;
        switch (state) {
            case StoreTlBTr012Revrese:
                setIndexerPos(Constants.Indexer.StoreTlBTr012Reverse);
                return;

            case StoreTlBTr120:
                setIndexerPos(Constants.Indexer.StoreTlBTr120);
                return;

            case StoreTlBTr201:
                setIndexerPos(Constants.Indexer.StoreTlBTr201);
                return;

            case StoreTlBTr012:
                setIndexerPos(Constants.Indexer.StoreTlBTr012);
                return;


            case TransferTBlBr012Reverse:
                setIndexerPos(Constants.Indexer.TransferTBlBr012Reverse);
                return;

            case TransferTBlBr120:
                setIndexerPos(Constants.Indexer.TransferTBlBr120);
                return;

            case TransferTBlBr201:
                setIndexerPos(Constants.Indexer.TransferTBlBr201);
                return;
        }
    }

    // Returns if the given state is a store state
    public boolean isAStoreState(IndexerEnums state) {
        switch (state) {
            case StoreTlBTr012Revrese:
                return true;

            case StoreTlBTr120:
                return true;

            case StoreTlBTr201:
                return true;

            case StoreTlBTr012:
                return true;


            case TransferTBlBr012Reverse:
                return false;

            case TransferTBlBr120:
                return false;

            case TransferTBlBr201:
                return false;
        }

        System.exit(0); // todo remove

        return false;
    }

    // uses time to figure out if ball has left
    public void update(boolean cancelShoot, boolean readyToShoot, boolean autoShoot) {
        if (lastTransferPower == 0) {
            transferTimer.resetTimer();
        }

        boolean ballHasLeft = transferTimer.getElapsedTimeSeconds() > Constants.Indexer.timeToTransfer;
        update(cancelShoot, readyToShoot, ballHasLeft, autoShoot);
    }

    // Combines all the functions above to move the indexer smartly based on current goals *******************************~
    public void update(boolean cancelShoot, boolean readyToShoot, boolean ballHasLeftShooter, boolean autoShoot) {
        // readyToShoot = we are in shooter zone, and we have balls to shoot, and our spinners are close enough
        // if readyToShoot,
        //      allow intaking false 
        //      move to shoot the ball in transferBall
        // once we have moved to the correct state being an intake state,
        //      allow intaking true
        // once we have moved to the correct state being a transfer state, 
        //      if readyToShoot
        //          then go into a shooting mode where we only look at if a ball has left the shooter or if we need to cancel shooting
        //      else
        //          we go back to the best intake position
        // if in that only shooting mode,
        //      if cancel shoot or ball has left shooter
        //          exit shooting mode
        //          move to best intake position
        //      if readyToShoot,
        //          spin transfer

        allowIntaking = isAStoreState(currentIndexerState) && movingTimer.getElapsedTimeSeconds() > 0 && !isHasBallsFull();


        // if nothing is in transferColor than try to put the most recent queue in there
        if (transferColor == BallColor.None && !queuedBalls.isEmpty()) {
            BallColor seeIfWeHaveColor = queuedBalls.get(0);
            // a stupid way to check if color is in hasBalls
            if (Arrays.stream(hasBalls).anyMatch(c -> c == seeIfWeHaveColor)) {
                queuedBalls.remove(0);
                transferColor = seeIfWeHaveColor;
            } else if (force && seeIfWeHaveColor != BallColor.None) {
                queuedBalls.remove(0); //HERE
                transferColor = BallColor.Any;
            }
        }

        if (readyToShoot && (transferColor != BallColor.None || autoShoot) && isAStoreState(currentIndexerState) && movingTimer.getElapsedTimeSeconds() > 0) {
            if (transferColor != BallColor.None) {
                setIndexerPos(getCorrectTransferStateForTransfer(transferColor));
            } else {
                setIndexerPos(getCorrectTransferStateForTransfer(BallColor.Any));
            }
        }

        if (movingTimer.getElapsedTimeSeconds() > 0) {
            // Indexer stopped spinning
            if (isAStoreState(currentIndexerState)) {
                // at a store state
            } else if (!inShootingMode) {
                // at a transfer state
                if (readyToShoot) {
                    inShootingMode = true;
                } else {
                    setIndexerPos(getCorrectStoreStateAfterIntake());
                }
            }
        }

        if (inShootingMode && movingTimer.getElapsedTimeSeconds() > 0) {
            // Shooting Mode
            // spins transfer wheel if we are ready to shoot
            transfer(readyToShoot);
            if (ballHasLeftShooter) {
                transferColor = BallColor.None;
                hasBalls[getCurrentIndexAtShooter()] = BallColor.None;

                // if we probebly want to shoot more
                if (autoShoot && !isHasBallsEmpty()) {
                    getCorrectTransferStateForTransfer(BallColor.Any);
                } else {
                    setIndexerPos(getCorrectStoreStateAfterIntake());
                    inShootingMode = false;
                }
            }
            if (cancelShoot) {
                transferColor = BallColor.None;
                setIndexerPos(getCorrectStoreStateAfterIntake());
                inShootingMode = false;
            }
        } 
        if (!inShootingMode && !isHasBallsFull() && movingTimer.getElapsedTimeSeconds() > 0) {
            // Intaking mode
            if (getColor() != BallColor.None) {
                hasBalls[getCurrentIndexAtBottem()] = getColor();
                setIndexerPos(getCorrectStoreStateAfterIntake());
            }
        }
        if (!inShootingMode && movingTimer.getElapsedTimeSeconds() > 0) {
            // Be Ready To Transfer Mode
            if (beReadyToTransferColor != BallColor.None) {
                setIndexerPos(getCorrectStoreStateForBeReadyToTransfer(beReadyToTransferColor));
                beReadyToTransferColor = BallColor.None;
            }
        }
    }
        
        // Store States
        //Reverse
        // 0   2 | 1   0 | 2   1 | 0   2 
        //   1   |   2   |   0   |   1   
        // Transfer States 
        //    Reverse
        //       0   |   1   |   2   |   0   
        //     1   2 | 2   0 | 0   1 | 1   2 

}