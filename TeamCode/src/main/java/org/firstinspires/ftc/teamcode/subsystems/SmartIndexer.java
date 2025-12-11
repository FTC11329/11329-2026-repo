import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.IndexerEnums;

public class SmartIndexer {
    // declaring motor variables
    Servo spindexer1;
    Servo spindexer2;
    DcMotorEx feeder;
    RevColorSensorV3 colorSensor;

    // ONLY USE STORE STATES FROM THE ENUM IN HERE
    private IndexerEnums currentIndexerState = IndexerEnums.StoreTlBTr012;

    private BallColor beReadyToTransferColor = BallColor.None;
    private BallColor transferColor = BallColor.None;

    BallColor[] hasBalls = new BallColor[3];

    double lastIndexerPos;
    double lastTransferPower;

    boolean inShootingMode = false;
    boolean allowIntakeing = true;

    Timer movingTimer;

    //HOW THIS CLASS WORKS:
    // On initialization the indexer starts with index 0 at top left, index 1 in the intake opening (bottem), and index 2 top right
    // When we move the indexer, THIS IS NO LONGER THE CASE. When we rotate the indexer, the holes will stay at the same index but be in differnet spots

    public SmartIndexer(HardwareMap hardwaremap) {
        spindexer1 = hardwaremap.get(Servo.class, "spindexer1");
        spindexer2 = hardwaremap.get(Servo.class, "spindexer2");
        spindexer1.setDirection(DcMotorSimple.Direction.FORWARD);
        spindexer2.setDirection(DcMotorSimple.Direction.FORWARD);

        feeder = hardwaremap.get(DcMotorEx.class, "transfer");
        feeder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feeder.setDirection(DcMotorSimple.Direction.REVERSE);

        colorSensor = hardwaremap.get(RevColorSensorV3.class, "spindexerColorSensor");
        movingTimer = new Timer();
        hasBalls[0] = BallColor.None;
        hasBalls[1] = BallColor.None;
        hasBalls[2] = BallColor.None;
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

    // Moves a ball of color to the transfer
    // if ball of color is not in the has balls and force is false it will move any ball to the transfer
    public void shootWhenReady(BallColor color, boolean force) {
        // a stupid way to check if color is in hasBalls
        if (Arrays.stream(hasBalls).anyMatch(c -> c == color) || force) {
            transferColor = color;
        }
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
        return ColorFunctions.toColor(getColorRGBA(), getDistance(DistanceUnit.INCH));
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
    
    // Helper method to calculate rotations needed to move a ball to top
    private int calculateRotationsToTop(int ballIndex) {
        // Simplified: assume we want to rotate clockwise
        return (ballIndex - currentIndexAtBottem + 3) % 3;
    }
    // Functions to get current index positions ***************************************~
    // Gets the current index at the bottom and shooter positions based on current state
    public int getCurrentIndexAtBottem(){
        switch (currentIndexerState) {
            case IndexerEnums.StoreTlBTr012Revrese:
                return 1;
            case IndexerEnums.StoreTlBTr120:
                return 2;
            case IndexerEnums.StoreTlBTr201:
                return 0;
            case IndexerEnums.StoreTlBTr012:
                return 1;
        }
    }

    // Gets the current index at the shooter position based on current state
    public int getCurrentIndexAtShooter(){
        switch (currentIndexerState) {
            case IndexerEnums.TransferTBlBr012Reverse:
                return 0;
            case IndexerEnums.TransferTBlBr120:
                return 1;
            case IndexerEnums.TransferTBlBr201:
                return 2;
            case IndexerEnums.TransferTBlBr012:
                return 0;
        }
    }

    // Functions to get order of indexer states ***************************************~
    // Returns the prefered order of indexer states to store a new ball based on current state
    public IndexerEnums[] getPreferedIndexToStore() {
        switch (currentIndexerState) {
            case IndexerEnums.StoreTlBTr012Revrese:
                return new IndexerEnums[]{
                    IndexerEnums.StoreTlBTr120,
                    IndexerEnums.StoreTlBTr201,
                    IndexerEnums.StoreTlBTr012
                };
            case IndexerEnums.StoreTlBTr120:
                return new IndexerEnums[]{
                    IndexerEnums.StoreTlBTr201,
                    IndexerEnums.StoreTlBTr012Revrese,
                    IndexerEnums.StoreTlBTr012
                };
            case IndexerEnums.StoreTlBTr201:
                return new IndexerEnums[]{
                    IndexerEnums.StoreTlBTr120,
                    IndexerEnums.StoreTlBTr012,
                    IndexerEnums.StoreTlBTr012Revrese
                };
            case IndexerEnums.StoreTlBTr012:
                return new IndexerEnums[] {
                    IndexerEnums.StoreTlBTr201,
                    IndexerEnums.StoreTlBTr120,
                    IndexerEnums.StoreTlBTr012Revrese
                };
        }
        return null;
    }

    // Returns the prefered order of indexer states to transfer a new ball based on current state
    public IndexerEnums[] getPreferedIndexToTransfer() {
        switch (currentIndexerState) {
            case IndexerEnums.StoreTlBTr012Revrese:
                return new IndexerEnums[]{
                    IndexerEnums.TransferTBlBr012,
                    IndexerEnums.TransferTBlBr120,
                    IndexerEnums.TransferTBlBr201
                };
            case IndexerEnums.StoreTlBTr120:
                return new IndexerEnums[]{
                    IndexerEnums.TransferTBlBr120,
                    IndexerEnums.TransferTBlBr012Reverse,
                    IndexerEnums.TransferTBlBr201
                };
            case IndexerEnums.StoreTlBTr201:
                return new IndexerEnums[]{
                    IndexerEnums.TransferTBlBr120,
                    IndexerEnums.TransferTBlBr201,
                    IndexerEnums.TransferTBlBr012Reverse // or 012, either works
                };
            case IndexerEnums.StoreTlBTr012:
                return new IndexerEnums[] {
                    IndexerEnums.TransferTBlBr201,
                    IndexerEnums.TransferTBlBr012,
                    IndexerEnums.TransferTBlBr120
                };
        }
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
    }

    // Returns the ball color at the top left or top right position if it were at given a store state
    public BallColor[] ballColorsAtReadyToShoot(IndexerEnums state) {
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
            case TransferTBlBr012:
                return hasBalls[0];
        }
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
                return getPreferedIndexToStore()[0];
            }
        }

        // Gets the prefered order to store
        IndexerEnums[] preferedStoreOrder = getPreferedIndexToStore();

        // Finds the first state in the order where the intake position is empty
        for (IndexerEnums state : preferedStoreOrder) {
            if (ballColorAtIntake(state) == BallColor.None) {
                return state;
            }
        }
    }

    // Returns the correct store state to move to for being ready to transfer a ball of given color
    public IndexerEnums getCorrectStoreStateForBeReadyToTransfer(BallColor color) {
        
        // Gets the prefered order to store
        IndexerEnums[] preferedStoreOrder = getPreferedIndexToStore();

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
    }
    
    // Returns the correct transfer state to move to for transferring a ball of given color
    public IndexerEnums getCorrectTransferStateForTransfer(BallColor color) {
        // Gets the prefered order to transfer
        IndexerEnums[] preferedTransferOrder = getPreferedIndexToTransfer();

        // Finds the first state in the order where the ball color is at the transfer position
        for (IndexerEnums state : preferedTransferOrder) {
            if (ballColorsAtTransferPosition(state) == color) {
                // if ball is correct color
                return state;
            } else if (beReadyToTransferColor != BallColor.Any && ballColorsAtTransferPosition(state) != BallColor.None) {
                // if looking for any ball and ball is not none
                return state;
            }
        }
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
            IndexerEnums.TransferTBlBr012
        };

        // Finds the indexes of the current and target states
        int currentStateIndex = null;
        int targetStateIndex = null;
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
            case IndexerEnums.StoreTlBTr012Revrese:
                setIndexerPos(Constants.StoreTlBTr012Revrese);
                return;

            case IndexerEnums.StoreTlBTr120:
                setIndexerPos(Constants.StoreTlBTr120);
                return;

            case IndexerEnums.StoreTlBTr201:
                setIndexerPos(Constants.StoreTlBTr201);
                return;

            case IndexerEnums.StoreTlBTr012:
                setIndexerPos(Constants.StoreTlBTr012);
                return;


            case IndexerEnums.TransferTBlBr012Reverse:
                setIndexerPos(Constants.TransferTBlBr012Reverse);
                return;

            case IndexerEnums.TransferTBlBr120:
                setIndexerPos(Constants.TransferTBlBr120);
                return;

            case IndexerEnums.TransferTBlBr201:
                setIndexerPos(Constants.TransferTBlBr201);
                return;

            case IndexerEnums.TransferTBlBr012:
                setIndexerPos(Constants.TransferTBlBr012);
                return;
        }
    }

    // Returns if the given state is a store state
    public boolean isAStoreState(IndexerEnums state) {
        switch (state) {
            case IndexerEnums.StoreTlBTr012Revrese:
                return true;

            case IndexerEnums.StoreTlBTr120:
                return true;

            case IndexerEnums.StoreTlBTr201:
                return true;

            case IndexerEnums.StoreTlBTr012:
                return true;


            case IndexerEnums.TransferTBlBr012Reverse:
                return false;

            case IndexerEnums.TransferTBlBr120:
                return false;

            case IndexerEnums.TransferTBlBr201:
                return false;

            case IndexerEnums.TransferTBlBr012:
                return false;
        }
    }


    // Combines all the functions above to move the indexer smartly based on current goals *******************************~
    public void update(boolean cancelShoot, boolean readyToShoot, boolean ballHasLeftShooter) {
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

        if (readyToShoot) {
            allowIntakeing = false;
            setIntakePos(getCorrectTransferStateForTransfer(transferColor));
        }

        if (movingTimer.getElapsedTimeSeconds() > 0) {
            // Indexer stopped spinning
            if (isAStoreState(currentIndexerState)) {
                // at a store state
                allowIntakeing = true;
            } else {
                // at a transfer state
                if (readyToShoot) {
                    inShootingMode = true;
                } else {
                    setIntakePos(getCorrectStoreStateAfterIntake());
                }
            }
        }

        if (inShootingMode && movingTimer.getElapsedTimeSeconds() > 0) {
            // Shooting Mode
            transfer(readyToShoot);
            if (ballHasLeftShooter) {
                hasBalls[getCurrentIndexAtShooter()] = BallColor.None;
                setIntakePos(getCorrectStoreStateAfterIntake());
                inShootingMode = false;
            }
            if (cancelShoot) {
                setIntakePos    (getCorrectStoreStateAfterIntake());
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