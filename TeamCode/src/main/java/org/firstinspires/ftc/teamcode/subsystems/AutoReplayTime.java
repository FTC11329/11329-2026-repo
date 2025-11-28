package org.firstinspires.ftc.teamcode.subsystems;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.SplineCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.math.Vector;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.util.DoubleAdapter;
import org.firstinspires.ftc.teamcode.util.FancyButton;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.List;

public class AutoReplayTime {

    Follower follower;
    Telemetry telemetry;
    Gamepad gamepad1;
    Gamepad gamepad2;

    Gamepad gamepadReplay1;
    Gamepad gamepadReplay2;

    FancyButton recording;
    FancyButton replay;
    FancyButton pointerInput;
    Pose lastPose = new Pose(0, 0, 0);
    double lastTime = 0;
    double deltaTime = 0.25;
    double deltaError = 2;
    int replayIndex = 0;

    int currentGamepadIndex = 0;
    int currentPoseIndex = 0;

    StateEntryJson currentReplayStates;
    PathChain replayPath;
    GamepadStateEntry gamepadDelta1;
    GamepadStateEntry gamepadDelta2;

    GamepadStateEntry lastGamePad1;
    GamepadStateEntry lastGamePad2;
    SplineCurve splineCurve;
    int logPointer = 0;

    public AutoReplayTime(Follower follower, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        this.follower = follower;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.telemetry = telemetry;
    }

    public void init() {
        recording = new FancyButton(FancyButton.PressType.Toggle);
        replay = new FancyButton(FancyButton.PressType.Toggle);
        pointerInput = new FancyButton(FancyButton.PressType.LongPress);

        currentReplayStates = new StateEntryJson();

        loadPointer();
        telemetry.addData("pointer: ", logPointer);
        telemetry.update();
    }

    public void recordPositions() {
        File dir = AppUtil.getInstance().getSettingsFile("TeamCodeLogs").getParentFile();
        File file = new File(dir, "movement" + logPointer + ".json");
        telemetry.addData("Recording, here: ", true);
        try (FileWriter writer = new FileWriter(file)) {
            Gson gson = new GsonBuilder()
                    .registerTypeAdapter(Double.class, new DoubleAdapter())
                    .registerTypeAdapter(double.class, new DoubleAdapter()) // primitive double
                    .create();
            writer.write(gson.toJson(currentReplayStates));
            writer.close();

            telemetry.addData("Drive log written", true);
        } catch (Exception e) {
            telemetry.addData("Drive log error", e.getMessage());
        }
    }

    public void savePointer() {
        File dir = AppUtil.getInstance().getSettingsFile("TeamCodeLogs").getParentFile();
        File file = new File(dir, "pointer.json");
        try (FileWriter writer = new FileWriter(file)) {
            Gson gson = new GsonBuilder().create();

            writer.write(gson.toJson(new PointerJson(logPointer)));
            writer.close();

            telemetry.addData("Pointer Saved written:", true);
        } catch (Exception e) {
            telemetry.addData("Pointer error", e.getMessage());
        }
    }

    public void loadPoses() {
        File dir = AppUtil.getInstance().getSettingsFile("TeamCodeLogs").getParentFile();
        File file = new File(dir, "movement" + logPointer + ".json");

        if (!file.exists()) return;

        StringBuilder jsonBuilder = new StringBuilder();

        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
            String line;
            while ((line = reader.readLine()) != null) {
                jsonBuilder.append(line);
            }
        } catch (Exception e) {
            telemetry.addData("Failed to Load", true);
        }

        String jsonString = jsonBuilder.toString();

        Gson gson = new GsonBuilder().create();
        currentReplayStates = gson.fromJson(jsonString, StateEntryJson.class);
    }

    public void loadPointer() {
        File dir = AppUtil.getInstance().getSettingsFile("TeamCodeLogs").getParentFile();
        File file = new File(dir, "pointer.json");

        if (!file.exists()) return;

        StringBuilder jsonBuilder = new StringBuilder();

        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
            String line;
            while ((line = reader.readLine()) != null) {
                jsonBuilder.append(line);
            }
        } catch (Exception e) {
            telemetry.addData("Failed to Load", true);
        }

        String jsonString = jsonBuilder.toString();

        Gson gson = new GsonBuilder().create();
        logPointer = gson.fromJson(jsonString, PointerJson.class).pointer;
    }

    public void update(){
        recording.checkStatus(gamepad1.a);
        replay.checkStatus(gamepad1.b);
        pointerInput.checkStatus(gamepad1.left_bumper);

        telemetry.addData("LOG POINTER", logPointer);

        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Replay Path Null", currentReplayStates == null);
        telemetry.addData("recording is on", recording.isOn);
        telemetry.addData("replay is on", replay.isOn);
        telemetry.addData("replay start", replay.startPress);
        telemetry.addData("Following Path: Replay Index: ", currentGamepadIndex);

        for (int i = 0; i < currentReplayStates.size; i++){
            telemetry.addData("t", (currentReplayStates.timeListPose.get(i)));
            telemetry.addData("pos-x", (currentReplayStates.poseList.get(i).x));
            telemetry.addData("pos-y", (currentReplayStates.poseList.get(i).y));
        }
        telemetry.addData("Size of List: ", currentReplayStates.size);

        if (pointerInput.startPress) logPointer = 0;
        if (pointerInput.isOn) logPointer = (int) Math.floor(pointerInput.time.seconds());
        if (pointerInput.endPress) savePointer();

        if (recording.startPress) {
            recording.resetTimer();
            currentReplayStates = new StateEntryJson();
            lastPose = follower.getPose();
            lastGamePad1 = new GamepadStateEntry(gamepad1);
            lastGamePad2 = new GamepadStateEntry(gamepad2);
        }
        if (recording.isOn) {
            if (recording.time.seconds() - lastTime > deltaTime) {
                currentReplayStates.timeListPose.add(recording.time.seconds());
                currentReplayStates.poseList.add(new PoseStateEntry(follower.getPose()));
                currentReplayStates.velocityList.add(new VelocityStateEntry(follower.getVelocity()));
                currentReplayStates.size += 1;
                lastPose = follower.getPose();
                lastTime = recording.time.seconds();
            }
            if (!lastGamePad1.compareGamepad(new GamepadStateEntry(gamepad1)) || !lastGamePad2.compareGamepad(new GamepadStateEntry(gamepad2))) {
                lastGamePad1 = new GamepadStateEntry(gamepad1);
                lastGamePad2 = new GamepadStateEntry(gamepad2);
                currentReplayStates.timeListGamepad.add(recording.time.seconds());
                currentReplayStates.gamepad1List.add(new GamepadStateEntry(gamepad1));
                currentReplayStates.gamepad2List.add(new GamepadStateEntry(gamepad2));
            }
        }
        if (recording.endPress){
            recordPositions();
        }
        if (currentReplayStates != null && currentReplayStates.size > 3){
            if (replay.startPress){
                replay.resetTimer();
                loadPoses();
                createPathSpline();
                currentGamepadIndex = 0;
                follower.followPath(new Path(splineCurve));
            }
            if (replay.isOn){
                double currentTime = replay.time.seconds();

                // Advance gamepad index if it's time
                if (currentGamepadIndex + 1 < currentReplayStates.timeListGamepad.size() &&
                        currentReplayStates.timeListGamepad.get(currentGamepadIndex + 1) <= currentTime) {
                    currentGamepadIndex++;
                }
                gamepadReplay1 = currentReplayStates.gamepad1List.get(currentGamepadIndex).convertToGamepad();
                gamepadReplay2 = currentReplayStates.gamepad2List.get(currentGamepadIndex).convertToGamepad();
            }
            if (replay.endPress){
                follower.breakFollowing();
            }
        }
    }

    public void createPathSpline(){
        int n = currentReplayStates.size;
        double[] t = new double[n];
        double[] x = new double[n];
        double[] y = new double[n];
        double[] theta = new double[n];

        List<Pose> points = new ArrayList<>();
        List<Pose> velocities = new ArrayList<>();
        List<Double> times = new ArrayList<>();

        for (int i = 0; i < n; i += 1){
            points.add(new Pose(currentReplayStates.poseList.get(i).x, currentReplayStates.poseList.get(i).y, currentReplayStates.poseList.get(i).heading));
            times.add(currentReplayStates.timeListPose.get(i));
            t[i] = currentReplayStates.timeListPose.get(i);
            x[i] = currentReplayStates.poseList.get(i).x;
            y[i] = currentReplayStates.poseList.get(i).y;
            theta[i] = currentReplayStates.poseList.get(i).heading;
        }

        for (int i = 1; i < n; i += 1){
            double dt = t[i - 1] - t[i];
            velocities.add(new Pose((x[i - 1] - x[i]) / dt, (y[i - 1] - y[i]) / dt, (theta[i - 1] - theta[i]) / dt));
        }
        splineCurve = new SplineCurve(points, velocities, times);
    }

    public boolean IsReplayOn(){
        return replay.isOn;
    }

    public Gamepad getGamepad1(){
        return gamepadReplay1;
    }
    public Gamepad getGamepad2(){
        return gamepadReplay2;
    }

    public static class GamepadStateEntry {
        public boolean a, b, x, y;
        public boolean dpad_up, dpad_down, dpad_left, dpad_right;
        public boolean left_bumper, right_bumper;
        public boolean left_stick_button, right_stick_button;
        public float left_stick_x, left_stick_y;
        public float right_stick_x, right_stick_y;
        public float left_trigger, right_trigger;

        public GamepadStateEntry(Gamepad g) {
            this.a = g.a;
            this.b = g.b;
            this.x = g.x;
            this.y = g.y;
            this.dpad_up = g.dpad_up;
            this.dpad_down = g.dpad_down;
            this.dpad_left = g.dpad_left;
            this.dpad_right = g.dpad_right;
            this.left_bumper = g.left_bumper;
            this.right_bumper = g.right_bumper;
            this.left_stick_button = g.left_stick_button;
            this.right_stick_button = g.right_stick_button;
            this.left_stick_x = g.left_stick_x;
            this.left_stick_y = g.left_stick_y;
            this.right_stick_x = g.right_stick_x;
            this.right_stick_y = g.right_stick_y;
            this.left_trigger = g.left_trigger;
            this.right_trigger = g.right_trigger;
        }

        public Gamepad convertToGamepad() {
            Gamepad g = new Gamepad();

            // Buttons
            g.a = this.a;
            g.b = this.b;
            g.x = this.x;
            g.y = this.y;
            g.dpad_up = this.dpad_up;
            g.dpad_down = this.dpad_down;
            g.dpad_left = this.dpad_left;
            g.dpad_right = this.dpad_right;
            g.left_bumper = this.left_bumper;
            g.right_bumper = this.right_bumper;
            g.left_stick_button = this.left_stick_button;
            g.right_stick_button = this.right_stick_button;

            // Joysticks
            g.left_stick_x = this.left_stick_x;
            g.left_stick_y = this.left_stick_y;
            g.right_stick_x = this.right_stick_x;
            g.right_stick_y = this.right_stick_y;

            // Triggers
            g.left_trigger = this.left_trigger;
            g.right_trigger = this.right_trigger;

            return g;
        }

        public void mergeBooleans(GamepadStateEntry other) {
            this.a &= other.a;
            this.b &= other.b;
            this.x &= other.x;
            this.y &= other.y;

            this.dpad_up &= other.dpad_up;
            this.dpad_down &= other.dpad_down;
            this.dpad_left &= other.dpad_left;
            this.dpad_right &= other.dpad_right;

            this.left_bumper &= other.left_bumper;
            this.right_bumper &= other.right_bumper;

            this.left_stick_button &= other.left_stick_button;
            this.right_stick_button &= other.right_stick_button;

            // Floats remain unchanged
        }

        //Auto generated function
        public boolean compareGamepad(GamepadStateEntry that) {
            return a == that.a &&
                    b == that.b &&
                    x == that.x &&
                    y == that.y &&
                    dpad_up == that.dpad_up &&
                    dpad_down == that.dpad_down &&
                    dpad_left == that.dpad_left &&
                    dpad_right == that.dpad_right &&
                    left_bumper == that.left_bumper &&
                    right_bumper == that.right_bumper &&
                    left_stick_button == that.left_stick_button &&
                    right_stick_button == that.right_stick_button &&
                    Math.abs(left_trigger - that.left_trigger) < 0.00001 &&
                    Math.abs(right_trigger - that.right_trigger) < 0.00001;
        }
    }

    public static class PoseStateEntry {
        public double x, y, heading;

        public PoseStateEntry(Pose pose) {
            this.x = pose.getX();
            this.y = pose.getY();
            this.heading = pose.getHeading();
        }

        public Pose toPose() {
            return new Pose(this.x, this.y, this.heading);
        }
    }

    public static class VelocityStateEntry {
        public double x, y;

        public VelocityStateEntry(Vector velocity) {
            this.x = velocity.getXComponent();
            this.y = velocity.getYComponent();
        }

        public Vector toVector() {
            Vector v = new Vector();
            v.setOrthogonalComponents(x,y);
            return v;
        }
    }

    public static class StateEntryJson {
        public int size = 0;
        public List<Double> timeListPose = new ArrayList<>();
        public List<Double> timeListGamepad = new ArrayList<>();
        public List<PoseStateEntry> poseList = new ArrayList<>();
        public List<VelocityStateEntry> velocityList = new ArrayList<>();
        public List<GamepadStateEntry> gamepad1List = new ArrayList<>();
        public List<GamepadStateEntry> gamepad2List = new ArrayList<>();


    }

    public static class PointerJson {
        public int pointer = 0;

        public PointerJson(int pointer){
            this.pointer = pointer;
        }
    }
}
