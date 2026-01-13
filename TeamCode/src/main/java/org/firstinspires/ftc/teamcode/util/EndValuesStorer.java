package org.firstinspires.ftc.teamcode.util;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;

public class EndValuesStorer {

    private static final String FILE_NAME = "endValues.json";
    private final Gson gson;

    public EndValuesStorer() {
        gson = new GsonBuilder().setPrettyPrinting().create();
    }

    private File getFile() {
        File dir = AppUtil.getInstance().getSettingsFile("TeamCodeLogs").getParentFile();
        return new File(dir, FILE_NAME);
    }

    // ---------------------------------------------------------
    // Save end-of-auto values to JSON
    // ---------------------------------------------------------
    public void saveEndValues(double robotX, double robotY, double robotHeading, int turretTicks, int indexerTicks) {
        EndValues values = new EndValues(robotX, robotY, robotHeading, turretTicks, indexerTicks);

        try (FileWriter writer = new FileWriter(getFile())) {
            gson.toJson(values, writer);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    // ---------------------------------------------------------
    // Load values back from JSON
    // ---------------------------------------------------------
    public EndValues loadEndValues() {
        File file = getFile();
        if (!file.exists()) {
            return null; // nothing saved yet
        }

        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
            return gson.fromJson(reader, EndValues.class);
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    // ---------------------------------------------------------
    // Data Model (Saved to JSON)
    // ---------------------------------------------------------
    public static class EndValues {
        public double x;
        public double y;
        public double heading;
        public int turretTicks;
        public int indexerTicks;

        public EndValues(double x, double y, double heading, int turretTicks, int indexerTicks) {
            this.x = x;
            this.y = y;
            this.heading = heading;
            this.turretTicks = turretTicks;
            this.indexerTicks = indexerTicks;
        }
    }
}
