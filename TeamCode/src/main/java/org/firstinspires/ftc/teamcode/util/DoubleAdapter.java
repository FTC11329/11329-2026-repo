package org.firstinspires.ftc.teamcode.util;

import com.google.gson.TypeAdapter;
import com.google.gson.stream.JsonReader;
import com.google.gson.stream.JsonWriter;

import java.io.IOException;
import java.text.DecimalFormat;

public class DoubleAdapter extends TypeAdapter<Double> {
    private final DecimalFormat format = new DecimalFormat("#.###"); // 3 decimal places

    @Override
    public void write(JsonWriter out, Double value) throws IOException {
        if (value == null) {
            out.nullValue();
            return;
        }
        out.value(Double.parseDouble(format.format(value)));
    }

    @Override
    public Double read(JsonReader in) throws IOException {
        return in.nextDouble();
    }
}
