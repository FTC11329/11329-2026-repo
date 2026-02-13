package org.firstinspires.ftc.teamcode.pedroPathing.telemetry;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import com.pedropathing.telemetry.Selectable;

public final class SelectScope<T> {
    private final List<Selectable<T>> selectables = new ArrayList<>();

    public List<Selectable<T>> getSelectables() {
        return new ArrayList<>(selectables);
    }

    public void add(String name, T value) {
        selectables.add(new Item<>(name, value));
    }

    public void folder(String name, Consumer<SelectScope<T>> children) {
        SelectScope<T> scope = new SelectScope<>();
        children.accept(scope);
        selectables.add(new Folder<>(name, scope.getSelectables()));
    }
}
