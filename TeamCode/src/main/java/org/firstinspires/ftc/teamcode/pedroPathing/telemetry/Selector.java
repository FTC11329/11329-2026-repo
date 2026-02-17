package org.firstinspires.ftc.teamcode.pedroPathing.telemetry;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Consumer;
import org.firstinspires.ftc.teamcode.pedroPathing.telemetry.Selectable;

public final class Selector<T> {
    private final FolderStack<T> folders = new FolderStack<>();
    private final String[] message;

    public Selector(Folder<T> rootFolder, String[] message) {
        folders.push(rootFolder);
        this.message = message;
    }

    public Selector(Folder<T> rootFolder, String message) {
        this(rootFolder, new String[]{message});
    }

    public Selector(Folder<T> rootFolder) {
        this(rootFolder, new String[]{});
    }

    public static <T> Selector<T> create(String name, Consumer<SelectScope<T>> children,
                                         String[] message) {
        SelectScope<T> scope = new SelectScope<>();
        children.accept(scope);
        return new Selector<>(new Folder<>(name, scope.getSelectables()), message);
    }

    public static <T> Selector<T> create(String name, Consumer<SelectScope<T>> children,
                                         String message) {
        return create(name, children, new String[]{message});
    }

    public static <T> Selector<T> create(String name, Consumer<SelectScope<T>> children) {
        return create(name, children, new String[]{});
    }

    public List<String> getLines() {
        List<String> lines = new ArrayList<>();
        lines.add(folders.getBreadcrumb());
        Collections.addAll(lines, message);
        lines.add("------------");
        lines.addAll(folders.peek().getLines());
        return lines;
    }

    public void incrementSelected() {
        folders.peek().incrementSelected();
    }

    public void decrementSelected() {
        folders.peek().decrementSelected();
    }

    private Consumer<T> onSelect = item -> {
    };

    public void onSelect(Consumer<T> callback) {
        onSelect = callback;
    }

    public void select() {
        Selectable<T> selectable = folders.peek().getSelected();
        if (selectable instanceof Folder) {
            folders.push((Folder<T>) selectable);
        } else if (selectable instanceof Item) {
            onSelect.accept(((Item<T>) selectable).value);
        } else throw new RuntimeException("Selected Selectable is not a Folder or an Item!");
    }

    public void goBack() {
        if (folders.size() > 1) folders.pop();
    }
}
