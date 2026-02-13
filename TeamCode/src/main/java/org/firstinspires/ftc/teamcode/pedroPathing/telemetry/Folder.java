package org.firstinspires.ftc.teamcode.pedroPathing.telemetry;

import java.util.ArrayList;
import java.util.List;
import com.pedropathing.telemetry.Selectable;

public final class Folder<T> extends Selectable<T> {
    private final List<Selectable<T>> children;
    private int selectedIndex = 0;

    public Selectable<T> getSelected() {
        return children.get(selectedIndex);
    }

    public void incrementSelected() {
        if (selectedIndex < children.size() - 1) selectedIndex++;
    }

    public void decrementSelected() {
        if (selectedIndex > 0) selectedIndex--;
    }

    /**
     * Constructor for Folder with children and parent
     */
    public Folder(String name, List<Selectable<T>> children) {
        super(name);
        this.children = children;
    }

    public List<String> getLines() {
        List<String> lines = new ArrayList<>();
        for (int i = 0; i < children.size(); i++) {
            Selectable<T> child = children.get(i);
            String listName;
            if (child instanceof Item) listName = child.name;
            else listName = child.name + " [...]";
            lines.add((i == selectedIndex ? "> " : "  ") + listName);
        }
        return lines;
    }
}
