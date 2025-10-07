package org.firstinspires.ftc.teamcode.pedroPathing.telemetry;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.Objects;
import java.util.stream.Collectors;

public final class FolderStack<T> {
    private final Deque<Folder<T>> folders = new ArrayDeque<>();
    private String breadcrumb = "";

    public String getBreadcrumb() {
        return breadcrumb;
    }

    public void push(Folder<T> folder) {
        folders.addLast(folder);
        breadcrumb += (Objects.equals(breadcrumb, "") ? "" : " / ") + folder.name;
    }

    public Folder<T> pop() {
        Folder<T> folder = folders.removeLast();

        breadcrumb = folders.stream()
                .map(f -> f.name)
                .collect(Collectors.joining(" / "));

        return folder;
    }

    public Folder<T> peek() {
        return folders.peekLast();
    }

    public int size() {
        return folders.size();
    }
}
