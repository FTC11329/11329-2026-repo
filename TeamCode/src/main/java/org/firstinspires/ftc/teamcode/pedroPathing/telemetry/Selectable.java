package com.pedropathing.telemetry;

import org.jetbrains.annotations.NotNull;

/**
 * An item that can be selected
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Davis Luxenberg - 12649 Code Blooded
 * @noinspection unused
 */
public abstract class Selectable<T> {
    /**
     * The name of the {@link Selectable}. Shown in the select menu.
     */
    @NotNull
    public final String name;

    /**
     * Constructs a {@link Selectable} with the given name.
     *
     * @param name the name of the selectable
     */
    protected Selectable(@NotNull String name) {
        this.name = name;
    }
}

