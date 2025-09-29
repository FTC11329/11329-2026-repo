package org.firstinspires.ftc.teamcode.pedroPathing.math;

import java.util.Map;

/**
 * A bijective map interface that allows for a one-to-one mapping between keys and values.
 * This interface provides methods to put, get, invert, check existence, remove entries,
 * and retrieve the forward and reverse maps.
 *
 * @param <T> the type of keys in the map
 * @param <S> the type of values in the map
 *
 * @author Havish Sripada - 12808 RevAmped Robotics
 */
public interface BijectiveMap<T, S> {
    /**
     * Puts a key-value pair into the bijective map.
     *
     * @param key   the key to put
     * @param value the value to put
     */
    void put(T key, S value);

    /**
     * Gets the value associated with the given key.
     *
     * @param key the key to look up
     * @return the value associated with the key, or null if not found
     */
    S get(T key);

    /**
     * Inverts the value to get the corresponding key.
     *
     * @param value the value to invert
     * @return the key associated with the value, or null if not found
     */
    T invert(S value);

    /**
     * Checks if the map contains the specified key.
     *
     * @param key the key to check
     * @return true if the map contains the key, false otherwise
     */
    boolean containsKey(T key);

    /**
     * Checks if the map contains the specified value.
     *
     * @param value the value to check
     * @return true if the map contains the value, false otherwise
     */
    boolean containsValue(S value);

    /**
     * Removes the key-value pair associated with the specified key.
     *
     * @param key the key to remove
     */
    void remove(T key);

    /**
     * Removes the key-value pair associated with the specified value.
     *
     * @param value the value to remove
     */
    void removeValue(S value);

    /**
     * Gets the size of the bijective map.
     *
     * @return the number of key-value pairs in the map
     */
    int size();

    /**
     * Gets the forward map from keys to values.
     *
     * @return the forward map
     */
    Map<T, S> getForwardMap();

    /**
     * Gets the reverse map from values to keys.
     *
     * @return the reverse map
     */
    Map<S, T> getReverseMap();
}
