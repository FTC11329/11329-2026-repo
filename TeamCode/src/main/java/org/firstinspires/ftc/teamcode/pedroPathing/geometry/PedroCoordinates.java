package org.firstinspires.ftc.teamcode.pedroPathing.geometry;

/**
 * <p>A coordinate system, such as FTC standard coordinates or Pedro coordinates</p>
 * <br>
 * Pedro coordinates are a coordinate system used by the Pedro Pathing library.
 * It is a 144x144 coordinate system with the origin at (72, 72),
 * with a heading of 0 degrees facing the red alliance wall from the center of the field.
 *
 * @author BeepBot99
 * @author Baron Henderson - 20077 The Indubitables
 */
public enum PedroCoordinates implements CoordinateSystem {
    INSTANCE;

    /**
     * Converts a {@link Pose} to this coordinate system from Pedro coordinates
     *
     * @param pose The {@link Pose} to convert, in the Pedro coordinate system
     * @return The converted {@link Pose}, in Pedro coordinates
     */
    @Override
    public Pose convertFromPedro(Pose pose) {
        return pose;
    }

    /**
     * Converts a {@link Pose} to Pedro coordinates from this coordinate system
     *
     * @param pose The {@link Pose} to convert, in Pedro coordinates
     * @return The converted {@link Pose}, in Pedro coordinate system
     */
    @Override
    public Pose convertToPedro(Pose pose) {
        return pose;
    }
}
