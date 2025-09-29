package org.firstinspires.ftc.teamcode.pedroPathing.geometry;

/**
 * <p>A coordinate system, such as FTC standard coordinates or Pedro coordinates</p>
 * <br>
 * FTC standard coordinates are as defined on the
 * <a href="https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html">FTC Docs</a>.
 *
 * @author BeepBot99
 */
public interface CoordinateSystem {

    /**
     * Converts a {@link Pose} from this coordinate system to Pedro coordinates
     *
     * @param pose The {@link Pose} to convert, in this coordinate system
     * @return The converted {@link Pose}, in Pedro coordinates
     */
    Pose convertToPedro(Pose pose);

    /**
     * Converts a {@link Pose} from Pedro coordinates to this coordinate system
     *
     * @param pose The {@link Pose} to convert, in Pedro coordinates
     * @return The converted {@link Pose}, in this coordinate system
     */
    Pose convertFromPedro(Pose pose);
}