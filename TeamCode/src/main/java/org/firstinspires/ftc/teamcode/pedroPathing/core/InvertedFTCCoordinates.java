package org.firstinspires.ftc.teamcode.pedroPathing.core;

import org.firstinspires.ftc.teamcode.pedroPathing.geometry.CoordinateSystem;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.PedroCoordinates;
import org.firstinspires.ftc.teamcode.pedroPathing.geometry.Pose;

/**
 * An enum that contains an inverted FTC standard coordinate system.
 * This implementation performs numeric transforms directly on the Pose components
 * to avoid calling {@code Pose} methods that may themselves trigger coordinate conversions.
 *
 * @author BeepBot99
 * @author Baron Henderson
 */
public enum InvertedFTCCoordinates implements CoordinateSystem {
    INSTANCE;

    /**
     * Converts a {@link Pose} to this coordinate system from Pedro coordinates
     *
     * @param pose The {@link Pose} to convert, in the Pedro coordinate system
     * @return The converted {@link Pose}, in FTC standard coordinates
     */
    @Override
    public Pose convertFromPedro(Pose pose) {
        Pose newPose = pose.minus(new Pose(72, 72)).rotate(Math.PI / 2, true);
        return new Pose(newPose.getX(), newPose.getY(), newPose.getHeading(), INSTANCE);
    }

    /**
     * Converts a {@link Pose} to Pedro coordinates from this coordinate system
     *
     * @param pose The {@link Pose} to convert, in FTC standard coordinates
     * @return The converted {@link Pose}, in Pedro coordinate system
     */
    @Override
    public Pose convertToPedro(Pose pose) {
        Pose newPose = new Pose (pose.getX(), pose.getY(), pose.getHeading(), PedroCoordinates.INSTANCE);
        return newPose.rotate(Math.PI / 2, true).plus(new Pose(72, 72));
    }
}