package org.firstinspires.ftc.teamcode.TechnicalTerrors;

import com.acmerobotics.roadrunner.geometry.Pose2d;

/**
 * Simple static field serving as a storage medium for the bot's pose.
 * This allows different classes/opmodes to set and read from a central source of truth.
 * A static field allows data to persist between opmodes.
 */
public class PoseStorage {
    public static Pose2d currentPose = new Pose2d(14, -62, Math.toRadians(90));
}