package frc.robot.common;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Utils {
    /**
     * Clamps the value between a given range
     */
    public static double clamp(double value, double min, double max) {
        return Math.max(Math.min(value, max), min);
    }

    /**
     * Clamps the value between a given range
     */
    public static double clamp(double value, double range) {
        return Math.max(Math.min(value, range), -range);
    }

    /**
     * Rounds the number to a certain number of decimal places
     */
    public static double roundTo(double value, double decimals) {
        return Math.round(value*Math.pow(10, decimals))/Math.pow(10, decimals);
    }

    /**
     * Whether the particlar value is within a specified tolerance of the target
     */
    public static boolean isWithin(double value, double target, double tolerance) {
        return Math.abs(target - value) <= tolerance;
    }

    public static Pose2d flipPoseToRed(Pose2d pose, boolean flip) {
        Pose2d newPose = pose;
        if (flip) {
            newPose = new Pose2d(
                new Translation2d(Constants.FIELD.RED_WALL_X - pose.getX(), pose.getY()),
                pose.getRotation().minus(Rotation2d.fromDegrees(180))
            );
        }

        return newPose;
    }
}