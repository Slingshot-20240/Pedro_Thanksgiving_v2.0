package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.geometry.Pose;

public class LocalizationMath {

    // Goal coordinates
    public static final double RED_GOAL_X = 141;
    public static final double RED_GOAL_Y = 141;

    public static final double BLUE_GOAL_X = -141;
    public static final double BLUE_GOAL_Y = 141;


    /* ---------------------------------------------------------
     * Generic Helpers (distance, angle, heading error)
     * --------------------------------------------------------- */

    /** Distance (inches) to any goal */
    public static double getDistanceToGoal(Pose pose, double goalX, double goalY) {
        double dx = goalX - pose.getX();
        double dy = goalY - pose.getY();
        return Math.hypot(dx, dy);
    }

    /** Absolute angle to goal in degrees */
    public static double getAngleToGoal(Pose pose, double goalX, double goalY) {
        double dx = goalX - pose.getX();
        double dy = goalY - pose.getY();
        return Math.toDegrees(Math.atan2(dy, dx));
    }

    /** Heading error in degrees (-180 to +180) */
    public static double getHeadingError(Pose pose, double goalX, double goalY) {
        double angleToGoal = getAngleToGoal(pose, goalX, goalY);
        double headingDeg = Math.toDegrees(pose.getHeading()); // Pedro returns radians

        double error = angleToGoal - headingDeg;

        // normalize
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        return error;
    }


    /* ---------------------------------------------------------
     * Red Goal Helpers
     * --------------------------------------------------------- */

    public static double getRedDistance(Pose pose) {
        return getDistanceToGoal(pose, RED_GOAL_X, RED_GOAL_Y);
    }

    public static double getRedHeadingError(Pose pose) {
        return getHeadingError(pose, RED_GOAL_X, RED_GOAL_Y);
    }


    /* ---------------------------------------------------------
     * Blue Goal Helpers
     * --------------------------------------------------------- */

    public static double getBlueDistance(Pose pose) {
        return getDistanceToGoal(pose, BLUE_GOAL_X, BLUE_GOAL_Y);
    }

    public static double getBlueHeadingError(Pose pose) {
        return getHeadingError(pose, BLUE_GOAL_X, BLUE_GOAL_Y);
    }
}
