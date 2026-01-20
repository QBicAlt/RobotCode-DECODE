package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

@Config
public class VisionDistanceHelper {

    // --- GOAL TARGET COORDINATES (Pedro System) ---
    // Standard Close Range Target
    public static double GOAL_TARGET_X = 132.0;
    public static double GOAL_TARGET_Y = 136.0;

    // --- NEW: LONG RANGE TARGET COORDINATES ---
    // When far away, switch aim to these coordinates
    public static double GOAL_TARGET_X_FAR = 138.0; // Tune this
    public static double GOAL_TARGET_Y_FAR = 144.0; // Tune this

    // Distance (inches) at which to switch to the "Far" target
    public static double AIM_SWITCH_THRESHOLD = 104.0;

    // Used for calculating distance (Static Tag Location)
    public static double GOAL_TAG_X_IN = 144 - 127.64;
    public static double GOAL_TAG_Y_IN = 130.37;

    /**
     * Calculates the distance from the robot center to the static Goal Tag.
     */
    public static double distanceToGoalInches(Pose robotPose) {
        if (robotPose == null) return 0.0;

        double dx = GOAL_TAG_X_IN - robotPose.getX();
        double dy = GOAL_TAG_Y_IN - robotPose.getY();
        return Math.hypot(dx, dy);
    }

    /**
     * Calculates the Field-Centric angle (in Radians) the robot needs to look at to see the goal.
     * Swaps targets based on distance.
     */
    public static double getAngleToGoalRad(Pose robotPose) {
        if (robotPose == null) return 0.0;

        // 1. Determine which target to use based on distance
        double currentDist = distanceToGoalInches(robotPose);
        double targetX, targetY;

        if (currentDist > AIM_SWITCH_THRESHOLD) {
            targetX = GOAL_TARGET_X_FAR;
            targetY = GOAL_TARGET_Y_FAR;
        } else {
            targetX = GOAL_TARGET_X;
            targetY = GOAL_TARGET_Y;
        }

        // 2. Calculate Angle
        double dx = targetX - robotPose.getX();
        double dy = targetY - robotPose.getY();

        // atan2 gives the angle from the X-axis to the vector pointing at the goal
        return Math.atan2(dy, dx);
    }
}