package org.firstinspires.ftc.teamcode.pathfinding; // Change package if needed

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

public class GrabGateStorage {

    // ==========================================================
    // 1. THE POSES (Edit these numbers to tune ALL paths at once)
    // ==========================================================
    public static final Pose startingPose = new Pose(24.9, 122.000, Math.toRadians(90));

    // The starting position (scoring area)
    public static final Pose scorePose = new Pose(52.000, 83.000);
    public static final Pose curveScorePose = new Pose(22.76288659793814, 59.051546391752574);
    public static final Pose curveScorePose2 = new Pose(13.5257731958762894, 66.14432989690722);

    public static final Pose curvegatePose = new Pose(44.536, 66.309);

            ;


    // The intermediate spot before curving to the gate
    public static final Pose approachPose = new Pose(18.500, 62.500);

    // The curve control point (determines the shape of the turn)
    public static final Pose curveControlPose = new Pose(39.75257731958763, 59.21649484536082);

    // The point right in front of the sample
    public static final Pose preGrabPose = new Pose(13.000, 57.7500);

    // The final position where the grab actually happens
    public static final Pose grabPose = new Pose(12.00, 58.25);


    // ==========================================================
    // 2. THE PATHS (Returns standard Pedro PathChains)
    // ==========================================================
}
