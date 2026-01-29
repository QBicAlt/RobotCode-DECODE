package org.firstinspires.ftc.teamcode.pathfinding; // Change package if needed

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

public class GrabGateStorage {

    // ==========================================================
    // 1. THE POSES (Edit these numbers to tune ALL paths at once)
    // ==========================================================
    public static final Pose startingPose = new Pose(24.700, 122.000, Math.toRadians(90));

    // The starting position (scoring area)
    public static final Pose scorePose = new Pose(57.000, 82.000);
    public static final Pose curveScorePose = new Pose(20.266, 51.985);
            ;


    // The intermediate spot before curving to the gate
    public static final Pose approachPose = new Pose(18.500, 64.500);

    // The curve control point (determines the shape of the turn)
    public static final Pose curveControlPose = new Pose(15.363068456914775, 61.137492471746896);

    // The point right in front of the sample
    public static final Pose preGrabPose = new Pose(13.000, 59.000);

    // The final position where the grab actually happens
    public static final Pose grabPose = new Pose(13.00, 59.7500);


    // ==========================================================
    // 2. THE PATHS (Returns standard Pedro PathChains)
    // ==========================================================
}
