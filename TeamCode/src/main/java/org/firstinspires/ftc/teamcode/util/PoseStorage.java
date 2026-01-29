package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;

public class PoseStorage {
    public static Pose currentPose = new Pose(0, 0, 0);
    public static boolean hasAutoRun = false;

    // This variable MUST be here. If it is missing, the app crashes.
    public static double turretEncoderTicks = 0.0;
}