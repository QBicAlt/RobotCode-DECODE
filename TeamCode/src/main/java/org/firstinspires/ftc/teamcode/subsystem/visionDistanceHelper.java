package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
@Config
public class visionDistanceHelper {

    // --- Tag 24 field position (from fmap), in your Pedro field frame, inches ---
    // fmap: tx = -1.4827 m, ty = 1.4133 m
    // Your code uses: x_in = inches(LL_y), y_in = -inches(LL_x)
    private static final double TAG24_LL_X_M = -1.4827;
    private static final double TAG24_LL_Y_M =  1.4133;

    public static final double GOAL_TAG_X_IN =
            DistanceUnit.INCH.fromMeters(TAG24_LL_Y_M);      // ≈ 55.64 in

    public static final double GOAL_TAG_Y_IN =
            -DistanceUnit.INCH.fromMeters(TAG24_LL_X_M);     // ≈ 58.37 in

    // Distance from turret pivot (robot center) to Limelight lens, in inches.
    // TODO: measure this on your bot and update
    public static double CAMERA_RADIUS_IN = 4.704;

    /**
     * Convert Limelight MegaTag2 pose (treated as CAMERA pose) + turret angle
     * into robot-center pose in Pedro's coordinate system.
     *
     * @param result          Latest Limelight result (must be valid & MT2 enabled)
     * @param turretAngleDeg  turret angle relative to robot forward, in degrees
     * @return Pedro Pose (x, y, heading) of ROBOT CENTER, or null if result invalid
     */
    public static Pose robotPoseFromLimelight(LLResult result, double turretAngleDeg) {
     if (result == null || !result.isValid()) {
     return null;
     }

     Pose3D camPose = result.getBotpose_MT2();   // with LL Forward/Right = 0, this is CAMERA pose
     if (camPose == null) {
     return null;
     }

          // Match your Pedro frame:
        //  fieldX = inches(LL_y)
          //  fieldY = -inches(LL_x)
     double camX = DistanceUnit.INCH.fromMeters(camPose.getPosition().y);
     double camY = -DistanceUnit.INCH.fromMeters(camPose.getPosition().x);

          // Camera yaw in field coords (radians)
     double thetaCam = camPose.getOrientation().getYaw(AngleUnit.RADIANS);

          // Turret angle relative to robot forward (degrees -> radians)
     double turretAngleRad = -Math.toRadians(turretAngleDeg);

          // Robot heading = camera heading - turret angle
     double thetaRobot = thetaCam - turretAngleRad;

          // Robot center is CAMERA minus offset along ROBOT heading
     double xRobot = camX - CAMERA_RADIUS_IN * Math.cos(thetaRobot);
     double yRobot = camY - CAMERA_RADIUS_IN * Math.sin(thetaRobot);

     return new Pose(xRobot, yRobot, thetaRobot);
     }

    /**
     * Distance from a given robot-center pose to goal tag 24, in inches.
     * Works with either Pedro pose or LL-derived pose, as long as they're in the
     * same field frame.
     */
    public static double distanceToGoalInches(Pose robotPose) {
        if (robotPose == null) {
            return Double.NaN;
        }

        double dx = GOAL_TAG_X_IN - robotPose.getX();
        double dy = GOAL_TAG_Y_IN - robotPose.getY();
        return Math.hypot(dx, dy);
    }

    /**
     * Convenience: get distance directly from Limelight + turret angle.
     * Use this when tags are visible (calibration, relocalization, etc.).
     */
    public static double distanceToGoalFromLimelight(LLResult result, double turretAngleDeg) {
        Pose robotPose = robotPoseFromLimelight(result, turretAngleDeg);
        return distanceToGoalInches(robotPose);
    }
}
