package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Config
public class VisionDistanceHelper {

    // Pedro coords of the GOAL tag (inches) – tune these once it’s roughly right
    public static double GOAL_TAG_X_IN = 127.64;
    public static double GOAL_TAG_Y_IN =130.37 ;

    // Distance from robot center (turret pivot) to Limelight lens in inches
    public static double CAMERA_RADIUS_IN = 4.704;

    // Pedro field center (tiles are 144x144 in)
    public static final double FIELD_CENTER_X = 72.0;
    public static final double FIELD_CENTER_Y = 72.0;

    /**
     * Relocalize Pedro:
     *  - LL MT1 botpose -> field coords (center origin)
     *  - manual map -> Pedro coords (0..144, 0..144)
     *  - turret trig -> robot center
     *  - overwrite X/Y only, keep heading from Pedro/IMU
     */
    public static void relocalizePedroFromLimelight(
            Follower follower,
            LLResult result,
            double turretAngleDeg
    ) {
        Pose llRobotPedro = robotPoseFromLimelight(result, turretAngleDeg);
        if (llRobotPedro == null) return;

        Pose odoPose = follower.getPose();

        follower.setPose(new Pose(
                llRobotPedro.getX(),
                llRobotPedro.getY(),
                odoPose.getHeading()       // keep heading from odometry
        ));
    }

    /**
     * MT1 + turret:
     *  - use botpose as CAMERA pose in FMap/FTC center-origin
     *  - map to Pedro coords explicitly
     *  - use turret angle + CAMERA_RADIUS_IN to get ROBOT CENTER
     */
    public static Pose robotPoseFromLimelight(LLResult result, double turretAngleDeg) {
        if (result == null || !result.isValid()) {
            return null;
        }

        Pose3D camPose3D = result.getBotpose();  // MT1
        if (camPose3D == null) {
            return null;
        }

        // === 1) Raw DECODE/FMap FTC coords (center origin), meters ===
        double llX_m = camPose3D.getPosition().x;
        double llY_m = camPose3D.getPosition().y;
        double llYaw_rad = camPose3D.getOrientation().getYaw(AngleUnit.RADIANS);

        // meters -> inches
        double llX_in = DistanceUnit.INCH.fromMeters(llX_m);
        double llY_in = DistanceUnit.INCH.fromMeters(llY_m);

        // === 2) Map to "field X/Y" like you had before ===
        // Your friend & you both use:
        //   fieldX = inches(LL_y)
        //   fieldY = -inches(LL_x)
        double fieldX = DistanceUnit.INCH.fromMeters(llY_m);   // left/right
        double fieldY = -DistanceUnit.INCH.fromMeters(llX_m);  // up/down field

        // === 3) Center-origin -> Pedro 0..144, 0..144 ===
        double camX_pedro = FIELD_CENTER_X + fieldX;
        double camY_pedro = FIELD_CENTER_Y + fieldY;

        // Heading: LL’s yaw is already CCW+, just treat it as field heading.
        double thetaCam = llYaw_rad;

        // === 4) Turret trig: camera -> robot center ===
        double turretAngleRad = Math.toRadians(turretAngleDeg);  // keep sign consistent with your turret code

        // Robot heading = camera heading - turret angle
        double thetaRobot = thetaCam - turretAngleRad;

        // Robot center is CAMERA minus offset along ROBOT heading
        double xRobot = camX_pedro - CAMERA_RADIUS_IN * Math.cos(thetaRobot);
        double yRobot = camY_pedro - CAMERA_RADIUS_IN * Math.sin(thetaRobot);

        return new Pose(xRobot, yRobot, thetaRobot);
    }

    public static double distanceToGoalInches(Pose robotPose) {
        if (robotPose == null) return Double.NaN;

        double dx = GOAL_TAG_X_IN - robotPose.getX();
        double dy = GOAL_TAG_Y_IN - robotPose.getY();
        return Math.hypot(dx, dy);
    }

    public static double distanceToGoalFromLimelight(LLResult result, double turretAngleDeg) {
        Pose robotPosePedro = robotPoseFromLimelight(result, turretAngleDeg);
        return distanceToGoalInches(robotPosePedro);
    }
}
