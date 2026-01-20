package org.firstinspires.ftc.teamcode.subsystem;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.core.commands.Command;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;

public class LimelightPointDrive extends Command {

    // --- CAMERA GEOMETRY ---
    private final double CAMERA_HEIGHT = 9.7;
    private final double MOUNT_ANGLE = 15.0;

    // --- CONFIGURATION ---
    // These are the "Blue" values.
    private final double FIXED_X_BLUE = 10.0;
    private final double STANDOFF_X_BLUE = 38.0;

    // Headings
    private final double HEADING_BLUE = Math.toRadians(180); // Facing Blue Wall (Left)
    private final double HEADING_RED = Math.toRadians(0);    // Facing Red Wall (Right)

    // Y Constraints (SHARED for both sides in this version)
    private final double MIN_Y = 7.0;
    private final double MAX_Y = 58.0;
    private final double FALLBACK_Y = 9.0;

    private final double FIELD_WIDTH = 144.0;

    // --- VARIABLES ---
    private Limelight3A limelight;
    private ElapsedTime timer = new ElapsedTime();

    private double sumTx = 0;
    private double sumTy = 0;
    private int sampleCount = 0;

    private boolean pathGenerated = false;
    private boolean isRed;

    /**
     * @param isRed Set to true to Mirror X and Heading, but keep Y the same.
     */
    public LimelightPointDrive(boolean isRed) {
        this.isRed = isRed;
    }

    @Override
    public void start() {
        pathGenerated = false;
        sumTx = 0;
        sumTy = 0;
        sampleCount = 0;

        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(3);

        PedroComponent.follower().breakFollowing();
        timer.reset();
    }

    @Override
    public void update() {
        // PHASE 1: COLLECT DATA
        if (timer.milliseconds() < 200) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                sumTx += result.getTx();
                sumTy += result.getTy();
                sampleCount++;
            }
            return;
        }

        // PHASE 2: GENERATE PATH
        if (!pathGenerated) {
            Follower follower = PedroComponent.follower();
            Pose currentPose = follower.getPose();
            double targetY;

            // --- 1. SET TARGETS BASED ON ALLIANCE ---
            double targetFixedX;
            double targetStandoffX;
            double targetHeading;

            if (isRed) {
                // Mirror X (144 - BlueX), Flip Heading (0), KEEP Y THE SAME
                targetFixedX = FIELD_WIDTH - FIXED_X_BLUE;       // 144 - 10 = 134
                targetStandoffX = FIELD_WIDTH - STANDOFF_X_BLUE; // 144 - 38 = 106
                targetHeading = HEADING_RED;
            } else {
                // Standard Blue
                targetFixedX = FIXED_X_BLUE;
                targetStandoffX = STANDOFF_X_BLUE;
                targetHeading = HEADING_BLUE;
            }

            if (sampleCount > 0) {
                double avgTx = sumTx / sampleCount;
                double avgTy = sumTy / sampleCount;

                // --- 2. CALCULATE Y ---
                // Note: This math works for both sides automatically provided the Heading is correct.
                // On Blue (180 deg), a positive tx (Right) increases Y.
                // On Red (0 deg), a negative tx (Left) increases Y.

                double angleToGoalDepression = MOUNT_ANGLE - avgTy;
                if (angleToGoalDepression < 1.0) angleToGoalDepression = 1.0;

                double distanceToArtifact = CAMERA_HEIGHT / Math.tan(Math.toRadians(angleToGoalDepression));
                double absoluteAngle = currentPose.getHeading() - Math.toRadians(avgTx);
                double artifactY = currentPose.getY() + Math.sin(absoluteAngle) * distanceToArtifact;

                // Clip to the SAME Y range for both alliances
                targetY = Range.clip(artifactY, MIN_Y, MAX_Y);
            } else {
                targetY = FALLBACK_Y;
            }

            // --- 3. BUILD PATH ---
            Pose standoffPose = new Pose(targetStandoffX, targetY, targetHeading);
            Pose finalPose = new Pose(targetFixedX, targetY, targetHeading);

            follower.followPath(
                    follower.pathBuilder()
                            .addPath(new BezierLine(currentPose, standoffPose))
                            .setConstantHeadingInterpolation(targetHeading)
                            .addPath(new BezierLine(standoffPose, finalPose))
                            .setConstantHeadingInterpolation(targetHeading)
                            .build(),
                    false
            );

            pathGenerated = true;
        }
    }

    @Override
    public boolean isDone() {
        return pathGenerated && !PedroComponent.follower().isBusy();
    }
}