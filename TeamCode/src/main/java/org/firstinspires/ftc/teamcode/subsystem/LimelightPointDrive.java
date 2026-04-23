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

    private final double CAMERA_HEIGHT = 9.7;
    private final double MOUNT_ANGLE = 0.0;

    private final double FIXED_X_BLUE = 10.0;
    private final double STANDOFF_X_BLUE = 38.0;
    private final double HEADING_BLUE = Math.toRadians(180);
    private final double HEADING_RED = Math.toRadians(0);

    private final double MIN_Y = 7.0;
    private final double MAX_Y = 58.0;
    private final double FALLBACK_Y = 9.0;
    private final double FIELD_WIDTH = 144.0;

    private Limelight3A limelight;
    private ElapsedTime timer = new ElapsedTime();

    private ElapsedTime pathDurationTimer = new ElapsedTime();
    private ElapsedTime stuckTimer = new ElapsedTime();

    private double sumTx = 0;
    private double sumTy = 0;
    private int sampleCount = 0;

    private boolean pathGenerated = false;
    private boolean isRed;

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
        Follower follower = PedroComponent.follower();

        if (timer.milliseconds() < 200) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                sumTx += result.getTx();
                sumTy += result.getTy();
                sampleCount++;
            }
            return;
        }

        if (!pathGenerated) {
            Pose currentPose = follower.getPose();
            double targetY;

            double targetFixedX;
            double targetStandoffX;
            double targetHeading;

            if (isRed) {
                targetFixedX = FIELD_WIDTH - FIXED_X_BLUE;
                targetStandoffX = FIELD_WIDTH - STANDOFF_X_BLUE;
                targetHeading = HEADING_RED;
            } else {
                targetFixedX = FIXED_X_BLUE;
                targetStandoffX = STANDOFF_X_BLUE;
                targetHeading = HEADING_BLUE;
            }

            if (sampleCount > 0) {
                double avgTx = sumTx / sampleCount;
                double avgTy = sumTy / sampleCount;
                double angleToGoalDepression = MOUNT_ANGLE - avgTy;
                if (angleToGoalDepression < 1.0) angleToGoalDepression = 1.0;

                double distanceToArtifact = CAMERA_HEIGHT / Math.tan(Math.toRadians(angleToGoalDepression));
                double absoluteAngle = currentPose.getHeading() - Math.toRadians(avgTx);
                double artifactY = currentPose.getY() + Math.sin(absoluteAngle) * distanceToArtifact;

                targetY = Range.clip(artifactY, MIN_Y, MAX_Y);
            } else {
                targetY = FALLBACK_Y;
            }

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

            pathDurationTimer.reset();
            stuckTimer.reset();
            pathGenerated = true;
        }


        if (pathGenerated && follower.isBusy()) {
            double speed = follower.getVelocity().getMagnitude();


            if (speed > 2.0 || pathDurationTimer.seconds() < 0.5) {
                stuckTimer.reset();
            }

            if (stuckTimer.seconds() > 0.5) {
                follower.breakFollowing();

            }
        }
    }

    @Override
    public boolean isDone() {
        return pathGenerated && !PedroComponent.follower().isBusy();
    }
}