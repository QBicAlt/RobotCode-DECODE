package org.firstinspires.ftc.teamcode.util;

import static java.lang.Math.PI;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import dev.nextftc.core.commands.Command;
import dev.nextftc.ftc.Gamepads;

public class LimelightManager {
    private final Limelight3A limelight;

    public LimelightManager(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(4);
        limelight.start();
    }

    public Pose getPose(double headingRadians) {
        LLResult result = limelight.getLatestResult();
        limelight.updateRobotOrientation(Math.toDegrees(headingRadians + PI/2));

        if (result != null && result.isValid() && result.getStaleness() < 200) {
            Pose3D llPose = result.getBotpose_MT2();
            if (llPose != null) {
                return new Pose(
                        72 + llPose.getPosition().toUnit(DistanceUnit.INCH).y,
                        72 - llPose.getPosition().toUnit(DistanceUnit.INCH).x,
                        headingRadians
                );
            }
        }
        return null;
    }

    // --- FIXED COMMAND FACTORY ---
    public Command relocalize(Follower follower) {
        return new Command() {
            private final ElapsedTime timer = new ElapsedTime();
            private double sumX = 0;
            private double sumY = 0;
            private int count = 0;

            // CHANGED: onStart -> start
            @Override
            public void start() {
                timer.reset();
                sumX = 0;
                sumY = 0;
                count = 0;
// Instead of rumble(200)
                Gamepads.gamepad1().getGamepad().invoke().rumble(200);
            }

            // CHANGED: onUpdate -> update
            @Override
            public void update() {
                double currentHeading = follower.getPose().getHeading();
                Pose instantPose = getPose(currentHeading);

                if (instantPose != null) {
                    sumX += instantPose.getX();
                    sumY += instantPose.getY();
                    count++;
                }
            }

            @Override
            public boolean isDone() {
                return timer.milliseconds() >= 200;
            }

            // CHANGED: onStop -> stop(boolean interrupted)
            @Override
            public void stop(boolean interrupted) {
                if (count > 0) {
                    double avgX = sumX / count;
                    double avgY = sumY / count;

                    follower.setPose(new Pose(avgX, avgY, follower.getPose().getHeading()));

                    Gamepads.gamepad1().getGamepad().invoke().rumble(200);
                }
            }
        };
    }
}