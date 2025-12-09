package org.firstinspires.ftc.teamcode.opmode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.LauncherOuttakeFuckingThing;
import org.firstinspires.ftc.teamcode.subsystem.Turret;
import org.firstinspires.ftc.teamcode.subsystem.VisionDistanceHelper;

import java.util.List;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "blueCornerSpam")
public class BlueCornerSpam extends NextFTCOpMode {

    public BlueCornerSpam() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Turret.INSTANCE),
                new SubsystemComponent(Intake.INSTANCE),
                new SubsystemComponent(LauncherOuttakeFuckingThing.INSTANCE),
                BindingsComponent.INSTANCE
        );
    }

    // ======================
    // PATH / POSE COMMANDS
    // ======================

    // 1) go grab first corner stack
    public static final Command grabfirst = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierCurve(
                                                new Pose(48.000, 7.000),
                                                new Pose(81.406, 42.284),
                                                new Pose(23.972, 35.292),
                                                new Pose(14.000, 36.000)
                                        )
                                )
                                .setLinearHeadingInterpolation(
                                        Math.toRadians(180),
                                        Math.toRadians(180)
                                )
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    // 2) score from first corner position
    public static final Command scorefirst = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                new Pose(14.000, 36.000),
                                                new Pose(55.000, 17.000)
                                        )
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(180))
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    // 3) grab from corner #1 again
    public static final Command grabfromcorner1 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierCurve(
                                                new Pose(55.000, 17.000),
                                                new Pose(40.620, 7.824),
                                                new Pose(10.000, 9.000)
                                        )
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(180))
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    // 4) score from corner #1
    public static final Command scorecorner1 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                new Pose(10.000, 9.000),
                                                new Pose(55.000, 17.000)
                                        )
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(180))
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    // 5) transition path toward second corner
    public static final Command Path5 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                new Pose(55.000, 17.000),
                                                new Pose(9.000, 20.000)
                                        )
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(180))
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    // 6) grab from second corner
    public static final Command grabcorener2 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierCurve(
                                                new Pose(9.000, 20.000),
                                                new Pose(22.141, 14.317),
                                                new Pose(9.000, 10.000)
                                        )
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(180))
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    // 7) final score from second corner
    public static final Command scorecorner3 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                new Pose(9.000, 10.000),
                                                new Pose(55.000, 17.000)
                                        )
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(180))
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());


    @Override
    public void onInit() {
        // Disable relocalization for this auto (same as artifact autos)
        VisionDistanceHelper.RELOCALIZATION_ENABLED = false;

        // Enable auto RPM/angle calculation + fallback for this auto
        LauncherOuttakeFuckingThing.INSTANCE.enableAutoCalculation();

        // Kick the Limelight once (matches your other autos)
        LLResult result = Turret.INSTANCE.runLimelight();
        Turret.INSTANCE.limelight.pipelineSwitch(1);

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
    }

    @Override
    public void onStartButtonPressed() {
        Follower follower = PedroComponent.follower();

        // Start pose for this corner auto: first path's start pose + heading 180
        follower.setPose(new Pose(48.0, 7.0, Math.toRadians(180)));

        // Skeleton auto sequence.
        // You can now drop in the exact Intake/Turret/Launcher logic you use
        // in blueArtifactSpamSTATIC around these commands.
        Command auto = new SequentialGroup(
                // Example: preload setup if you want it; copy your artifact spam here
                new LambdaCommand().setStart(() -> {
                    // Example shooter/intake prep – customize as needed
                    LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed);
                    LauncherOuttakeFuckingThing.INSTANCE.setFallback(2600, 27);
                    LauncherOuttakeFuckingThing.INSTANCE.enableAutoCalculation();
                    Turret.INSTANCE.enableAutoAim(true);
                }),

                grabfirst,
                scorefirst,
                grabfromcorner1,
                scorecorner1,
                Path5,
                grabcorener2,
                scorecorner3,


                new Delay(0.5)
        );

        auto.schedule();
    }
}
