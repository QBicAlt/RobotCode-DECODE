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

@Autonomous(name = "redCornerSpam")
public class redCornerSpam extends NextFTCOpMode {

    public redCornerSpam() {
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


    public static final Command movefirst = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(new Pose(48.000, 8.000).mirror(), new Pose(60.000, 25.000).mirror())
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(0))
                                .build(),
                        .7, false
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());


    public static final Command grabfirst_1 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierCurve(
                                                new Pose(60.000, 25.000).mirror(),
                                                new Pose(33.628, 14.816).mirror(),
                                                new Pose(8.000, 25.000).mirror()
                                        )
                                )
                                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270))
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    // grabfirst
    public static final Command grabfirst_2 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierLine(new Pose(8.000, 25.000).mirror(), new Pose(8.000, 10.000).mirror()))
                                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(270))
                                .build(),
                .3, false

                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    // 3) grab from corner #1 again
    public static final Command scoreFirst = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(new Pose(8.000, 10.000).mirror(),
                                                new Pose(60.000, 25.000).mirror())
                                )
                                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0))
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    // 4) score from corner #1
    public static final Command grabsecond = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(new Pose(60.000, 25.000).mirror(),
                                                new Pose(10.000, 15.000).mirror())
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(0))
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    // 5) transition path toward second corner
    public static final Command grabsecondwiggle = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierCurve(
                                                new Pose(10.000, 15.000).mirror(),
                                                new Pose(25.138, 19.477).mirror(),
                                                new Pose(9.822, 25.803).mirror()
                                                ))

                                .setConstantHeadingInterpolation(Math.toRadians(0))
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    // 6) grab from second corner
    public static final Command scoresecond= new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(new Pose(9.822, 25.803).mirror(),
                                                new Pose(60.000, 25.000).mirror())
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(0))
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    // 7) final score from second corner
    public static final Command grabthird = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(new Pose(60.000, 25.000).mirror(),
                                                new Pose(10.000, 10.000).mirror())

                                )
                                .setConstantHeadingInterpolation(Math.toRadians(0))
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());


    @Override
    public void onInit() {
        // Disable relocalization for this auto (same as artifact autos)
        Turret.INSTANCE.limelight.pipelineSwitch(0);
        VisionDistanceHelper.RELOCALIZATION_ENABLED = false;

        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed);


        Turret.INSTANCE.LIMELIGHT_X_OFFSET_DEG = -3;


                // Kick the Limelight once (matches your other autos)
        LLResult result = Turret.INSTANCE.runLimelight();
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
    }

    @Override
    public void onStartButtonPressed() {

        Follower follower = PedroComponent.follower();

        // Start pose for this corner auto: first path's start pose + heading 180
        follower.setPose(new Pose(48.0, 7.0, Math.toRadians(180)).mirror());


        // Skeleton auto sequence.
        // You can now drop in the exact Intake/Turret/Launcher logic you use
        // in blueArtifactSpamSTATIC around these commands.
        Command auto = new SequentialGroup(
                Intake.INSTANCE.indexerIn,

                // Example: preload setup if you want it; copy your artifact spam here
                new LambdaCommand().setStart(() -> {
                    // Example shooter/intake prep – customize as needed
                    LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed);
                    LauncherOuttakeFuckingThing.INSTANCE.setFallback(3600, 41.5);
                    LauncherOuttakeFuckingThing.INSTANCE.enableAutoCalculation();
                    Turret.INSTANCE.enableAutoAim(true);
                }),
                Intake.INSTANCE.intakeOneZero,
                Intake.INSTANCE.intakeTwoZero,
                movefirst,


        new Delay(1.5),
                Intake.INSTANCE.intakeOnePowerFull,
                Intake.INSTANCE.intakeTwoPowerFull,

                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)
                ),
                new Delay(1.5),
                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)
                ),
                grabfirst_1,
                grabfirst_2,
                new Delay(.2),
                Intake.INSTANCE.intakeTwoZero,
                scoreFirst,


                new LambdaCommand().setStart(() ->
                     LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)),
                new LambdaCommand().setStart(() ->
                        Turret.INSTANCE.snapToRememberedGoalAndEnable()
                ),
                new Delay(1),

                Intake.INSTANCE.intakeTwoPowerFull,

                new Delay(1.5),

                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)
                )

            //    grabfromcorner1,
              //  new Delay(.75),
                //Intake.INSTANCE.intakeTwoZero,
                //scorecorner1,
                //new Delay(.5),
                //new LambdaCommand().setStart(() ->
                  //      LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)),
                //new LambdaCommand().setStart(() ->
                    //    Turret.INSTANCE.snapToRememberedGoalAndEnable()
                //),
                //Intake.INSTANCE.intakeTwoPowerFull

             //   Path5,
               // grabcorener2,
                //scorecorner3,


        );

        auto.schedule();
    }
}
