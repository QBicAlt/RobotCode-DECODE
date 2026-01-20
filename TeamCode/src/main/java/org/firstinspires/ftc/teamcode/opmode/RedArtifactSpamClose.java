package org.firstinspires.ftc.teamcode.opmode;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.LauncherOuttakeFuckingThing;
import org.firstinspires.ftc.teamcode.subsystem.Turret;
import org.firstinspires.ftc.teamcode.subsystem.VisionDistanceHelper;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "redArtifactSpamClose")
public class RedArtifactSpamClose extends NextFTCOpMode {

    public RedArtifactSpamClose() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Turret.INSTANCE),
                new SubsystemComponent(Intake.INSTANCE),
                new SubsystemComponent(LauncherOuttakeFuckingThing.INSTANCE),
                BindingsComponent.INSTANCE
        );
    }

    public static final Command scoreFirst1 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierLine(new Pose(17.8, 118).mirror(), new Pose(48.000, 96.000).mirror()))
                                .setTangentHeadingInterpolation()
                                .setReversed()
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    public static final Command grabfirst2 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierCurve(new Pose(48.000, 96.000).mirror(), new Pose(69.753, 81.739).mirror(), new Pose(19.000, 84.000).mirror()))
                                .setConstantHeadingInterpolation(Math.toRadians(0))
                                .build(),
                        .8, false
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());


    public static final Command OpenGateFirst2 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierCurve(new Pose(19.000, 84.000).mirror(), new Pose(37.124, 76.911).mirror(), new Pose(17.815, 71.000).mirror()))
                                .setConstantHeadingInterpolation(Math.toRadians(0))
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    public static final Command scorefirst3 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierLine(new Pose(17.815, 71.000).mirror(), new Pose(55.000, 84.000).mirror()))
                                .setConstantHeadingInterpolation(Math.toRadians(0))
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    public static final Command grabsecond4 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierCurve(new Pose(55.000, 84.000).mirror(), new Pose(59.598, 52.772).mirror(), new Pose(40.786, 60.264).mirror(), new Pose(21.309, 60.097).mirror(), new Pose(43.616, 58.765).mirror(), new Pose(15, 55).mirror()))
                                .setConstantHeadingInterpolation(Math.toRadians(0))
                                .build(),
                        .8, false
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    public static final Command scoresecond5 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierLine(new Pose(15.000, 55).mirror(), new Pose(55.000, 84.000).mirror()))
                                .setConstantHeadingInterpolation(Math.toRadians(0))
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    public static final Command grabthird1_7 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierCurve(new Pose(55.000, 84.000).mirror(), new Pose(63.760, 36.791).mirror(), new Pose(44.615, 35.292).mirror()))
                                .setTangentHeadingInterpolation()
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    public static final Command grabthird2_8 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierLine(new Pose(44.615, 35.292).mirror(), new Pose(15.487, 35.292).mirror()))
                                .setTangentHeadingInterpolation()
                                .build(),
                        .8, false);
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    public static final Command scorethird = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierLine(new Pose(11.487, 35.292).mirror(), new Pose(55.000, 84.000).mirror()))
                                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(60))
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    public static final Command move = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierLine(new Pose(55, 84).mirror(), new Pose(55.000, 40.000).mirror()))
                                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());


    @Override
    public void onInit() {
        // IMPORTANT: Zero Encoders
        Turret.INSTANCE.resetEncoderLogic();
        Turret.INSTANCE.enableOdometryAim();
        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed);

        // Set RED Goal Coordinates
        VisionDistanceHelper.GOAL_TARGET_X =  127.64;
        VisionDistanceHelper.GOAL_TARGET_Y = 130.37;
    }

    @Override
    public void onStartButtonPressed() {
        LauncherOuttakeFuckingThing.INSTANCE.enableAutoCalculation();
        LauncherOuttakeFuckingThing.autoCalculate = true;

        Follower follower = PedroComponent.follower();
        follower.setPose(new Pose(17.8, 118, Math.toRadians(144)).mirror());

        Command auto = new SequentialGroup(
                Intake.INSTANCE.intakeOneZero,
                Intake.INSTANCE.intakeTwoZero,
                // Spin up + Aim
                Intake.INSTANCE.indexerIn,
                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)
                ),
                // Initialize systems
                new LambdaCommand().setStart(() -> {
                    LauncherOuttakeFuckingThing.INSTANCE.enableAutoCalculation();
                    Turret.INSTANCE.enableOdometryAim();
                }),

                new Delay(.5),
                Intake.INSTANCE.indexerIn,

                Intake.INSTANCE.intakeOnePowerFull,

                scoreFirst1,

                new Delay(.5),
                Intake.INSTANCE.indexerIn,

                Intake.INSTANCE.intakeTwoPowerFull,
                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)
                ),

                new Delay(2),

                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)
                ),

                grabfirst2,
                OpenGateFirst2,
                new Delay(.25),
                Intake.INSTANCE.intakeTwoZero,

                scorefirst3,
                Intake.INSTANCE.indexerIn,

                // AIM
                new LambdaCommand().setStart(() -> Turret.INSTANCE.enableOdometryAim()),

                new Delay(1),

                Intake.INSTANCE.intakeTwoPowerFull,
                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)
                ),
                new Delay(2),
                Intake.INSTANCE.intakeTwoZero,
                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)
                ),
                Intake.INSTANCE.intakeTwoPowerFull,

                grabsecond4,
                Intake.INSTANCE.intakeTwoZero,

                scoresecond5,
                Intake.INSTANCE.indexerIn,

                // AIM
                new LambdaCommand().setStart(() -> Turret.INSTANCE.enableOdometryAim()),

                new Delay(1),
                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)
                ),
                Intake.INSTANCE.intakeTwoPowerFull,
                new Delay(2),
                Intake.INSTANCE.intakeTwoZero,
                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)
                ),

                grabthird1_7,
                Intake.INSTANCE.intakeTwoPowerFull,
                grabthird2_8,
                Intake.INSTANCE.intakeTwoZero,
                Intake.INSTANCE.intakeOneZero,

                // MANUAL SAFETY or AIM
                new ParallelGroup(
                        new LambdaCommand()
                                .setStart(() -> {
                                    Turret.INSTANCE.enableOdometryAim();
                                }),
                        scorethird
                ),

                Intake.INSTANCE.indexerIn,

                // AIM
                new LambdaCommand().setStart(() -> Turret.INSTANCE.enableOdometryAim()),

                new Delay(1),
                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)
                ),
                Intake.INSTANCE.intakeOnePowerFull,
                Intake.INSTANCE.intakeTwoPowerFull,
                new Delay(2),
                move
        );

        auto.schedule();
    }

    @Override
    public void onUpdate() {
        BindingManager.update();
        Pose pedroPose = follower().getPose();
        double distInches = VisionDistanceHelper.distanceToGoalInches(pedroPose);

        telemetry.addData("Dist to Goal (Odo)", distInches);
        telemetry.addData("target RPM", LauncherOuttakeFuckingThing.INSTANCE.getTargetRpm());
        telemetry.addData("motor rpm", LauncherOuttakeFuckingThing.INSTANCE.getCurrentRpm());
        telemetry.addData("turret_angle_deg", Turret.INSTANCE.getMeasuredAngleDeg());
        telemetry.addData("X", pedroPose.getX());
        telemetry.addData("Y", pedroPose.getY());

        telemetry.update();
    }
}