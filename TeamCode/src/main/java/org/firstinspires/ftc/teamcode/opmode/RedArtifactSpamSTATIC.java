package org.firstinspires.ftc.teamcode.opmode;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

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
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "redArtifactSpamSTATIC")
public class RedArtifactSpamSTATIC extends NextFTCOpMode {

    public RedArtifactSpamSTATIC() {
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
                                .addPath(new BezierCurve(new Pose(48.000, 96.000).mirror(), new Pose(69.753, 81.739).mirror(), new Pose(15.000, 84.000).mirror()))
                                .setConstantHeadingInterpolation(Math.toRadians(0))
                                .build(),
                        .6, false
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    public static final Command scorefirst3 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierLine(new Pose(15.000, 84.000).mirror(), new Pose(55.000, 84.000).mirror()))
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
                                .addPath(new BezierCurve(new Pose(55.000, 84.000).mirror(), new Pose(59.598, 52.772).mirror(), new Pose(40.786, 60.264).mirror(), new Pose(21.309, 60.097).mirror(), new Pose(43.616, 58.765).mirror(), new Pose(14.150, 59.931).mirror()))
                                .setConstantHeadingInterpolation(Math.toRadians(0))
                                .build(),
                        .6, false
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    public static final Command grabsecondgate5 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierCurve(new Pose(19.150, 56.931).mirror(), new Pose(26.969, 54.437).mirror(), new Pose(31.464, 62.594).mirror(), new Pose(15.000, 69.420).mirror()))
                                .setConstantHeadingInterpolation(Math.toRadians(0))
                                .build(),
                        .6, false
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    public static final Command scoresecond5 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierLine(new Pose(15.000, 69.420).mirror(), new Pose(55.000, 84.000).mirror()))
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
                        .6, false);
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    public static final Command scorethird = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierLine(new Pose(11.487, 35.292).mirror(), new Pose(55.000, 84.000).mirror()))
                                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36))
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());


    @Override
    public void onInit() {
        // Zero Encoders
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
                Intake.INSTANCE.indexerIn,

                // Init systems
                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)
                ),
                new LambdaCommand().setStart(() -> {
                    LauncherOuttakeFuckingThing.INSTANCE.enableAutoCalculation();
                    Turret.INSTANCE.enableOdometryAim();
                }),

                Intake.INSTANCE.intakeOnePowerFull,

                scoreFirst1,

                Intake.INSTANCE.intakeTwoPowerFull,
                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)
                ),

                new Delay(1.5),

                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)
                ),

                grabfirst2,
                Intake.INSTANCE.intakeTwoZero,

                scorefirst3,

                // AIM
                new LambdaCommand().setStart(() -> Turret.INSTANCE.enableOdometryAim()),

                new Delay(.75),

                Intake.INSTANCE.intakeTwoPowerFull,
                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)
                ),
                new Delay(1.5),
                Intake.INSTANCE.intakeTwoZero,
                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)
                ),
                Intake.INSTANCE.intakeTwoPowerFull,

                grabsecond4,
                grabsecondgate5,
                new Delay(1.5),
                Intake.INSTANCE.intakeTwoZero,

                scoresecond5,

                // AIM
                new LambdaCommand().setStart(() -> Turret.INSTANCE.enableOdometryAim()),

                new Delay(.75),
                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)
                ),
                Intake.INSTANCE.intakeTwoPowerFull,
                new Delay(1.5),
                Intake.INSTANCE.intakeTwoZero,
                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)
                ),

                grabthird1_7,
                Intake.INSTANCE.intakeTwoPowerFull,
                grabthird2_8,
                new Delay(.75),
                Intake.INSTANCE.intakeTwoZero,
                Intake.INSTANCE.intakeOneZero,

                scorethird,

                // AIM
                new LambdaCommand().setStart(() -> Turret.INSTANCE.enableOdometryAim()),

                new Delay(1),
                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)
                ),
                Intake.INSTANCE.intakeOnePowerFull,
                Intake.INSTANCE.intakeTwoPowerFull
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