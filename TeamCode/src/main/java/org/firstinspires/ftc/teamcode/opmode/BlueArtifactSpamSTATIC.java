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

@Autonomous(name = "blueArtifactSpamSTATIC")
public class BlueArtifactSpamSTATIC extends NextFTCOpMode {

    public BlueArtifactSpamSTATIC() {
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
                                .addPath(new BezierLine(new Pose(17.8, 118), new Pose(48.000, 96.000)))
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
                                .addPath(new BezierCurve(new Pose(48.000, 96.000), new Pose(69.753, 81.739), new Pose(19.000, 84.000)))
                                .setConstantHeadingInterpolation(Math.toRadians(180))
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
                                .addPath(new BezierLine(new Pose(19.000, 84.000), new Pose(55.000, 84.000)))
                                .setConstantHeadingInterpolation(Math.toRadians(180))
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    public static final Command grabsecond4 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierCurve(new Pose(55.000, 84.000), new Pose(59.598, 52.772), new Pose(40.786, 60.264), new Pose(21.309, 60.097), new Pose(43.616, 58.765), new Pose(14.150, 59.931)))
                                .setConstantHeadingInterpolation(Math.toRadians(180))
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
                                .addPath(new BezierCurve(new Pose(19.150, 56.931), new Pose(26.969, 54.437), new Pose(31.464, 62.594), new Pose(15.000, 69.420)))
                                .setConstantHeadingInterpolation(Math.toRadians(180))
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
                                .addPath(new BezierLine(new Pose(15.000, 69.420), new Pose(55.000, 84.000)))
                                .setConstantHeadingInterpolation(Math.toRadians(180))
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    public static final Command grabthird1_7 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierCurve(new Pose(55.000, 84.000), new Pose(63.760, 36.791), new Pose(44.615, 35.292)))
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
                                .addPath(new BezierLine(new Pose(44.615, 35.292), new Pose(11.487, 35.292)))
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
                                .addPath(new BezierLine(new Pose(11.487, 35.292), new Pose(55.000, 84.000)))
                                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());


    @Override
    public void onInit() {
        Turret.INSTANCE.resetEncoderLogic();
        Turret.INSTANCE.enableOdometryAim();
        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed);

        // Blue Goal Coords
        VisionDistanceHelper.GOAL_TARGET_X = 144 - 127.64;
        VisionDistanceHelper.GOAL_TARGET_Y = 130.37;
    }

    @Override
    public void onStartButtonPressed() {
        LauncherOuttakeFuckingThing.INSTANCE.enableAutoCalculation();
        LauncherOuttakeFuckingThing.autoCalculate = true;

        Follower follower = PedroComponent.follower();
        follower.setPose(new Pose(17.8, 118, Math.toRadians(144)));

        Command auto = new SequentialGroup(
                Intake.INSTANCE.intakeOneZero,
                Intake.INSTANCE.intakeTwoZero,
                Intake.INSTANCE.indexerIn,
                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)
                ),

                // Spin up + Aim
                new LambdaCommand().setStart(() -> {
                    LauncherOuttakeFuckingThing.INSTANCE.enableAutoCalculation();
                    Turret.INSTANCE.enableOdometryAim();
                }),

                Intake.INSTANCE.intakeOnePowerFull,
                Intake.INSTANCE.intakeTwoZero,

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

                // Aim
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

                // Aim
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

                // Aim
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