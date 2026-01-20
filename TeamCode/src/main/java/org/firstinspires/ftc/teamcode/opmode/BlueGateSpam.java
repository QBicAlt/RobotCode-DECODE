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
import org.firstinspires.ftc.teamcode.subsystem.waitForShots;

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

@Autonomous(name = "blueGateSpam")
public class BlueGateSpam extends NextFTCOpMode {

    public BlueGateSpam() {
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
                                .addPath(new BezierLine(new Pose(19.00, 118.000), new Pose(48.000, 96.000))
                                )
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
                                .addPath(
                                        new BezierCurve(
                                                new Pose(48.000, 96.000),
                                                new Pose(69.753, 81.739),
                                                new Pose(46.613, 79.575),
                                                new Pose(18.479, 82.072)
                                        )
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(180))
                                .build(),
                        .8, false
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());



    public static final Command scorefirst3 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(new Pose(18.479, 82.072), new Pose(55.000, 84.000))
                                )
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
                                .addPath(
                                        new BezierCurve(
                                                new Pose(55.000, 84.000),
                                                new Pose(59.600, 52.772),
                                                new Pose(40.786, 60.264),
                                                new Pose(21.642, 62.095),
                                                new Pose(43.783, 57.933),
                                                new Pose(23.306, 54.936),
                                                new Pose(18.000, 58.432)
                                        )
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(180))
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
                                .addPath(
                                        new BezierLine(new Pose(18.000, 58.432), new Pose(55.000, 84.000))
                                )
                                .setTangentHeadingInterpolation()
                                .setReversed()
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    public static final Command grabGate = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierCurve(
                                                new Pose(55.000, 84.000),
                                                new Pose(33.628, 38.455),
                                                new Pose(13.30, 58.5)
                                        )
                                )
                                .setLinearHeadingInterpolation(Math.toRadians(-148), Math.toRadians(140))
                                .build(),
                                false
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());



    public static final Command scoreGate = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierCurve(
                                                new Pose(13.3, 58.500),
                                                new Pose(37.124, 62.095),
                                                new Pose(55.000, 84.000)
                                        )
                        )
                        .setTangentHeadingInterpolation()
                        .setReversed()
                                .build()
                        , false);

            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());


    @Override
    public void onInit() {
        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed);
        Turret.INSTANCE.resetEncoderLogic();


        // Blue Goal
        VisionDistanceHelper.GOAL_TAG_X_IN=  144.0 - 127.64;
        VisionDistanceHelper.GOAL_TAG_Y_IN  = 130.37;
        VisionDistanceHelper.GOAL_TARGET_X_FAR =  144.0 - 136.0 ;
        VisionDistanceHelper.GOAL_TARGET_Y_FAR = 144;

        VisionDistanceHelper.GOAL_TARGET_X =  7 ;
        VisionDistanceHelper.GOAL_TARGET_Y = 131.0;

        Turret.INSTANCE.enableOdometryAim();

    }

    @Override
    public void onStartButtonPressed() {
        LauncherOuttakeFuckingThing.INSTANCE.enableAutoCalculation();
        LauncherOuttakeFuckingThing.autoCalculate = true;

        Follower follower = PedroComponent.follower();
        follower.setPose(new Pose(19, 118, Math.toRadians(144)));

        Command auto = new SequentialGroup(
                Intake.INSTANCE.indexerOut,
                Intake.INSTANCE.indexerIn,

                Intake.INSTANCE.intakeOneZero,
                Intake.INSTANCE.intakeTwoZero,
                // Spin up + Aim
                new LambdaCommand().setStart(() -> {
                    LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed);
                    LauncherOuttakeFuckingThing.INSTANCE.enableAutoCalculation();
                    Turret.INSTANCE.enableOdometryAim();
                }),

                Intake.INSTANCE.indexerIn,

                Intake.INSTANCE.intakeOnePowerFull,

                scoreFirst1,


                Intake.INSTANCE.indexerIn,

                new LambdaCommand().setStart(() -> Turret.INSTANCE.enableOdometryAim()),
                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)),
                new Delay(.2),

                Intake.INSTANCE.intakeTwoPowerFull,
                Intake.INSTANCE.intakeOnePowerFull,




                new waitForShots(2,.4),

                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)
                ),


                grabfirst2,
                Intake.INSTANCE.intakeTwoZero,
                Intake.INSTANCE.intakeOnePowerFull,


                scorefirst3,
                Intake.INSTANCE.intakeTwoZero,

                Intake.INSTANCE.indexerIn,

                new LambdaCommand().setStart(() -> Turret.INSTANCE.enableOdometryAim()),
                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)),
                new Delay(.2),

                Intake.INSTANCE.intakeTwoPowerFull,
                Intake.INSTANCE.intakeOnePowerFull,


                new waitForShots(2,.4),

                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)
                ),

                grabsecond4,
                Intake.INSTANCE.intakeTwoZero,

                scoresecond5,

                Intake.INSTANCE.indexerIn,
                Intake.INSTANCE.intakeTwoZero,



                new LambdaCommand().setStart(() -> Turret.INSTANCE.enableOdometryAim()),
                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)),
                new Delay(.2),

                Intake.INSTANCE.intakeTwoPowerFull,
                Intake.INSTANCE.intakeOnePowerFull,




                new waitForShots(2,.4),

                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)
                ),

                grabGate,
                new Delay(3),
                Intake.INSTANCE.intakeTwoPowerFull,
                Intake.INSTANCE.intakeOnePowerFull,


                scoreGate,

                Intake.INSTANCE.indexerIn,
                Intake.INSTANCE.intakeTwoZero,


                new LambdaCommand().setStart(() -> Turret.INSTANCE.enableOdometryAim()),
                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)),
                new Delay(.2),

                Intake.INSTANCE.intakeTwoPowerFull,
                Intake.INSTANCE.intakeOnePowerFull,




                new waitForShots(3,.4),

                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)
                ),
                Intake.INSTANCE.intakeTwoZero,
                Intake.INSTANCE.intakeOneZero

                // Manual Turret Logic to avoid collision if necessary, then aim

                // Final Ai
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