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
import org.firstinspires.ftc.teamcode.subsystem.LimelightPointDrive;
import org.firstinspires.ftc.teamcode.subsystem.Turret;
import org.firstinspires.ftc.teamcode.subsystem.VisionDistanceHelper;
import org.firstinspires.ftc.teamcode.subsystem.waitForShots;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "redCornerSpam")
public class RedCornerSpam extends NextFTCOpMode {

    public RedCornerSpam() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Turret.INSTANCE),
                new SubsystemComponent(Intake.INSTANCE),
                new SubsystemComponent(LauncherOuttakeFuckingThing.INSTANCE),
                BindingsComponent.INSTANCE
        );
    }


    public static final Command movefirst = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierLine(new Pose(48.000, 7.000).mirror(), new Pose(59.000, 22.000).mirror()))
                                .setConstantHeadingInterpolation(Math.toRadians(0))
                                .build(),
                        true
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    public static final Command grabfirst_1 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierCurve(new Pose(59.000, 22.000).mirror(), new Pose(34.627, 13.151).mirror(), new Pose(11.000, 15.000).mirror()))
                                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                                .build(),
                                false
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    public static final Command grabfirst_2 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierCurve(new Pose(11.000, 15.000).mirror(), new Pose(35.126, 11.320).mirror(), new Pose(11.000, 8.000).mirror()))
                                .setConstantHeadingInterpolation(Math.toRadians(0))
                                .build(),
                        1, false
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    public static final Command scoreFirst = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierLine(new Pose(9.000, 10.000).mirror(), new Pose(59.000, 22.000).mirror()))
                                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(15))
                                .build(),
                        false
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    public static final Command grabsecond = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierLine(new Pose(59.000, 22.000).mirror(), new Pose(10.000, 15.000).mirror()))
                                .setConstantHeadingInterpolation(Math.toRadians(180))
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    public static final Command grabsecondwiggle = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierCurve(new Pose(10.000, 15.000).mirror(), new Pose(25.138, 19.477).mirror(), new Pose(9.822, 15.803).mirror()))
                                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(345))
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    public static final Command scoresecond = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(), new Pose(59.000, 22.000).mirror()))
                                .setConstantHeadingInterpolation(Math.toRadians(10))
                                .build(),
                                false
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    public static final Command grabthird = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierLine(new Pose(59.000, 22.000).mirror(), new Pose(10.000, 30.000).mirror()))
                                .setConstantHeadingInterpolation(Math.toRadians(0))
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    public static final Command grabthirdwiggle = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierCurve(new Pose(10.000, 30.000).mirror(), new Pose(25.138, 19.477).mirror(), new Pose(9.822, 15.803).mirror()))
                                .setConstantHeadingInterpolation(Math.toRadians(0))
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    public static final Command scorethird = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(), new Pose(59.000, 22.000).mirror()))
                                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(15))
                                .build(),
                        false
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    public static final Command scorefourth = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(), new Pose(59.000, 22.000).mirror()))
                                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(10))
                                .build(),
                        false
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    public static final Command scorefifth = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(), new Pose(59.000, 22.000).mirror()))
                                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(345))
                                .build(),
                        false
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    public static final Command move = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(new BezierLine(new Pose(59, 22).mirror(), new Pose(30.000, 22.000).mirror()))
                                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(30))
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());


    @Override
    public void onInit() {
        // IMPORTANT: Zero the turret encoder logic based on current position
        Turret.INSTANCE.resetEncoderLogic();

        VisionDistanceHelper.GOAL_TAG_X_IN=  127.64;
        VisionDistanceHelper.GOAL_TAG_Y_IN  = 130.37;
        VisionDistanceHelper.GOAL_TARGET_X_FAR =  136;
        VisionDistanceHelper.GOAL_TARGET_Y_FAR = 144;

        VisionDistanceHelper.GOAL_TARGET_X =  132;
        VisionDistanceHelper.GOAL_TARGET_Y = 131;



        // Auto-aim immediately uses Odometry now
        Turret.INSTANCE.enableOdometryAim();
    }

    @Override
    public void onStartButtonPressed() {
        LauncherOuttakeFuckingThing.INSTANCE.enableAutoCalculation();
        LauncherOuttakeFuckingThing.autoCalculate = true;

        Follower follower = PedroComponent.follower();
        follower.setPose(new Pose(48.0, 7.0, Math.toRadians(180)).mirror());

        Command auto = new SequentialGroup(
                Intake.INSTANCE.indexerOut,

                Intake.INSTANCE.indexerIn,

                // Enable Systems
                new LambdaCommand().setStart(() -> {
                    LauncherOuttakeFuckingThing.INSTANCE.enableAutoCalculation();
                    Turret.INSTANCE.enableOdometryAim();
                }),

                Intake.INSTANCE.intakeOneZero,
                Intake.INSTANCE.intakeTwoZero,
                movefirst,
                Intake.INSTANCE.indexerIn,

                Intake.INSTANCE.intakeOnePowerFull,
                Intake.INSTANCE.intakeTwoPowerFull,

                new waitForShots(3,.4),
                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)
                ),
                grabfirst_1,
                grabfirst_2,

                new Delay(.2),
                Intake.INSTANCE.intakeTwoZero,
                scoreFirst,
                Intake.INSTANCE.indexerIn,


                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)),


                // AIM
                new LambdaCommand().setStart(() -> Turret.INSTANCE.enableOdometryAim()),
                Intake.INSTANCE.intakeTwoPowerFull,
                new waitForShots(3,.4),

                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)
                ),
                new LimelightPointDrive(true),
                Intake.INSTANCE.intakeTwoZero,


              /*  grabsecond
                  grabsecondwiggle,
                new Delay(.75),
                Intake.INSTANCE.intakeTwoZero,

                // Manual move for wall clearance if needed, then re-enable auto aim
                new LambdaCommand()
                        .setStart(() -> {
                            Turret.INSTANCE.setManualAngle(-60);
                        }),
                */
                scoresecond,
                Intake.INSTANCE.indexerIn,

                new LambdaCommand().setStart(() -> Turret.INSTANCE.enableOdometryAim()),
                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)),
                new Delay(.1),

                Intake.INSTANCE.intakeTwoPowerFull,



                new waitForShots(2,.4),

                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)
                ),
                new LimelightPointDrive(true),
                Intake.INSTANCE.intakeTwoZero,

                scorethird,
                Intake.INSTANCE.indexerIn,


                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)),
                new Delay(.1),

                Intake.INSTANCE.intakeTwoPowerFull,

                new waitForShots(2,.4),
                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)
                ),
                new LimelightPointDrive(true),
                Intake.INSTANCE.intakeTwoZero,

                scorefourth,
                Intake.INSTANCE.indexerIn,


                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)),
                new Delay(.1),

                Intake.INSTANCE.intakeTwoPowerFull,


                new waitForShots(2,.4),
                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)
                ),
                new LimelightPointDrive(true),
                Intake.INSTANCE.intakeTwoZero,

                scorefifth,
                Intake.INSTANCE.indexerIn,



                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)),
                 new Delay(.2),
                Intake.INSTANCE.intakeTwoPowerFull








                /*grabthird,
                grabthirdwiggle,
                Intake.INSTANCE.intakeTwoZero,
                scorethird,

                // AIM
                new LambdaCommand().setStart(() -> Turret.INSTANCE.enableOdometryAim()),

                new Delay(.5),
                Intake.INSTANCE.intakeTwoPowerFull,

                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)),
                new Delay(2),
                move */
        );

        auto.schedule();
    }

    @Override
    public void onUpdate () {
        BindingManager.update();
        Pose pedroPose = follower().getPose();

        // Calculate distance via Odometry now
        double distInches = VisionDistanceHelper.distanceToGoalInches(pedroPose);

        TelemetryPacket packet = new TelemetryPacket();
        Canvas field = packet.fieldOverlay();
        double x = pedroPose.getX();
        double y = pedroPose.getY();
        double h = pedroPose.getHeading();
        field.strokeCircle(x, y, 9.0);
        field.strokeLine(x, y, x + 10 * Math.cos(h), y + 10 * Math.sin(h));

        telemetry.addData("Dist to Goal (Odo)", distInches);
        telemetry.addData("target RPM", LauncherOuttakeFuckingThing.INSTANCE.getTargetRpm());
        telemetry.addData("motor rpm", LauncherOuttakeFuckingThing.INSTANCE.getCurrentRpm());
        telemetry.addData("turret_angle_deg", Turret.INSTANCE.getMeasuredAngleDeg());
        telemetry.addData("X", pedroPose.getX());
        telemetry.addData("Y", pedroPose.getY());
        telemetry.update();
    }
}