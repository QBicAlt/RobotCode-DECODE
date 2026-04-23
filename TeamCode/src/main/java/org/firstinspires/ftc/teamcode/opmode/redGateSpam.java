package org.firstinspires.ftc.teamcode.opmode;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pathfinding.GrabGateStorage;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.LauncherOuttakeFuckingThing;
import org.firstinspires.ftc.teamcode.subsystem.Turret;
import org.firstinspires.ftc.teamcode.subsystem.VisionDistanceHelper;
import org.firstinspires.ftc.teamcode.subsystem.waitForShots;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

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

@Autonomous(name = "redGateSpam")
public class redGateSpam extends NextFTCOpMode {

    public redGateSpam() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Turret.INSTANCE),
                new SubsystemComponent(Intake.INSTANCE),
                new SubsystemComponent(LauncherOuttakeFuckingThing.INSTANCE),
                BindingsComponent.INSTANCE
        );
    }

    // --- HELPER FOR MIRRORING HEADINGS (RADIANS) ---
    // Reflects angle across the Y-axis (180 - angle)
    private static double mirrorH(double radian) {
        return Math.PI - radian;
    }

    // --- COMMANDS ---

    public static final Command scoreFirst1 = new LambdaCommand() // 1
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                GrabGateStorage.startingPose.mirror(),
                                                new Pose(48.000, 96.000).mirror()
                                        )
                                )
                                .setLinearHeadingInterpolation(mirrorH(Math.toRadians(90)), mirrorH(Math.toRadians(180)))
                                .build()
                );
            })
            .setIsDone(() -> {
                Follower f = PedroComponent.follower();
                return  !f.isBusy();
            });

    public static final Command grabmid2 = new LambdaCommand() // 2
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierCurve(
                                                new Pose(48.000, 96.000).mirror(),
                                                new Pose(45.947, 55.602).mirror(),
                                                new Pose(40.786, 60.264).mirror(),
                                                new Pose(21.642, 62.095).mirror(),
                                                new Pose(43.783, 57.933).mirror(),
                                                new Pose(22.973, 57.434).mirror(),
                                                new Pose(22.000, 58.432).mirror()
                                        )
                                )
                                .setConstantHeadingInterpolation(mirrorH(Math.toRadians(180)))
                                .build(),
                        1, false
                );
            })
            .setIsDone(() -> {
                Follower f = PedroComponent.follower();
                return f.getCurrentTValue() > 0.95 || !f.isBusy();
            });


    public static final Command scoremid3 = new LambdaCommand() // 3
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                new Pose(22.000, 58.432).mirror(),
                                                GrabGateStorage.scorePose.mirror()
                                        )
                                )
                                .setTangentHeadingInterpolation()
                                .setReversed()
                                .build(),
                        false
                );
            })
            .setIsDone(() -> {
                Follower f = PedroComponent.follower();
                return  !f.isBusy();
            });

    public static final Command grabGate1_4 = new LambdaCommand() //4
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierCurve(GrabGateStorage.scorePose.mirror(),
                                                GrabGateStorage.curveControlPose.mirror(),
                                                GrabGateStorage.approachPose.mirror())
                                )
                                .setConstantHeadingInterpolation(mirrorH(Math.toRadians(180)))
                                .build(),
                        1, false
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    public static final Command grabGate2_5 = new LambdaCommand() // 5
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                GrabGateStorage.approachPose.mirror(),
                                                GrabGateStorage.preGrabPose.mirror()
                                        )
                                )
                                .setConstantHeadingInterpolation(mirrorH(Math.toRadians(130)))
                                .build(),
                        false
                );
            })
            .setIsDone(() -> {
                Follower f = PedroComponent.follower();
                return f.getCurrentTValue() > 0.95 || !f.isBusy();
            });

    public static final Command grabGate3 = new LambdaCommand() // 5
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                GrabGateStorage.preGrabPose.mirror(),
                                                GrabGateStorage.grabPose.mirror()
                                        )
                                )
                                .setConstantHeadingInterpolation(mirrorH(Math.toRadians(140)))
                                .build(),
                        false
                );
            })
            .setIsDone(() -> {
                Follower f = PedroComponent.follower();
                return f.getCurrentTValue() > 0.95 || !f.isBusy();
            });

    public static final Command scoreGate1_6 = new LambdaCommand() //6
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierCurve(
                                                GrabGateStorage.grabPose.mirror(),
                                                GrabGateStorage.curveScorePose.mirror(),
                                                GrabGateStorage.scorePose.mirror()
                                        ))
                                .setTangentHeadingInterpolation()
                                .setReversed()
                                .build(),
                        false
                );
            })
            .setIsDone(() -> {
                Follower f = PedroComponent.follower();
                return  !f.isBusy();
            });


    public static final Command grabgate3_7 = new LambdaCommand() //7
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierCurve(GrabGateStorage.scorePose.mirror(),
                                                GrabGateStorage.curveControlPose.mirror(),
                                                GrabGateStorage.approachPose.mirror())
                                )
                                .setConstantHeadingInterpolation(mirrorH(Math.toRadians(180)))
                                .build(),
                        false

                );

            })
            .setIsDone(() -> {
                Follower f = PedroComponent.follower();
                return f.getCurrentTValue() > 0.95 || !f.isBusy();
            });


    public static final Command grabGate4_8 = new LambdaCommand() //8
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                GrabGateStorage.approachPose.mirror(),
                                                GrabGateStorage.preGrabPose.mirror()
                                        )
                                )
                                .setConstantHeadingInterpolation(mirrorH(Math.toRadians(130)))
                                .build()
                );

            })
            .setIsDone(() -> {
                Follower f = PedroComponent.follower();
                return f.getCurrentTValue() > 0.95 || !f.isBusy();
            });

    public static final Command grabGate6 = new LambdaCommand() //8
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                GrabGateStorage.preGrabPose.mirror(),
                                                GrabGateStorage.grabPose.mirror()
                                        )
                                )
                                .setConstantHeadingInterpolation(mirrorH(Math.toRadians(140)))
                                .build(),
                        false
                );

            })
            .setIsDone(() -> {
                Follower f = PedroComponent.follower();
                return f.getCurrentTValue() > 0.95 || !f.isBusy();
            });

    public static final Command scoreGate_9 = new LambdaCommand() //9
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierCurve(
                                                GrabGateStorage.grabPose.mirror(),
                                                GrabGateStorage.curveScorePose.mirror(),
                                                GrabGateStorage.scorePose.mirror()
                                        ))
                                .setTangentHeadingInterpolation()
                                .setReversed()
                                .build(),
                        false
                );

            })
            .setIsDone(() -> {
                Follower f = PedroComponent.follower();
                return  !f.isBusy();
            });

    public static final Command grabFar_10 = new LambdaCommand() //10
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierCurve(
                                                GrabGateStorage.scorePose.mirror(),
                                                new Pose(64.067, 31.719).mirror(),
                                                new Pose(60.145, 37.929).mirror(),
                                                new Pose(20.000, 37.500).mirror()
                                        )
                                )
                                .setTangentHeadingInterpolation()
                                .build(), false
                );

            })
            .setIsDone(() -> {
                Follower f = PedroComponent.follower();
                return f.getCurrentTValue() > 0.95 || !f.isBusy();
            });

    public static final Command scoreFar_11 = new LambdaCommand() //11
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                new Pose(20, 37.5).mirror(),
                                                new Pose(57.000, 82.000).mirror()
                                        )
                                )
                                .setTangentHeadingInterpolation()
                                .setReversed()
                                .build(), false
                );

            })
            .setIsDone(() -> {
                Follower f = PedroComponent.follower();
                return  !f.isBusy();
            });

    public static final Command grabClose_12 = new LambdaCommand() //12
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierCurve(
                                                new Pose(57.000, 82.000).mirror(),
                                                new Pose(49.443, 85.401).mirror(),
                                                new Pose(25.000, 84.000).mirror()
                                        )
                                )
                                .setConstantHeadingInterpolation(mirrorH(Math.toRadians(180)))
                                .build(), false
                );

            })
            .setIsDone(() -> {
                Follower f = PedroComponent.follower();
                return f.getCurrentTValue() > 0.95 || !f.isBusy();
            });

    public static final Command scoreClose_13 = new LambdaCommand() //13
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                new Pose(25.000, 84.000).mirror(),
                                                new Pose(52.000, 86.000).mirror()
                                        )
                                )
                                .setTangentHeadingInterpolation()
                                .setReversed()
                                .build(),
                        false
                );

            })
            .setIsDone(() -> {
                Follower f = PedroComponent.follower();
                return !f.isBusy();
            });

    public static final Command park_14 = new LambdaCommand() //14
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                new Pose(52.000, 86.000).mirror(),
                                                new Pose(40.000, 86.000).mirror()
                                        )
                                )
                                .setTangentHeadingInterpolation()
                                .build(),
                        false
                );

            })
            .setIsDone(() -> {
                Follower f = PedroComponent.follower();
                return f.getCurrentTValue() > 0.98 || !f.isBusy();
            });


    @Override
    public void onInit() {
        // Red Goal is mirrored across X
        VisionDistanceHelper.GOAL_TAG_X_IN = 144.0 - (144.0 - 127.64); // Logic checks out, but simplify: Just use the actual X coordinate for Red side tags
        // Alternatively, if VisionDistanceHelper handles raw coords:
        VisionDistanceHelper.GOAL_TAG_X_IN = 127.64; // Red side tag X is typically opposite
        VisionDistanceHelper.GOAL_TAG_Y_IN = 130.37;

        VisionDistanceHelper.GOAL_TARGET_X = 144; // Mirrored from 0?
        VisionDistanceHelper.GOAL_TARGET_Y = 144;
    }

    @Override
    public void onStartButtonPressed() {
        LauncherOuttakeFuckingThing.autoCalculate = true;
        LauncherOuttakeFuckingThing.INSTANCE.disableCompensation();

        Follower follower = PedroComponent.follower();
        follower.setPose(GrabGateStorage.startingPose.mirror());

        // Time to wait after path starts before killing the front intake
        double frontIntakeCutoff = .5;

        Command auto = new SequentialGroup(
                new LambdaCommand().setStart(() -> {
                    Turret.INSTANCE.enableOdometryAim();
                }),
                Intake.INSTANCE.indexerOut,
                Intake.INSTANCE.indexerIn,
                Intake.INSTANCE.intakeOneZero,
                Intake.INSTANCE.intakeTwoZero,
                Intake.INSTANCE.indexerIn,

                // --- PRELOAD SCORE ---
                new ParallelGroup(
                        Intake.INSTANCE.indexerIn,
                        scoreFirst1
                ),
                new Delay(.2),

                Intake.INSTANCE.intakeTwoPowerFull,
                Intake.INSTANCE.intakeOnePowerFull,

                new waitForShots(2, .3),

                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)
                ),

                // --- CYCLE 1 ---
                grabmid2,
                Intake.INSTANCE.intakeOneZero,

                new ParallelGroup(
                        new SequentialGroup(
                                new Delay(.4),
                                Intake.INSTANCE.intakeTwoZero// Kept as is
                        ),
                        Intake.INSTANCE.indexerIn,
                        new SequentialGroup(
                                scoremid3,
                                new LambdaCommand().setStart(() ->
                                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)
                                )
                        )

                ),

                new Delay(.1),
                Intake.INSTANCE.intakeTwoPowerFull,
                Intake.INSTANCE.intakeOnePowerFull,

                new waitForShots(2, .3),

                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)
                ),

                // --- CYCLE 2 ---
                grabGate1_4,
                grabGate2_5,
                grabGate3,
                new Delay(.85),
                Intake.INSTANCE.intakeOneZero,


                new ParallelGroup(
                        new SequentialGroup(
                                new Delay(.4),
                                Intake.INSTANCE.intakeTwoZero// Kept as is
                        ),
                        Intake.INSTANCE.indexerIn,
                        new SequentialGroup(
                                scoreGate1_6,
                                new LambdaCommand().setStart(() ->
                                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)
                                )
                        )

                ),

                Intake.INSTANCE.intakeTwoPowerFull,
                Intake.INSTANCE.intakeOnePowerFull,

                new waitForShots(2, .3),

                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)
                ),

                // --- CYCLE 3 ---
                grabgate3_7,
                grabGate4_8,
                grabGate6,
                new Delay(.85),
                Intake.INSTANCE.intakeOneZero,


                new ParallelGroup(
                        new SequentialGroup(
                                new Delay(.4),
                                Intake.INSTANCE.intakeTwoZero// Kept as is
                        ),
                        Intake.INSTANCE.indexerIn,
                        new SequentialGroup(
                                scoreGate_9,
                                new LambdaCommand().setStart(() ->
                                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open))
                        )
                ),

                Intake.INSTANCE.intakeTwoPowerFull,
                Intake.INSTANCE.intakeOnePowerFull,
                new waitForShots(3, .3),
                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)
                ),

                // --- FAR CYCLE ---
                grabFar_10,


                new ParallelGroup(
                        new SequentialGroup(
                                new Delay(.4),
                                Intake.INSTANCE.intakeTwoZero// Kept as is
                        ),
                        Intake.INSTANCE.indexerIn,
                        new SequentialGroup(
                                scoreFar_11,
                                new LambdaCommand().setStart(() ->
                                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)
                                )
                        ),
                        new SequentialGroup(
                                new Delay(.2),
                                Intake.INSTANCE.intakeOneZero
                        )
                ),

                Intake.INSTANCE.intakeTwoPowerFull,
                Intake.INSTANCE.intakeOnePowerFull,


                new waitForShots(3, .3),
                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)
                ),


                grabClose_12,
                new ParallelGroup(
                        new SequentialGroup(
                                new Delay(.4),
                                Intake.INSTANCE.intakeTwoZero// Kept as is
                        ),
                        Intake.INSTANCE.indexerIn,
                        new SequentialGroup(
                                scoreClose_13,
                                new LambdaCommand().setStart(() ->
                                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)
                                )
                        ),
                        new SequentialGroup(
                                new Delay(.2),
                                Intake.INSTANCE.intakeOneZero
                        )
                ),
                Intake.INSTANCE.intakeTwoPowerFull,
                Intake.INSTANCE.intakeOnePowerFull,


                new waitForShots(3, .3),

                Intake.INSTANCE.intakeOneZero,
                park_14
        );

        auto.schedule();
    }

    @Override
    public void onUpdate() {
        BindingManager.update();
        Pose pedroPose = follower().getPose();
        double distInches = VisionDistanceHelper.distanceToGoalInches(pedroPose);
        PoseStorage.turretAngle = Turret.INSTANCE.getMeasuredAngleDeg();


        telemetry.addData("Dist to Goal (Odo)", distInches);
        telemetry.addData("target RPM", LauncherOuttakeFuckingThing.INSTANCE.getTargetRpm());
        telemetry.addData("motor rpm", LauncherOuttakeFuckingThing.INSTANCE.getCurrentRpm());
        telemetry.addData("turret_angle_deg", Turret.INSTANCE.getMeasuredAngleDeg());
        telemetry.addData("X", pedroPose.getX());
        telemetry.addData("Y", pedroPose.getY());
        telemetry.update();
    }

    @Override
    public void onStop() {
        Pose finalPose = PedroComponent.follower().getPose();
        PoseStorage.currentPose = finalPose;
        PoseStorage.hasAutoRun = true;
    }
}