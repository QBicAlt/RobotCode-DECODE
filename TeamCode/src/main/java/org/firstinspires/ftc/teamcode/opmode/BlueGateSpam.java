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

    public static final Command scoreFirst1 = new LambdaCommand() // 1
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower .pathBuilder()
                                .addPath(
                                        new BezierLine(GrabGateStorage.startingPose, new Pose(48.000, 96.000))
                                )
                                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                                .build()
                );
            })
            // THIS IS THE KEY CHANGE
            .setIsDone(() -> {
                Follower f = PedroComponent.follower();
                // We are done if we pass 95% progress OR if the path finishes normally
                return !f.isBusy();
            });

    public static final Command grabmid2 = new LambdaCommand() // 2
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierCurve(
                                                new Pose(48.000, 96.000),
                                                new Pose(45.947, 55.602),
                                                new Pose(40.786, 60.264),
                                                new Pose(21.642, 62.095),
                                                new Pose(43.783, 57.933),
                                                new Pose(22.973, 57.434),
                                                new Pose(22.000, 58.432)
                                        )
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(180))
                                .build(),
                        1, false
                );
            })
            .setIsDone(() -> {
                Follower f = PedroComponent.follower();
                // We are done if we pass 95% progress OR if the path finishes normally
                return f.getCurrentTValue() > 0.95 || !f.isBusy();
            });


    public static final Command scoremid3 = new LambdaCommand() // 3
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(new Pose(20.000, 58.432), GrabGateStorage.scorePose)
                                )
                                .setTangentHeadingInterpolation()
                                .setReversed()
                                .build()


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
                                        new BezierCurve(GrabGateStorage.scorePose,
                                                GrabGateStorage.curveControlPose,
                                                GrabGateStorage.approachPose)
                        )
                        .setConstantHeadingInterpolation(Math.toRadians(180))
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
                                                GrabGateStorage.approachPose,
                                                GrabGateStorage.preGrabPose
                                        )
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(140))
                                .build(),
                             false
                );
            })
            .setIsDone(() -> {
                Follower f = PedroComponent.follower();
                // We are done if we pass 95% progress OR if the path finishes normally
                return f.getCurrentTValue() > 0.97 || !f.isBusy();
            });

    public static final Command grabGate3 = new LambdaCommand() // 5
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(GrabGateStorage.preGrabPose, GrabGateStorage.grabPose)
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(150))
                                .build(),
                        false
                );
            })
            .setIsDone(() -> {
                Follower f = PedroComponent.follower();
                // We are done if we pass 95% progress OR if the path finishes normally
                return f.getCurrentTValue() > 0.97 || !f.isBusy();
            });
    public static final Command scoreGate1_6 = new LambdaCommand() //6
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierCurve(
                                                GrabGateStorage.grabPose,
                                                GrabGateStorage.curveScorePose,
                                                GrabGateStorage.scorePose
                                        ))
                                .setTangentHeadingInterpolation()
                                .setReversed()
                                .build(),
                                false
                );
            })
            .setIsDone(() -> {
                Follower f = PedroComponent.follower();
                // We are done if we pass 95% progress OR if the path finishes normally
                return  !f.isBusy();
            });


    public static final Command grabgate3_7 = new LambdaCommand() //7
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierCurve(GrabGateStorage.scorePose,
                                                GrabGateStorage.curveControlPose,
                                                GrabGateStorage.approachPose)
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(180))
                                .build(),
                        false

                        );

            })
            .setIsDone(() -> {
                Follower f = PedroComponent.follower();
                // We are done if we pass 95% progress OR if the path finishes normally
                return f.getCurrentTValue() > 0.97 || !f.isBusy();
            });


    public static final Command grabGate4_8 = new LambdaCommand() //8
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                GrabGateStorage.approachPose,
                                                GrabGateStorage.preGrabPose
                                        )
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(140))
                                .build()
                );

            })
            .setIsDone(() -> {
                Follower f = PedroComponent.follower();
                // We are done if we pass 95% progress OR if the path finishes normally
                return f.getCurrentTValue() > 0.97 || !f.isBusy();
            });

    public static final Command grabGate6 = new LambdaCommand() //8
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(GrabGateStorage.preGrabPose, GrabGateStorage.grabPose)
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(150))
                                        .build(),
                                false
                );

            })
            .setIsDone(() -> {
                Follower f = PedroComponent.follower();
                // We are done if we pass 95% progress OR if the path finishes normally
                return f.getCurrentTValue() > 0.97 || !f.isBusy();
            });



    public static final Command scoreGate_9 = new LambdaCommand() //9
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierCurve(
                                                GrabGateStorage.grabPose,
                                                GrabGateStorage.curveScorePose,
                                                GrabGateStorage.scorePose
                                        ))
                                .setTangentHeadingInterpolation()
                                .setReversed()
                                .build(),
                        false
                );

            })
            .setIsDone(() -> {
                Follower f = PedroComponent.follower();
                // We are done if we pass 95% progress OR if the path finishes normally
                return  !f.isBusy();
            });
    public static final Command grabFar_10 = new LambdaCommand() //10
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()

                                .addPath(
                                        new BezierCurve(
                                                GrabGateStorage.scorePose,
                                                new Pose(64.067, 31.719),
                                                new Pose(60.145, 37.929),
                                                new Pose(20.000, 37.500)
                                        )
                                )
                                .setTangentHeadingInterpolation()
                                .build(),false
                );

            })
            .setIsDone(() -> {
                Follower f = PedroComponent.follower();
                // We are done if we pass 95% progress OR if the path finishes normally
                return f.getCurrentTValue() > 0.97 || !f.isBusy();
            });
    public static final Command scoreFar_11 = new LambdaCommand() //11
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(new Pose(20, 37.5), new Pose(57.000, 82.000))
                                )
                                .setTangentHeadingInterpolation()
                                .setReversed()
                                .build() ,false
                );

            })
            .setIsDone(() -> {
                Follower f = PedroComponent.follower();
                // We are done if we pass 95% progress OR if the path finishes normally
                return  !f.isBusy();
            });
    public static final Command grabClose_12 = new LambdaCommand() //12
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierCurve(
                                                new Pose(57.000, 82.000),
                                                new Pose(49.443, 85.401),
                                                new Pose(25.000, 84.000)
                                        )
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(180))
                                .build() , false
                );

            })
            .setIsDone(() -> {
                Follower f = PedroComponent.follower();
                // We are done if we pass 95% progress OR if the path finishes normally
                return f.getCurrentTValue() > 0.95 || !f.isBusy();
            });
    public static final Command scoreClose_13 = new LambdaCommand() //13
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(new Pose(25.000, 84.000), new Pose(52.000, 86.000))
                                )
                                .setTangentHeadingInterpolation()
                                .setReversed()
                                .build(),
                                false
                );

            })
            .setIsDone(() -> {
                Follower f = PedroComponent.follower();
                // We are done if we pass 95% progress OR if the path finishes normally
                return f.getCurrentTValue() > 0.99 || !f.isBusy();
            });

    public static final Command park_14 = new LambdaCommand() //13
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(new Pose(52.000, 86.000), new Pose(40.000, 86.000))
                                )
                                .setTangentHeadingInterpolation()
                                .build(),
                        false
                );

            })
            .setIsDone(() -> {
                Follower f = PedroComponent.follower();
                // We are done if we pass 95% progress OR if the path finishes normally
                return f.getCurrentTValue() > 0.98 || !f.isBusy();
            });








    @Override
    public void onInit() {

        // Blue Goal
        VisionDistanceHelper.GOAL_TAG_X_IN=  144.0 - 127.64;
        VisionDistanceHelper.GOAL_TAG_Y_IN  = 130.37;



        VisionDistanceHelper.GOAL_TARGET_X =  0;
        VisionDistanceHelper.GOAL_TARGET_Y = 140;

    }  @Override
    public void onStartButtonPressed() {
        LauncherOuttakeFuckingThing.autoCalculate = true;
        LauncherOuttakeFuckingThing.INSTANCE.disableCompensation();

        Follower follower = PedroComponent.follower();
        follower.setPose(GrabGateStorage.startingPose);

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
                new Delay(.4),

                Intake.INSTANCE.intakeTwoPowerFull,
                Intake.INSTANCE.intakeOnePowerFull,

                new waitForShots(1.5, .3),

                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)
                ),

                // --- CYCLE 1 ---
                grabmid2,

                new ParallelGroup(
                        new SequentialGroup(
                                new Delay(.5),
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

                new Delay(.1 ),
                Intake.INSTANCE.intakeTwoPowerFull,
                Intake.INSTANCE.intakeOnePowerFull,

                new waitForShots(1.5, .3),

                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)
                ),

                // --- CYCLE 2 ---
                grabGate1_4,
                grabGate2_5,
                grabGate3,
                new Delay(1.3),
                Intake.INSTANCE.intakeOneZero,


                new ParallelGroup(
                        new SequentialGroup(
                                new Delay(1),
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
                new Delay(1.3),
                Intake.INSTANCE.intakeOneZero,


        new ParallelGroup(
                        new SequentialGroup(
                                new Delay(1),
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
                new waitForShots(2, .3),
                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)
                ),

                // --- FAR CYCLE ---
                grabFar_10,


                new ParallelGroup(
                        new SequentialGroup(
                                new Delay(.5),
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
                                new Delay(.3),
                                Intake.INSTANCE.intakeOneZero
                        )
                ),

                Intake.INSTANCE.intakeTwoPowerFull,
                Intake.INSTANCE.intakeOnePowerFull,


                new waitForShots(2, .3),
                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)
                ),


                grabClose_12,
                new ParallelGroup(
                        new SequentialGroup(
                                new Delay(.5),
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
                                new Delay(.3),
                                Intake.INSTANCE.intakeOneZero
                        )
                ),
                Intake.INSTANCE.intakeTwoPowerFull,
                Intake.INSTANCE.intakeOnePowerFull,


                new waitForShots(2, .3),

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
    // Change "stop" to "onStop"
    // ... inside BlueGateSpam class ...

    // Change "stop" to "onStop"
    @Override
    public void onStop() {
// 1. Capture the exact position where Auto ended
        Pose finalPose = PedroComponent.follower().getPose();

        // 2. Save it to our static storage bridge
        PoseStorage.currentPose = finalPose;
        PoseStorage.hasAutoRun = true;



    }


}