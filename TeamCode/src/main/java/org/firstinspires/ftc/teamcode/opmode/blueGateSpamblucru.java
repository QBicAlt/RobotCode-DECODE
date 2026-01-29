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

@Autonomous(name = "blueGateSpamblucru ")
public class blueGateSpamblucru extends NextFTCOpMode {

    public blueGateSpamblucru() {
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
                return f.getCurrentTValue() > 0.99 || !f.isBusy();
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
                                        new BezierLine(new Pose(22.000, 58.432), GrabGateStorage.scorePose)
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
    public static final Command grabGate1_4 = new LambdaCommand() //4
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(GrabGateStorage.scorePose, GrabGateStorage.approachPose)
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
                                        new BezierCurve(
                                                GrabGateStorage.approachPose,
                                                GrabGateStorage.curveControlPose,
                                                GrabGateStorage.preGrabPose
                                        )
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(130))
                                .build(),
                        false
                );
            })
            .setIsDone(() -> {
                Follower f = PedroComponent.follower();
                // We are done if we pass 95% progress OR if the path finishes normally
                return f.getCurrentTValue() > 0.95 || !f.isBusy();
            });

    public static final Command grabGate3_6 = new LambdaCommand() // 5
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(GrabGateStorage.preGrabPose, GrabGateStorage.grabPose)
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(140))
                                .build(),
                        false
                );
            })
            .setIsDone(() -> {
                Follower f = PedroComponent.follower();
                // We are done if we pass 95% progress OR if the path finishes normally
                return f.getCurrentTValue() > 0.95 || !f.isBusy();
            });
    public static final Command scoreGate1_7 = new LambdaCommand() //6
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
                return f.getCurrentTValue() > 0.99 || !f.isBusy();
            });


    public static final Command grabgate3_8 = new LambdaCommand() //7
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(GrabGateStorage.scorePose, GrabGateStorage.approachPose)
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(180))
                                .build(),
                        false

                );

            })
            .setIsDone(() -> {
                Follower f = PedroComponent.follower();
                // We are done if we pass 95% progress OR if the path finishes normally
                return f.getCurrentTValue() > 0.95 || !f.isBusy();
            });


    public static final Command grabGate4_9 = new LambdaCommand() //8
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierCurve(
                                                GrabGateStorage.approachPose,
                                                GrabGateStorage.curveControlPose,
                                                GrabGateStorage.preGrabPose
                                        )
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(130))
                                .build()
                );

            })
            .setIsDone(() -> {
                Follower f = PedroComponent.follower();
                // We are done if we pass 95% progress OR if the path finishes normally
                return f.getCurrentTValue() > 0.95 || !f.isBusy();
            });

    public static final Command grabGate10 = new LambdaCommand() //8
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(new Pose(14.00, 56.5), GrabGateStorage.grabPose)
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(140))
                                .build(),
                        false
                );

            })
            .setIsDone(() -> {
                Follower f = PedroComponent.follower();
                // We are done if we pass 95% progress OR if the path finishes normally
                return f.getCurrentTValue() > 0.95 || !f.isBusy();
            });



    public static final Command scoreGate11 = new LambdaCommand() //9
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierCurve(
                                                GrabGateStorage.grabPose,
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
                return f.getCurrentTValue() > 0.99 || !f.isBusy();
            });
    public static final Command grabgate12 = new LambdaCommand() //7
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(GrabGateStorage.scorePose,GrabGateStorage.approachPose)
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(180))
                                .build(),
                        false

                );

            })
            .setIsDone(() -> {
                Follower f = PedroComponent.follower();
                // We are done if we pass 95% progress OR if the path finishes normally
                return f.getCurrentTValue() > 0.95 || !f.isBusy();
            });


    public static final Command grabGate13 = new LambdaCommand() //8
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierCurve(
                                                GrabGateStorage.approachPose,
                                                GrabGateStorage.curveControlPose,
                                                GrabGateStorage.preGrabPose
                                        )
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(130))
                                .build()
                );

            })
            .setIsDone(() -> {
                Follower f = PedroComponent.follower();
                // We are done if we pass 95% progress OR if the path finishes normally
                return f.getCurrentTValue() > 0.95 || !f.isBusy();
            });

    public static final Command grabGate14 = new LambdaCommand() //8
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(GrabGateStorage.preGrabPose, GrabGateStorage.grabPose)
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(140))
                                .build(),
                        false
                );

            })
            .setIsDone(() -> {
                Follower f = PedroComponent.follower();
                // We are done if we pass 95% progress OR if the path finishes normally
                return f.getCurrentTValue() > 0.95 || !f.isBusy();
            });



    public static final Command scoreGate_15 = new LambdaCommand() //9
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
                return f.getCurrentTValue() > 0.99 || !f.isBusy();
            });

    public static final Command grabClose_16 = new LambdaCommand() //12
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
    public static final Command scoreClose_17 = new LambdaCommand() //13
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
                return f.getCurrentTValue() > 0.98 || !f.isBusy();
            });

    public static final Command park_18 = new LambdaCommand() //13
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
        VisionDistanceHelper.GOAL_TARGET_Y = 144;

    }

    @Override
    public void onStartButtonPressed() {
        LauncherOuttakeFuckingThing.autoCalculate = true;
        LauncherOuttakeFuckingThing.INSTANCE.disableCompensation();


        Follower follower = PedroComponent.follower();
        follower.setPose(GrabGateStorage.startingPose);

        Command auto = new SequentialGroup(

                new LambdaCommand().setStart(() -> {
                    Turret.INSTANCE.enableOdometryAim();
                }),
                Intake.INSTANCE.indexerOut,
                Intake.INSTANCE.indexerIn,

                Intake.INSTANCE.intakeOneZero,
                Intake.INSTANCE.intakeTwoZero,


                Intake.INSTANCE.indexerIn,


                new ParallelGroup(
                        Intake.INSTANCE.indexerIn,
                        scoreFirst1
                ),
                new Delay(.4),

                Intake.INSTANCE.intakeTwoPowerFull,
                Intake.INSTANCE.intakeOnePowerFull,




                new waitForShots(2,.3),

                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)
                ),


                grabmid2,
                Intake.INSTANCE.intakeTwoZero,


                new ParallelGroup(
                        new SequentialGroup(
                                new Delay(1),

                                new LambdaCommand().setStart(() ->
                                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)
                                )),
                        Intake.INSTANCE.indexerIn,
                        scoremid3
                ),



                Intake.INSTANCE.intakeTwoPowerFull,
                Intake.INSTANCE.intakeOnePowerFull,


                new waitForShots(2,.3),

                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)
                ),

                grabGate1_4,
                grabGate2_5,
                grabGate3_6,
                new Delay(1),

                Intake.INSTANCE.intakeTwoZero,

                new ParallelGroup(
                        new SequentialGroup(
                                new Delay(1),

                                new LambdaCommand().setStart(() ->
                                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)
                                )),
                        Intake.INSTANCE.indexerIn,
                        scoreGate1_7
                ),




                Intake.INSTANCE.intakeTwoPowerFull,
                Intake.INSTANCE.intakeOnePowerFull,




                new waitForShots(2,.3),

                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)
                ),

                new Delay(.75),
                grabgate3_8,
                grabGate4_9,
                grabGate10,
                new Delay(1),
                Intake.INSTANCE.intakeTwoZero,
                new ParallelGroup(
                        new SequentialGroup(
                                new Delay(1),

                                new LambdaCommand().setStart(() ->
                                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)
                                )),
                        Intake.INSTANCE.indexerIn,
                        scoreGate11
                ),


                Intake.INSTANCE.intakeTwoPowerFull,
                Intake.INSTANCE.intakeOnePowerFull,
                new waitForShots(3,.3),
                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)
                ),
                new Delay(.75),
                grabgate12,
                grabGate13,
                grabGate14,
                new Delay(1),
                Intake.INSTANCE.intakeTwoZero,
                new ParallelGroup(
                        new SequentialGroup(
                                new Delay(1),

                                new LambdaCommand().setStart(() ->
                                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)
                                )),
                        Intake.INSTANCE.indexerIn,
                        scoreGate_15
                ),


                Intake.INSTANCE.intakeTwoPowerFull,
                Intake.INSTANCE.intakeOnePowerFull,
                new waitForShots(3,.3),
                new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)
                ),

                grabClose_16,
                Intake.INSTANCE.intakeTwoZero, new ParallelGroup(
                new SequentialGroup(
                        new Delay(1.5),

                        new LambdaCommand().setStart(() ->
                                LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)
                        )),
                Intake.INSTANCE.indexerIn,
                scoreClose_17
        ),
                Intake.INSTANCE.intakeTwoPowerFull,

                new waitForShots(3,.3),



                Intake.INSTANCE.intakeOneZero,
                park_18


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