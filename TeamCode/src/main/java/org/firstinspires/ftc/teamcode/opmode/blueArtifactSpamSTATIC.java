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
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "blueArtifactSpam")
public class blueArtifactSpamSTATIC extends NextFTCOpMode {

    public blueArtifactSpamSTATIC() {
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
                                .addPath(
                                        new BezierLine(
                                                new Pose(17.8, 118),
                                                new Pose(48.000, 96.000)
                                        )
                                )
                                .setTangentHeadingInterpolation()
                                .setReversed()
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    // 2) go grab first stack
    public static final Command grabfirst2 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierCurve(
                                                new Pose(48.000, 96.000),
                                                new Pose(69.753, 81.739),
                                                new Pose(19.000, 84.000)
                                        )
                                )

                                .setConstantHeadingInterpolation(Math.toRadians(180))

                                .build(),
                        .6,
                        false

                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    // 3) score first stack
    public static final Command scorefirst3 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                new Pose(19.000, 84.000),
                                                new Pose(55.000, 84.000)
                                        )
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(180))
                                //     .setReversed()
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    // 4) grab second stack
    public static final Command grabsecond4 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierCurve(
                                                new Pose(55.000, 84.000),
                                                new Pose(58.932, 54.603),
                                                new Pose(25.651, 57.764)
                                        )
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(180))
                                .build(),
                        .6,
                        false
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    // 5) score second stack
    public static final Command scoresecond5 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                new Pose(25.651, 57.764),
                                                new Pose(55.000, 84.000)
                                        )
                                )
                                .setConstantHeadingInterpolation(Math.toRadians(180))
                                //      .setReversed()
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    // 6) grab from gate
    public static final Command grabfromgate6 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                new Pose(55.000, 84.000),
                                                new Pose(15.000, 58.000)
                                        )
                                )
                                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    public static final Command grabfromgate2_6 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierCurve(
                                                new Pose(15.000, 58.000),
                                                new Pose(31.797, 54.603),
                                                new Pose(9, 54.768)
                                        )
                                )
                                .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    // 7) score from gate
    public static final Command scoregate7 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierLine(
                                                new Pose(9.000, 54.768),
                                                new Pose(50.000, 84.000)
                                        )
                                )
                                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    // 8) grab from red zone
    public static final Command grabfromredzone8 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierCurve(
                                                new Pose(50.000, 84.000),
                                                new Pose(31.131, 75.579),
                                                new Pose(31.131, 48.111),
                                                new Pose(7.000, 50.000)
                                        )
                                )
                                .setTangentHeadingInterpolation()
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    // 9) score from red zone
    public static final Command scorefromredzone9 = new LambdaCommand()
            .setStart(() -> {
                Follower follower = PedroComponent.follower();
                follower.followPath(
                        follower.pathBuilder()
                                .addPath(
                                        new BezierCurve(
                                                new Pose(7.000, 50.000),
                                                new Pose(23.473, 47.112),
                                                new Pose(50.000, 84.000)
                                        )
                                )
                                .setTangentHeadingInterpolation()
                                .setReversed()
                                .build()
                );
            })
            .setIsDone(() -> !PedroComponent.follower().isBusy());

    @Override
    public void onInit() {
        // 1. DISABLE Relocalization globally
        // This ensures the robot NEVER jumps position based on tags.
        VisionDistanceHelper.RELOCALIZATION_ENABLED = false;

        // 2. ENABLE Auto Calculation for RPM/Angle
        // This ensures the launcher uses the camera to figure out speed, but doesn't move the robot pose.
        LauncherOuttakeFuckingThing.INSTANCE.enableAutoCalculation();

        LLResult result = Turret.INSTANCE.runLimelight();
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
    }

    @Override
    public void onStartButtonPressed() {
        // Tell Pedro where we actually are at the start (artifact pile)
        Follower follower = PedroComponent.follower();
        follower.setPose(new Pose(17.8, 118, Math.toRadians(144)));

        Command auto = new SequentialGroup(
                Intake.INSTANCE.indexerIn,
                new LambdaCommand().setStart(() -> LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)),

                new LambdaCommand().setStart(() -> LauncherOuttakeFuckingThing.INSTANCE.setManualShooter(2500, 25)),



                new LambdaCommand().setStart(() -> Turret.INSTANCE.enableAutoAim(true)),


                // --- MANUAL OVERRIDE OPTION ---
                // If you ever want to STOP using the camera for speed and set it manually:
                // new LambdaCommand().setStart(() -> LauncherOuttakeFuckingThing.INSTANCE.setManualShooter(3500, 25)),
                //
                // To go back to using the camera:
                // new LambdaCommand().setStart(() -> LauncherOuttakeFuckingThing.INSTANCE.enableAutoCalculation()),
                Intake.INSTANCE.intakeOnePowerFull,
                Intake.INSTANCE.intakeTwoZero,
                scoreFirst1,
                new LambdaCommand().setStart(() -> LauncherOuttakeFuckingThing.INSTANCE.enableAutoCalculation()),
               Intake.INSTANCE.intakeTwoPowerFull,
                new LambdaCommand().setStart(() -> LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)),

                new Delay(1.5),

                new LambdaCommand().setStart(() -> LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)),

                grabfirst2,
                Intake.INSTANCE.intakeTwoZero,
                scorefirst3,
                new LambdaCommand().setStart(() -> Turret.INSTANCE.snapToRememberedGoalAndEnable()),


                new Delay(.75),


                Intake.INSTANCE.intakeTwoPowerFull,
                new LambdaCommand().setStart(() -> LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)),
                new Delay(1.5),
                Intake.INSTANCE.intakeTwoZero,
                new LambdaCommand().setStart(() -> LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)),
                Intake.INSTANCE.intakeTwoPowerFull,

                grabsecond4,
                Intake.INSTANCE.intakeTwoZero,
                scoresecond5,
                new LambdaCommand().setStart(() -> Turret.INSTANCE.snapToRememberedGoalAndEnable()),
                new Delay(.75),

                new LambdaCommand().setStart(() -> LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)),
                Intake.INSTANCE.intakeTwoPowerFull,
                new Delay(1.5),
                Intake.INSTANCE.intakeTwoZero,
                new LambdaCommand().setStart(() -> LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)),
                Intake.INSTANCE.intakeTwoPowerFull,
                grabfromgate6,
                new Delay(3),
                grabfromgate2_6,
                new Delay(.75),
                Intake.INSTANCE.intakeTwoZero,
                scoregate7,
                new LambdaCommand().setStart(() -> Turret.INSTANCE.snapToRememberedGoalAndEnable()),
                new Delay(1),
                new LambdaCommand().setStart(() -> LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)),
                Intake.INSTANCE.intakeTwoPowerFull


                //     new LambdaCommand().setStart(() -> LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)),
             //   Intake.INSTANCE.intakeTwoPowerFull
                //grabfromredzone8,
                //scorefromredzone9
        );

        // ... rest of auto ..

        auto.schedule();
    }
}