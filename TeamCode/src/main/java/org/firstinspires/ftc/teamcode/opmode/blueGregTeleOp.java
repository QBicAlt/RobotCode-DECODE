package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import dev.nextftc.core.commands.groups.SequentialGroup;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.LauncherOuttakeFuckingThing;
import org.firstinspires.ftc.teamcode.subsystem.Turret;
import org.firstinspires.ftc.teamcode.subsystem.VisionDistanceHelper;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.LimelightManager;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.Gamepad;

@Config
@TeleOp(name = "Blue TeleOp")
public class blueGregTeleOp extends NextFTCOpMode {

    private DriverControlledCommand driveCmd;
    private FtcDashboard dashboard;
    private LimelightManager limelightManager;

    // --- TUNING: TURN SPEED LIMIT ---
    // 0.5 means the robot spins at 50% speed when the stick is fully pushed
    public static double TURN_SENSITIVITY = 0.8;

    public blueGregTeleOp() {
        VisionDistanceHelper.GOAL_TAG_X_IN = 144 - 127.64;
        VisionDistanceHelper.GOAL_TAG_Y_IN = 130.37;
        VisionDistanceHelper.GOAL_TARGET_X_FAR = 0;
        VisionDistanceHelper.GOAL_TARGET_Y_FAR = 138;
        VisionDistanceHelper.GOAL_TARGET_X = 0;
        VisionDistanceHelper.GOAL_TARGET_Y = 136;

        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(Turret.INSTANCE),
                new SubsystemComponent(Intake.INSTANCE),
                new SubsystemComponent(LauncherOuttakeFuckingThing.INSTANCE),
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        limelightManager = new LimelightManager(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        if (PoseStorage.hasAutoRun) {
            follower().setPose(PoseStorage.currentPose);
            telemetry.addLine("Localization: Loaded from Auto Snapshot");

            // Restore Turret
        } else {
            follower().setPose(new Pose(72, 72, 0));
            telemetry.addLine("Localization: Default Start");
            Turret.INSTANCE.resetEncoderLogic();
        }
        follower().update();
    }

    @Override
    public void onStartButtonPressed() {
        Turret.INSTANCE.enableOdometryAim();
        LauncherOuttakeFuckingThing.INSTANCE.enableAutoCalculation();
        LauncherOuttakeFuckingThing.autoCalculate = true;

        follower().startTeleopDrive();

        // -----------------------------------------------------------
        // 1. DRIVETRAIN SENSITIVITY FIX
        // We wrap the turn input in a lambda to multiply it by 0.5
        // -----------------------------------------------------------
        driveCmd = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX().negate(),
                () -> -gamepad1.right_stick_x * TURN_SENSITIVITY // <--- SENSITIVITY APPLIED HERE
        );

        driveCmd.schedule();

        // -----------------------------------------------------------
        // 2. TOGGLE COMPENSATION (Gamepad 1 D-Pad Up)
        // -----------------------------------------------------------
        Gamepads.gamepad1().dpadUp().whenBecomesTrue(new LambdaCommand().setStart(() -> {
            LauncherOuttakeFuckingThing.INSTANCE.toggleCompensation();
        }));

        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(Intake.INSTANCE.intakeOnePowerFull)
                .whenBecomesFalse(Intake.INSTANCE.intakeOneZero);
        Gamepads.gamepad1().leftTrigger().greaterThan(.5)
                .whenBecomesTrue(Intake.INSTANCE.intakeTwoPowerFull)
                .whenBecomesFalse(Intake.INSTANCE.intakeTwoZero)
                .whenBecomesFalse(new LambdaCommand().setStart(() -> LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)));

        Gamepads.gamepad1().x().whenBecomesTrue(Intake.INSTANCE.indexerIn);
        Gamepads.gamepad1().y().whenBecomesTrue(Intake.INSTANCE.indexerOut);

        Gamepads.gamepad1().rightTrigger().greaterThan(0.5)
                .whenBecomesTrue(new LambdaCommand().setStart(() -> LauncherOuttakeFuckingThing.INSTANCE.setTargetRpm(LauncherOuttakeFuckingThing.LAUNCH_RPM)))
                .whenBecomesFalse(new LambdaCommand().setStart(() -> LauncherOuttakeFuckingThing.INSTANCE.setTargetRpm(LauncherOuttakeFuckingThing.SLOW_RPM)));

        Gamepads.gamepad1().share().whenBecomesTrue(new LambdaCommand().setStart(() -> {
            double currentHeading = follower().getPose().getHeading();
            Pose visionPose = limelightManager.getPose(currentHeading);
            if (visionPose != null) {
                follower().setPose(visionPose);
                gamepad1.rumble(500);
            }
        }));

        Gamepads.gamepad2().triangle()
                .whenBecomesTrue(new SequentialGroup(
                        new LambdaCommand().setStart(() -> {
                            Turret.INSTANCE.setManualAngle(0.0);
                            LauncherOuttakeFuckingThing.INSTANCE.setManualShooter(LauncherOuttakeFuckingThing.MANUAL_RPM, LauncherOuttakeFuckingThing.MANUAL_ANGLE);
                        }).setIsDone(() -> Math.abs(Turret.INSTANCE.getMeasuredAngleDeg()) < 5.0),
                        new LambdaCommand().setStart(() -> Turret.INSTANCE.off())
                ));


        Gamepads.gamepad2().dpadLeft().whenBecomesTrue(new LambdaCommand().setStart(() -> {
            VisionDistanceHelper.GOAL_TARGET_X_FAR -= 2.0;
            gamepad2.rumble(100); // Haptic feedback so you know it registered
        }));

        Gamepads.gamepad2().dpadRight().whenBecomesTrue(new LambdaCommand().setStart(() -> {
            VisionDistanceHelper.GOAL_TARGET_X_FAR += 2.0;
            gamepad2.rumble(100);
        }));

        Gamepads.gamepad2().square()
                .whenBecomesTrue(new LambdaCommand().setStart(() -> {
                    LauncherOuttakeFuckingThing.INSTANCE.enableAutoCalculation();
                    Turret.INSTANCE.enableOdometryAim();
                }));

        Gamepads.gamepad1().rightBumper().whenBecomesTrue(new LambdaCommand().setStart(() -> LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)));
        Gamepads.gamepad1().circle().whenBecomesTrue(new LambdaCommand().setStart(() -> Turret.INSTANCE.setManualAngle(0)));

        Gamepads.gamepad2().rightBumper().whenBecomesTrue(Intake.INSTANCE.extendClimb1).whenBecomesTrue(Intake.INSTANCE.extendClimb2);
        Gamepads.gamepad2().leftBumper().whenBecomesTrue(Intake.INSTANCE.retractClimb1).whenBecomesTrue(Intake.INSTANCE.retractClimb2);

        Gamepads.gamepad1().leftStickButton().whenBecomesTrue(new LambdaCommand().setStart(() -> follower().setPose(new Pose(11.37, 8.5f, Math.toRadians(180)))));
    }

    @Override
    public void onUpdate() {
        BindingManager.update();

        // ---------------------------------------------------------------------
        // SAFETY MANUAL RESET LOGIC
        // ---------------------------------------------------------------------
        boolean isSafetyPressed = gamepad2.cross;
        double manualPower = gamepad2.left_stick_x;

        if (!isSafetyPressed) {
            Turret.INSTANCE.resetLockout = false;
        }

        if (isSafetyPressed && Math.abs(manualPower) > 0.05 && !Turret.INSTANCE.resetLockout) {
            if (Turret.INSTANCE.isLimitPressed()) {
                Turret.INSTANCE.setRawPower(0);
                Turret.INSTANCE.resetEncoderLogic();
                Turret.INSTANCE.enableOdometryAim();
                if (gamepad2 != null) gamepad2.rumble(500);
                Turret.INSTANCE.resetLockout = true;
            } else {
                Turret.INSTANCE.setRawPower(manualPower * 0.4);
            }
        } else {
            if (Turret.INSTANCE.getState() == Turret.TurretState.RAW_MANUAL) {
                Turret.INSTANCE.off();
            }
        }

        Pose pedroPose = follower().getPose();
        double distInches = VisionDistanceHelper.distanceToGoalInches(pedroPose);

        TelemetryPacket packet = new TelemetryPacket();
        Canvas field = packet.fieldOverlay();

        double x = pedroPose.getX();
        double y = pedroPose.getY();
        double h = pedroPose.getHeading();

        field.strokeCircle(x, y, 9.0);
        field.strokeLine(x, y, x + 10 * Math.cos(h), y + 10 * Math.sin(h));

        telemetry.addData("Dist to Goal", distInches);
        // --- VISUAL CONFIRMATION OF TOGGLE ---
        telemetry.addData("Vel Compensation", LauncherOuttakeFuckingThing.useVelocityCompensation ? "ON" : "OFF");
        telemetry.addData("Calculated Offset", LauncherOuttakeFuckingThing.calculatedTurretOffset);

        telemetry.addData("target RPM", LauncherOuttakeFuckingThing.INSTANCE.getTargetRpm());
        telemetry.addData("motor rpm", LauncherOuttakeFuckingThing.INSTANCE.getCurrentRpm());
        telemetry.addData("turret_angle", Turret.INSTANCE.getMeasuredAngleDeg());
        telemetry.addData("Turret Mode", Turret.INSTANCE.getState());

        telemetry.addData("X", pedroPose.getX());
        telemetry.addData("Y", pedroPose.getY());

        telemetry.update();
        dashboard.sendTelemetryPacket(packet);
    }
}