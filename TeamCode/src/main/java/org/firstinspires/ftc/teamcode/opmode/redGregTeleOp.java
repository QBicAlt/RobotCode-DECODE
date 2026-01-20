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


@Config
@TeleOp(name = "red TeleOp")
public class redGregTeleOp extends NextFTCOpMode {

    private DriverControlledCommand driveCmd;
    private FtcDashboard dashboard;

    public redGregTeleOp() {
        // Set Blue Goal Coords
        VisionDistanceHelper.GOAL_TAG_X_IN=  127.64;
        VisionDistanceHelper.GOAL_TAG_Y_IN  = 130.37;
        VisionDistanceHelper.GOAL_TARGET_X_FAR =  139;
        VisionDistanceHelper.GOAL_TARGET_Y_FAR = 144;

        VisionDistanceHelper.GOAL_TARGET_X =  132;
        VisionDistanceHelper.GOAL_TARGET_Y = 131;

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
        follower().setPose(new Pose(72, 72, 0));
        follower().update();

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Zero Turret Encoder
        Turret.INSTANCE.resetEncoderLogic();
     //   LauncherOuttakeFuckingThing.INSTANCE.setTargetRpm(0.0);
    }

    @Override
    public void onStartButtonPressed() {

        // Enable Systems
        Turret.INSTANCE.enableOdometryAim();
        LauncherOuttakeFuckingThing.INSTANCE.enableAutoCalculation();
        LauncherOuttakeFuckingThing.autoCalculate = true;

        follower().startTeleopDrive();

        driveCmd = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX().negate(),
                Gamepads.gamepad1().rightStickX().negate()
        );

        driveCmd.schedule();

        // Bindings
        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(Intake.INSTANCE.intakeOnePowerFull)
                .whenBecomesFalse(Intake.INSTANCE.intakeOneZero);
        Gamepads.gamepad1().leftTrigger().greaterThan(.5)
                .whenBecomesTrue(Intake.INSTANCE.intakeTwoPowerFull)
                .whenBecomesFalse(Intake.INSTANCE.intakeTwoZero)
                .whenBecomesFalse(new LambdaCommand().setStart(() -> LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)));

        Gamepads.gamepad1().x()
                .whenBecomesTrue(Intake.INSTANCE.indexerIn);

        Gamepads.gamepad1().y()
                .whenBecomesTrue(Intake.INSTANCE.indexerOut);

        Gamepads.gamepad1().rightTrigger().greaterThan(0.5)
                .whenBecomesTrue(new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTargetRpm(LauncherOuttakeFuckingThing.LAUNCH_RPM)))
                .whenBecomesFalse(new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTargetRpm(LauncherOuttakeFuckingThing.SLOW_RPM)));

        // Manual Fallback / Reset
        Gamepads.gamepad2().triangle()
                .whenBecomesTrue(new SequentialGroup(
                        new LambdaCommand()
                                .setStart(() -> {
                                    Turret.INSTANCE.setManualAngle(0.0);

                                    // Static fallback values
                                    LauncherOuttakeFuckingThing.INSTANCE.setManualShooter(
                                            LauncherOuttakeFuckingThing.MANUAL_RPM,
                                            LauncherOuttakeFuckingThing.MANUAL_ANGLE
                                    );
                                })
                                .setIsDone(() -> Math.abs(Turret.INSTANCE.getMeasuredAngleDeg()) < 5.0),
                        new LambdaCommand().setStart(() -> Turret.INSTANCE.off())
                ));

        // Re-Enable Auto
        Gamepads.gamepad2().square()
                .whenBecomesTrue(new LambdaCommand().setStart(() -> {
                    LauncherOuttakeFuckingThing.INSTANCE.enableAutoCalculation();
                    Turret.INSTANCE.enableOdometryAim();
                }));

        // Latch Toggle
        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)));

        Gamepads.gamepad1().circle()
                .whenBecomesTrue(new LambdaCommand().setStart(() -> Turret.INSTANCE.setManualAngle(0)));

        Gamepads.gamepad2().rightBumper()
                .whenBecomesTrue(Intake.INSTANCE.extendClimb1)
                .whenBecomesTrue(Intake.INSTANCE.extendClimb2);

        Gamepads.gamepad2().leftBumper()
                .whenBecomesTrue(Intake.INSTANCE.retractClimb1)
                .whenBecomesTrue(Intake.INSTANCE.retractClimb2);

        Gamepads.gamepad1().leftStickButton()
                .whenBecomesTrue(new LambdaCommand().setStart(() ->
                        follower().setPose(new Pose(11.37, 8.5f, Math.toRadians(180)).mirror())
                ));




    }

    @Override
    public void onUpdate () {
        BindingManager.update();

        Pose pedroPose = follower().getPose();
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
        telemetry.addData("hasBall:", LauncherOuttakeFuckingThing.INSTANCE.hasBall());


        telemetry.update();
        dashboard.sendTelemetryPacket(packet);
    }
}