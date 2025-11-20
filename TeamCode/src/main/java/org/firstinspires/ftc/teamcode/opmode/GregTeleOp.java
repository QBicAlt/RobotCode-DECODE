package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.subsystem.LauncherOuttakeFuckingThing.turret_Closed;
import static org.firstinspires.ftc.teamcode.subsystem.LauncherOuttakeFuckingThing.turret_Open;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.LauncherOuttakeFuckingThing;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

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

@Config
@TeleOp(name = "Greg TeleOp")
public class GregTeleOp extends NextFTCOpMode {
    public final Turret turret;
    private DriverControlledCommand driveCmd;
    public static double turretAngle = 0;

    public GregTeleOp() {
        turret = new Turret();
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(turret),
                new SubsystemComponent(Intake.INSTANCE),
                new SubsystemComponent(LauncherOuttakeFuckingThing.INSTANCE),
                BindingsComponent.INSTANCE
        );
    }
    @Override
    public void onInit() {
        follower().setPose(new Pose(0, 0, 0));
        follower().update();


        telemetry = new MultipleTelemetry(telemetry);
        LauncherOuttakeFuckingThing.INSTANCE.setTargetRpm(0.0);
    }

    @Override
    public void onStartButtonPressed() {
        follower().startTeleopDrive();
        driveCmd = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX()
        );

        driveCmd.schedule();

        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(Intake.INSTANCE.intakeOnePowerFull)
                .whenBecomesFalse(Intake.INSTANCE.intakeOneZero);
        Gamepads.gamepad1().leftTrigger().greaterThan(.5)
                .whenBecomesTrue(Intake.INSTANCE.intakeTwoPowerFull)
                .whenBecomesFalse(Intake.INSTANCE.intakeTwoZero)
                .whenBecomesFalse(new LambdaCommand().setStart(() ->LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Closed)));



        Gamepads.gamepad1().x()
                .whenBecomesTrue(Intake.INSTANCE.indexerIn);

        Gamepads.gamepad1().y()
                .whenBecomesTrue(Intake.INSTANCE.indexerOut);
        Gamepads.gamepad1().rightTrigger().greaterThan(0.5)
                .whenBecomesTrue(new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTargetRpm(LauncherOuttakeFuckingThing.LAUNCH_RPM)))
                .whenBecomesFalse(new LambdaCommand().setStart(() ->
                        LauncherOuttakeFuckingThing.INSTANCE.setTargetRpm(LauncherOuttakeFuckingThing.SLOW_RPM)));

        Gamepads.gamepad1().circle()
                .whenBecomesTrue(new LambdaCommand().setStart(() ->
                        turret.enableAutoAim(false)));

        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(
                        new LambdaCommand().setStart(turret::snapToRememberedGoalAndEnable))
                .whenBecomesTrue(new LambdaCommand().setStart(() ->LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)));




                        //       .whenBecomesFalse(new LambdaCommand().setStart(() ->
           //             turret.enableAutoAim(false)));


    }

    @Override
    public void onUpdate () {
        BindingManager.update();
        telemetry.addData("target RPM", LauncherOuttakeFuckingThing.INSTANCE.getTargetRpm());
        telemetry.addData("motor rpm", LauncherOuttakeFuckingThing.INSTANCE.getCurrentRpm());
        telemetry.addData("turret_angle_deg", turret.getMeasuredAngleDeg());
        telemetry.addData("turret_volts", turret.turretFeedback.getVoltage());
        telemetry.addData("turret_state", turret.turretStateString());
        telemetry.addData("imu", turret.getRobotHeadingDeg());
        telemetry.update();
    }

}