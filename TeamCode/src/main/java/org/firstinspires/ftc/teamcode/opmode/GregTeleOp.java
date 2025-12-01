package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.limelightvision.LLResult;

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
@TeleOp(name = "Greg TeleOp")
public class GregTeleOp extends NextFTCOpMode {
    private DriverControlledCommand driveCmd;
    public static double turretAngle = 0;

    private FtcDashboard dashboard;

    public GregTeleOp() {
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
        follower().setPose(new Pose(0, 0, 0));
        follower().update();

        dashboard = FtcDashboard.getInstance();


        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        LauncherOuttakeFuckingThing.INSTANCE.setTargetRpm(0.0);
    }

    @Override
    public void onStartButtonPressed() {
        follower().startTeleopDrive();
        Turret.INSTANCE.enableAutoAim(true);
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


        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(
                        new LambdaCommand().setStart(Turret.INSTANCE::snapToRememberedGoalAndEnable))
                .whenBecomesTrue(new LambdaCommand().setStart(() ->LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open)));
        Gamepads.gamepad1().circle()
                .whenBecomesTrue(new LambdaCommand().setStart(() ->Turret.INSTANCE.manual()));




                        //       .whenBecomesFalse(new LambdaCommand().setStart(() ->
           //             turret.enableAutoAim(false)));


    }

    @Override
    public void onUpdate () {
        BindingManager.update();

        Pose pedroPose = follower().getPose();


        LLResult result = Turret.INSTANCE.limelight.getLatestResult();
        double turretAngleDeg = Turret.INSTANCE.getMeasuredAngleDeg();  // from your turret class

        double distLL = VisionDistanceHelper.distanceToGoalFromLimelight(result, turretAngleDeg);


        // --- DASHBOARD FIELD MAP DRAWING ---
        TelemetryPacket packet = new TelemetryPacket();
        Canvas field = packet.fieldOverlay();

        double x = pedroPose.getX();       // assumed inches in Pedro frame
        double y = pedroPose.getY();
        double h = pedroPose.getHeading(); // radians

        double robotRadius = 9.0; // ~9in radius for visualization

        // Draw a circle for the robot
        field.strokeCircle(x, y, robotRadius);

        // Draw a heading line
        double lineLen = robotRadius * 1.2;
        double hx = x + lineLen * Math.cos(h);
        double hy = y + lineLen * Math.sin(h);
        field.strokeLine(x, y, hx, hy);

        telemetry.addData("LL distance to goal (in)", distLL);
        telemetry.addData("target RPM", LauncherOuttakeFuckingThing.INSTANCE.getTargetRpm());
        telemetry.addData("motor rpm", LauncherOuttakeFuckingThing.INSTANCE.getCurrentRpm());
        telemetry.addData("turret_angle_deg", Turret.INSTANCE.getMeasuredAngleDeg());
        telemetry.addData("turret_volts", Turret.INSTANCE.turretFeedback.getVoltage());
        telemetry.addData("turret_state", Turret.INSTANCE.turretStateString());
        telemetry.addData("imu", Turret.INSTANCE.getRobotHeadingDeg());
        telemetry.addData("X", pedroPose.getX());
        telemetry.addData("Y", pedroPose.getY());

        telemetry.update();

        dashboard.sendTelemetryPacket(packet);

    }

}