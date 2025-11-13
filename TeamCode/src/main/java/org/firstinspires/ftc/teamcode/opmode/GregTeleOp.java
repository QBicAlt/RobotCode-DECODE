package org.firstinspires.ftc.teamcode.opmode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.LauncherOuttakeFuckingThing;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

@TeleOp(name = "Greg TeleOp")
public class GregTeleOp extends NextFTCOpMode {
    private DriverControlledCommand driveCmd;


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
//        LauncherOuttakeFuckingThing.INSTANCE.slow.schedule();
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
                .whenBecomesTrue(Intake.INSTANCE.intakeTwoPowerHalf)
                .whenBecomesFalse(Intake.INSTANCE.intakeTwoZero)
                .whenBecomesFalse(Intake.INSTANCE.intakeOneZero);

        Gamepads.gamepad1().x()
                .whenBecomesTrue(Intake.INSTANCE.indexerIn);

        Gamepads.gamepad1().y()
                .whenBecomesTrue(Intake.INSTANCE.indexerOut);

/*
        Gamepads.gamepad1().rightTrigger().greaterThan(0.5)
                .whenBecomesTrue(LauncherOuttakeFuckingThing.INSTANCE.launch);

        Gamepads.gamepad1().rightTrigger().greaterThan(0.5)
                .whenBecomesFalse(LauncherOuttakeFuckingThing.INSTANCE.slow);
*/
    }


        @Override
        public void onUpdate () {
            BindingManager.update();
        }

}