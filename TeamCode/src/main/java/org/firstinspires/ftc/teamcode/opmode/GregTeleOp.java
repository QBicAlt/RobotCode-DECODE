package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.Drive;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp
public class GregTeleOp extends NextFTCOpMode {
    private Drive drive;
    @Override
    public void onInit() {
        drive = new Drive(hardwareMap);

        addComponents(
                new SubsystemComponent(drive)
        );
    }

    @Override
    public void onUpdate() {
        drive.drive(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x
        );

        telemetry.addData("gamepad1 left_stick_x:", gamepad1.left_stick_x);
        telemetry.update();
    }
}
