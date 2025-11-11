package org.firstinspires.ftc.teamcode.opmode;

import org.firstinspires.ftc.teamcode.subsystem.Drive;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;

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
    }
}
