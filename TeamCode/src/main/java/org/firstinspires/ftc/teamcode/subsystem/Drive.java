package org.firstinspires.ftc.teamcode.subsystem;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class Drive implements Subsystem {
    private final Follower follower;

    public Drive(HardwareMap hwm) {
        this.follower = new Follower(
                new FollowerConstants(),
                new PinpointLocalizer(
                        hwm,
                        new PinpointConstants()
                ),
                new Mecanum(
                        hwm,
                        buildConstants()
                )
        );
    }

    private MecanumConstants buildConstants() {
        MecanumConstants mecanumConstants = new MecanumConstants();
        mecanumConstants.setLeftRearMotorDirection(DcMotorSimple.Direction.REVERSE);
        mecanumConstants.setRightRearMotorDirection(DcMotorSimple.Direction.REVERSE);
        return mecanumConstants;
    }

    public void drive(double x, double y, double r) {
        follower.setTeleOpDrive(y, x, r, true);
    }
}
