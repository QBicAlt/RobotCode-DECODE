package org.firstinspires.ftc.teamcode.subsystem;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class Drive implements Subsystem {
    private final Follower follower;

    public Drive(HardwareMap hwm) {
        this.follower = Constants.createFollower(hwm);
        follower.startTeleopDrive();
        follower.update();
    }

    private MecanumConstants buildConstants() {
        MecanumConstants mecanumConstants = new MecanumConstants();
        mecanumConstants.setLeftRearMotorDirection(DcMotorSimple.Direction.REVERSE);
        mecanumConstants.setRightRearMotorDirection(DcMotorSimple.Direction.REVERSE);
        return mecanumConstants;
    }

    @Override
    public void initialize() {
        follower.startTeleopDrive();
    }

    public void drive(double x, double y, double r) {
        follower.update();
        follower.setTeleOpDrive(y, x, r, true);
    }
}
