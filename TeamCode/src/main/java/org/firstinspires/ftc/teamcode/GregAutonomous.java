package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Drive;
import org.firstinspires.ftc.teamcode.subsystem.LimeLight;

import dev.frozenmilk.mercurial.Mercurial;

@Mercurial.Attach
@Drive.Attach
@LimeLight.Attach
@TeleOp(name="Greg Auto", group="Autonomous")
public class GregAutonomous extends OpMode {
    @Override
    public void init() {

    }

    @Override
    public void loop() {
        Drive.drive();
    }
}
