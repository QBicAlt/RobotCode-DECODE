package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.bindings.BoundGamepad;

@Mercurial.Attach
@Drive.Attach
public class TeleOp extends OpMode {
    public static BoundGamepad gamepad1;
    public static BoundGamepad gamepad2;

    @Override
    public void init() {
        gamepad1 = Mercurial.gamepad1();
        gamepad2 = Mercurial.gamepad2();
    }

    @Override
    public void loop() {
        Drive.drive(
                gamepad1.leftStickX().state(),
                gamepad1.leftStickY().state(),
                gamepad1.rightStickX().state()
        );
    }
}
