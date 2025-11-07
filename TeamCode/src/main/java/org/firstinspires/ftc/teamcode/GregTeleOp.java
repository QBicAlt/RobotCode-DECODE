package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Drive;
import org.firstinspires.ftc.teamcode.subsystem.LimeLight;

import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.bindings.BoundGamepad;

@Mercurial.Attach
@Drive.Attach
//@LimeLight.Attach
@TeleOp(name="Greg TeleOp", group="Linear OpMode")
public class GregTeleOp extends OpMode {
    public static BoundGamepad gamepad1;
    public static BoundGamepad gamepad2;
    public static double turn;

    @Override
    public void init() {
        gamepad1 = Mercurial.gamepad1();
        gamepad2 = Mercurial.gamepad2();
        turn = 0;
    }

    @Override
    public void loop() {
//        double tx = LimeLight.getX();
//        turn = Math.tanh(tx == 0 ? -LimeLight.getIMU().getAngularOrientation().firstAngle : tx);
        Drive.drive(
                gamepad1.leftStickX().state(),
                gamepad1.leftStickY().state(),
                gamepad1.rightStickX().state() /*+ turn*/
        );
    }
}
