package org.firstinspires.ftc.teamcode.opmode;

import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.LauncherOuttakeFuckingThing;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp(name = "GregAuto")
public class GregAuto extends NextFTCOpMode {
    public double turretAngle = 0;
    public int pattern = 0;

    public GregAuto() {
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
        LLResult result = Turret.INSTANCE.runLimelight();
        if (pattern == 2) {
            Paths.GPP.schedule();
        } else if (pattern == 1) {
            Paths.PGP.schedule();
        } else {
            Paths.PPG.schedule();
        }
    }

    @Override
    public void onUpdate() {
        if (pattern == 2) {
            Paths.GPP.schedule();
        } else if (pattern == 1) {
            Paths.PGP.schedule();
        } else {
            Paths.PPG.schedule();
        }
    }
}
