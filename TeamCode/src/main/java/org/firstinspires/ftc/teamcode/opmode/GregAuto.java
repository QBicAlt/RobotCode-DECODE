package org.firstinspires.ftc.teamcode.opmode;

import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.LauncherOuttakeFuckingThing;
import org.firstinspires.ftc.teamcode.subsystem.Turret;

import java.util.List;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp(name = "GregAuto")
public class GregAuto extends NextFTCOpMode {
    public double turretAngle = 0;

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
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

        for (LLResultTypes.FiducialResult tag : tags) {
           int tagID = tag.getFiducialId();
            if (tagID == 21) {
                Paths.GPP.schedule();
                break;
            } else if (tagID == 22) {
                Paths.PGP.schedule();
                break;
            } else if (tagID == 23) {
                Paths.PPG.schedule();
                break;
            }
        }

        Intake.INSTANCE.intakeOnePowerFull.schedule();

        Turret.INSTANCE.snapToRememberedGoalAndEnable();
        LauncherOuttakeFuckingThing.INSTANCE.setTurretLatch(LauncherOuttakeFuckingThing.turret_Open);
    }
}
