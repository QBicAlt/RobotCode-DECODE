package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.Mth;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;

public class Turret implements Subsystem {
    public ServoEx turretOne;
    public ServoEx turretTwo;
    public final OpMode opMode;
    public Limelight3A limelight;
    double maxRange = 289.07; // max range of turret
    public double forward = maxRange * .5;

    public Turret(OpMode opMode) {
        this.opMode = opMode;
    }

    @Override
    public void initialize() {
        turretOne = new ServoEx("turret_one");
        turretTwo = new ServoEx("turret_two");
        limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
    }

    public void setPos(double angle){
        angle = angle / 180;
        angle = Mth.clamp(angle, -forward, forward);
        turretOne.setPosition(angle);
        turretTwo.setPosition(-angle);
    }

    @Override
    public void periodic() { // runs in the loop
        LLResult result = limelight.getLatestResult();
        double tx = result.getTx();


    }
}
