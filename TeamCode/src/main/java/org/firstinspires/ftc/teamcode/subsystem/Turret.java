package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.util.Mth;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

public class Turret implements Subsystem {
    public static final Turret INSTANCE = new Turret();
    public ServoEx turretOne;
    public ServoEx turretTwo;
    public Limelight3A limelight;
    double maxRange = 289.07; // max range of turret
    public double forward = maxRange * .5;

    @Override
    public void initialize() {
        turretOne = new ServoEx("turret_one");
        turretTwo = new ServoEx("turret_two");
    }

    public void turretPos(double angle){
        angle = angle / 180;
        angle = Mth.clamp(angle, -forward, forward);
        turretOne.setPosition(angle);
        turretTwo.setPosition(-angle);
    }

    @Override
    public void periodic() { // runs in the loop

    }
}
