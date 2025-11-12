package org.firstinspires.ftc.teamcode.subsystem;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;

public class Turret implements Subsystem {
    public static final Turret INSTANCE = new Turret();
    public ServoEx turretOne;

    @Override
    public void initialize() {
        turretOne = new ServoEx("turret_one");
    }

    @Override
    public void periodic() { // runs in the loop
        turretOne.setPosition(0.0);
    }
}
