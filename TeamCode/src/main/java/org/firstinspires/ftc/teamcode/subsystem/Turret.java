package org.firstinspires.ftc.teamcode.subsystem;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

public class Turret implements Subsystem {
    public static final Turret INSTANCE = new Turret();
    public ServoEx turretOne;
    public ServoEx turretTwo;
    double maxRange = 289.07; // max range of turret
    public double forward = maxRange * .5;



    @Override
    public void initialize() {
     //   turretOne = new ServoEx("turret_one");
     //   turretTwo = new ServoEx("turret_two");

    }

  //  public void turretPos(double angle){
       // angle = angle/maxRange;
//turretOne.setPosition(angle);
      //  turretTwo.setPosition(-angle);
//    }

  //  public final Command turretPositon = new SetPosition(turretOne,turretPos();).requires(this);


    @Override
    public void periodic() { // runs in the loop

    }
}
