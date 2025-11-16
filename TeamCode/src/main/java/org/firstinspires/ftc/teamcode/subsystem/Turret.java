package org.firstinspires.ftc.teamcode.subsystem;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.util.Mth;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.ServoEx;

public class Turret implements Subsystem {
    public ServoEx turretOne;
    public ServoEx turretTwo;
    public IMUEx imu;
    public Limelight3A limelight;
    double maxRange = 289.07; // max range of turret
    public double halfRange = maxRange * .5;

    @Override
    public void initialize() {
        turretOne = new ServoEx("turret_one");
        turretTwo = new ServoEx("turret_two");
        imu = new IMUEx("imu", Direction.RIGHT, Direction.FORWARD);
        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        limelight.start();
    }

    public void setPos(double angle){
        angle -= imu.get().inDeg;
        angle = angle / 180;
        angle = Mth.clamp(angle, -halfRange, halfRange);
        turretOne.setPosition(angle);
        turretTwo.setPosition(-angle);
    }

    @Override
    public void periodic() { // runs in the loop
        LLResult result = limelight.getLatestResult();
        double tx = result.getTx();


    }
}
