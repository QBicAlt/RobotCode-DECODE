package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.hardware.android.GpioPin;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.ServoEx;

public class Turret implements Subsystem {

    public ServoEx turretOne;
    public ServoEx turretTwo;
    public IMUEx imu;
    public Limelight3A limelight;

    // Turret mechanical range (in DEGREES) – total sweep
    private static final double TURRET_RANGE_DEG      = 289.07;
    private static final double TURRET_HALF_RANGE_DEG = TURRET_RANGE_DEG * 0.5;

    // Where center is in servo units (tune this if needed!)
    private static final double SERVO_CENTER = 0.5;

    @Override
    public void initialize() {
        Servo sdkTurretTwo = ActiveOpMode.hardwareMap().get(Servo.class, "turret_two");
        sdkTurretTwo.setDirection(Servo.Direction.REVERSE);
        turretOne = new ServoEx("turret_one");
        turretTwo = new ServoEx(sdkTurretTwo);

        imu = new IMUEx("imu", Direction.RIGHT, Direction.FORWARD);

        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        limelight.start();
    }


    public void setPos(double angleDeg) {
        // angleDeg -= imu.get().inDeg;

        double clampedDeg = Math.max(-TURRET_HALF_RANGE_DEG,
                Math.min(TURRET_HALF_RANGE_DEG, angleDeg));

        double norm = clampedDeg / TURRET_RANGE_DEG;

        double servoOnePos = SERVO_CENTER + norm;    // center at 0.5
        double servoTwoPos = SERVO_CENTER - norm;    // mirrored

        servoOnePos = Math.max(0.0, Math.min(1.0, servoOnePos));
        servoTwoPos = Math.max(0.0, Math.min(1.0, servoTwoPos));

        turretOne.setPosition(servoOnePos);
        turretTwo.setPosition(servoTwoPos);
    }

    @Override
    public void periodic() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx();
            // you can use tx to auto-aim: setPosDegrees(tx + offset), etc.
        }
    }
}

