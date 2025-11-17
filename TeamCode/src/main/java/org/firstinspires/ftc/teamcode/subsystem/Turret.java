package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.ServoEx;
@Config
public class Turret implements Subsystem {

    public ServoEx turretOne;
    public ServoEx turretTwo;
    public IMUEx imu;
    public Limelight3A limelight;

    private static final double TURRET_RANGE_DEG      = 289.07;
    private static final double TURRET_HALF_RANGE_DEG = TURRET_RANGE_DEG * 0.5;
    private static final double SERVO_CENTER = 0.5;

    private double turretAngleDeg = 0.0;
    private boolean autoAimEnabled = false;
    public static double kP = .15;
    public static double deadband = .3;


   public static double LIMELIGHT_TURRET_OFFSET_DEG = -3;

    @Override
    public void initialize() {
        Servo sdkTurretTwo = ActiveOpMode.hardwareMap().get(Servo.class, "turret_two");
        sdkTurretTwo.setDirection(Servo.Direction.REVERSE);
        turretOne = new ServoEx("turret_one");
        turretTwo = new ServoEx(sdkTurretTwo);

        imu = new IMUEx("imu", Direction.RIGHT, Direction.FORWARD);

        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        limelight.start();

        setPos(0.0);
    }

    public void enableAutoAim(boolean enabled) {
        this.autoAimEnabled = enabled;
    }

    public void setManualAngle(double angleDeg) {
        autoAimEnabled = false;
        turretAngleDeg = angleDeg;
        setPos(turretAngleDeg);
    }

    public void setPos(double angleDeg) {
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
        if (!autoAimEnabled) return;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;

        double tx = result.getTx() - LIMELIGHT_TURRET_OFFSET_DEG;

        if (Math.abs(tx) < deadband) return;

        turretAngleDeg += kP * tx;

        // Apply limelight ↔ turret alignment offset
        double commanded = turretAngleDeg;
        commanded = Range.clip(commanded, -TURRET_HALF_RANGE_DEG, TURRET_HALF_RANGE_DEG);

        setPos(commanded);
    }
}
