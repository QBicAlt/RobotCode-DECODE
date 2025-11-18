package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
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

    public static double kP = -0.25;
    public static double LIMELIGHT_X_OFFSET_DEG = -3;

    public static double filteredTx = 0.0;
    public static double TX_FILTER_ALPHA = 0.3;

    public static double LOCK_DEADBAND_DEG   = 0.5;
    public static double UNLOCK_DEADBAND_DEG = 2;

    private boolean lockedOnTarget = false;

    // remembered goal heading in field space
    private boolean hasGoalFieldHeading = false;
    private double lastGoalFieldHeadingDeg = 0.0;

    private int lostFrames = 0;
    // tunable from Dashboard if you want
    public static int MAX_LOST_FRAMES = 8;

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

    public double getRobotHeadingDeg() {
        return imu.get().inDeg;
    }

    private static double wrapDeg(double a) {
        while (a > 180) a -= 360;
        while (a < -180) a += 360;
        return a;
    }

    public void enableAutoAim(boolean enabled) {
        autoAimEnabled = enabled;
        if (!enabled) {
            lockedOnTarget = false;
        } else {
            filteredTx = 0.0;
        }
    }

    public void setManualAngle(double angleDeg) {
        enableAutoAim(false);
        setPos(angleDeg);
    }

    public void setPos(double angleDeg) {
        double clampedDeg = Range.clip(angleDeg,
                -TURRET_HALF_RANGE_DEG,
                TURRET_HALF_RANGE_DEG);

        turretAngleDeg = clampedDeg;

        double norm = clampedDeg / TURRET_RANGE_DEG;

        double servoOnePos = SERVO_CENTER + norm;
        double servoTwoPos = SERVO_CENTER - norm;

        servoOnePos = Range.clip(servoOnePos, 0.0, 1.0);
        servoTwoPos = Range.clip(servoTwoPos, 0.0, 1.0);

        turretOne.setPosition(servoOnePos);
        turretTwo.setPosition(servoTwoPos);
    }

    @Override
    public void periodic() {
        if (!autoAimEnabled) return;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            lockedOnTarget = false;

            // count how many frames we've lost the tag
            lostFrames++;

            // if it's just a momentary dropout, keep auto aim ON and
            // just hold the turret where it is
            if (lostFrames <= MAX_LOST_FRAMES) {
                return;
            }

            // tag has been gone for a while -> now we truly give up
            autoAimEnabled = false;
            return;
        } else {
            // got a valid result again, reset miss counter
            lostFrames = 0;
        }

        double rawTx = result.getTx();
        double tx = rawTx - LIMELIGHT_X_OFFSET_DEG;

        // low-pass filter
        filteredTx = TX_FILTER_ALPHA * filteredTx
                + (1.0 - TX_FILTER_ALPHA) * tx;

        double absFiltered = Math.abs(filteredTx);

        // hysteresis lock logic
        if (!lockedOnTarget) {
            if (absFiltered < LOCK_DEADBAND_DEG) {
                lockedOnTarget = true;
            }
        } else {
            if (absFiltered > UNLOCK_DEADBAND_DEG) {
                lockedOnTarget = false;
            }
        }
        // when basically centered, remember global goal heading
        if (Math.abs(tx) < LOCK_DEADBAND_DEG) {
            double robotHeading = getRobotHeadingDeg();
            // goal heading in field space = robot heading - turret angle (because turret + is "right")
            lastGoalFieldHeadingDeg = wrapDeg(robotHeading - turretAngleDeg);
            hasGoalFieldHeading = true;
        }
        if (lockedOnTarget) return;

        // P control toward the tag using filtered tx
        turretAngleDeg -= kP * filteredTx; // flip sign if needed

        turretAngleDeg = Range.clip(turretAngleDeg,
                -TURRET_HALF_RANGE_DEG,
                TURRET_HALF_RANGE_DEG);

        setPos(turretAngleDeg);
    }

    public void snapToRememberedGoalAndEnable() {
        // Always turn on auto aim
        enableAutoAim(true);

        // If tag is currently visible, DON'T snap with IMU—just let LL tracking handle it
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return;
        }

        // If no tag but we have a remembered heading, snap toward it
        if (!hasGoalFieldHeading) {
            return;
        }

        double currentHeading = getRobotHeadingDeg();
        double desiredTurretAngle = wrapDeg(currentHeading - lastGoalFieldHeadingDeg);

        desiredTurretAngle = Range.clip(desiredTurretAngle,
                -TURRET_HALF_RANGE_DEG,
                TURRET_HALF_RANGE_DEG);

        setPos(desiredTurretAngle);
    }
}
