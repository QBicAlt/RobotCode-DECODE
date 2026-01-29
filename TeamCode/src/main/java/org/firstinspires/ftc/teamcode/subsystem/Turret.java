package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;

import org.firstinspires.ftc.teamcode.subsystem.LauncherOuttakeFuckingThing;

@Config
public class Turret implements Subsystem {

    public static final Turret INSTANCE = new Turret();
    public Turret() {}

    public enum TurretState {
        OFF,
        MANUAL,
        ODOMETRY_AIM,
        RAW_MANUAL
    }

    private TurretState state = TurretState.OFF;

    private CRServo turretOne;
    private CRServo turretTwo;
    private DcMotorEx encoderPortContainer;
    private DigitalChannel turretLimit;

    public static String ENCODER_PORT_NAME = "leftRear";
    public static String LIMIT_SWITCH_NAME = "turret_sensor";

    public boolean resetLockout = false;

    public static double TICKS_PER_REV_ENCODER = 4096;
    public static double TURRET_GEAR_RATIO = 112.0 / 19.0;
    public static boolean ENCODER_REVERSE = true;
    public static double MIN_LIMIT_DEG = -130.0;
    public static double MAX_LIMIT_DEG = 160;
    public static boolean INVERT_SERVO_OUTPUT = true;
    public static double kP = 0.02;
    public static double kI = 0.0;
    public static double kD = 0.0027;
    public static double MAX_POWER = 1.0;

    // --- DELETED PHYSICS CONSTANTS (Now in Launcher) ---
    // FLYWHEEL_RADIUS_IN and EFFICIENCY are gone from here.

    private double turretSetpointDeg = 0.0;
    private double lastError = 0.0;
    private double currentOffset = 0.0;
    private double manualRawPower = 0.0;
    public static double angle_tester = 0.0;

    @Override
    public void initialize() {
        turretOne = ActiveOpMode.hardwareMap().get(CRServo.class, "turret_one");
        turretTwo = ActiveOpMode.hardwareMap().get(CRServo.class, "turret_two");
        turretOne.setDirection(CRServo.Direction.REVERSE);
        turretTwo.setDirection(CRServo.Direction.REVERSE);

        encoderPortContainer = ActiveOpMode.hardwareMap().get(DcMotorEx.class, ENCODER_PORT_NAME);
        turretLimit = ActiveOpMode.hardwareMap().get(DigitalChannel.class, LIMIT_SWITCH_NAME);
        turretLimit.setMode(DigitalChannel.Mode.INPUT);

        resetEncoderLogic();
        turretSetpointDeg = 0.0;
        state = TurretState.MANUAL;
    }

    public TurretState getState() {
        return state;
    }

    public boolean isLimitPressed() {
        return !turretLimit.getState();
    }

    public void setRawPower(double power) {
        this.state = TurretState.RAW_MANUAL;
        this.manualRawPower = power;
    }

    public void resetEncoderLogic() {
        if(encoderPortContainer != null) {
            double raw = encoderPortContainer.getCurrentPosition();
            if(ENCODER_REVERSE) raw = -raw;
            currentOffset = raw;
            turretSetpointDeg = 0;
            angle_tester = 0;
        }
    }

    public void restoreCalibrationFromAuto(double lastKnownAngleDeg) {
        if (encoderPortContainer == null) return;
        double raw = encoderPortContainer.getCurrentPosition();
        if (ENCODER_REVERSE) raw = -raw;
        double ticksPerDeg = (TICKS_PER_REV_ENCODER * TURRET_GEAR_RATIO) / 360.0;
        currentOffset = raw - (lastKnownAngleDeg * ticksPerDeg);
        turretSetpointDeg = lastKnownAngleDeg;
        angle_tester = lastKnownAngleDeg;
    }

    public double getMeasuredAngleDeg() {
        if (encoderPortContainer == null) return 0.0;
        double raw = encoderPortContainer.getCurrentPosition();
        if(ENCODER_REVERSE) raw = -raw;
        double relativeTicks = raw - currentOffset;
        double ticksPerDeg = (TICKS_PER_REV_ENCODER * TURRET_GEAR_RATIO) / 360.0;
        return relativeTicks / ticksPerDeg;
    }

    // --- DELETED getDynamicProjectileSpeed() (No longer needed) ---

    private double wrapDeg(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    public void setManualAngle(double angle) {
        state = TurretState.MANUAL;
        angle_tester = Range.clip(angle, MIN_LIMIT_DEG, MAX_LIMIT_DEG);
        turretSetpointDeg = angle_tester;
    }

    public void enableOdometryAim() {
        state = TurretState.ODOMETRY_AIM;
    }

    public void off() {
        state = TurretState.OFF;
        turretOne.setPower(0);
        turretTwo.setPower(0);
    }

    @Override
    public void periodic() {
        double currentAngle = getMeasuredAngleDeg();
        double power = 0.0;

        switch (state) {
            case OFF:
                power = 0;
                break;
            case RAW_MANUAL:
                power = manualRawPower;
                break;
            case MANUAL:
                angle_tester = Range.clip(angle_tester, MIN_LIMIT_DEG, MAX_LIMIT_DEG);
                turretSetpointDeg = angle_tester;
                power = calculatePID(turretSetpointDeg, currentAngle);
                break;
            case ODOMETRY_AIM:
                Pose robotPose = PedroComponent.follower().getPose();

                if (robotPose != null) {
                    double fieldAngleToGoal = VisionDistanceHelper.getAngleToGoalRad(robotPose);

                    // 1. Base Aim (Goal Centric)
                    double baseTargetFieldHeading = fieldAngleToGoal;

                    // 2. Add Physics Compensation from Launcher
                    // The Turret now purely trusts the Launcher's calculation.
                    double physicsOffsetDeg = LauncherOuttakeFuckingThing.calculatedTurretOffset;

                    // Convert to Radians for math
                    double finalTargetFieldHeading = baseTargetFieldHeading + Math.toRadians(physicsOffsetDeg);

                    // 3. Convert to Robot Relative Turret Angle
                    double robotHeading = robotPose.getHeading();
                    double targetRad = finalTargetFieldHeading - robotHeading;

                    double targetDeg = wrapDeg(-Math.toDegrees(targetRad));
                    turretSetpointDeg = Range.clip(targetDeg, MIN_LIMIT_DEG, MAX_LIMIT_DEG);

                    // Set values for PID
                    angle_tester = turretSetpointDeg;
                    power = calculatePID(turretSetpointDeg, currentAngle);
                }
                break;
        }

        if (state != TurretState.RAW_MANUAL) {
            if (currentAngle > MAX_LIMIT_DEG && power > 0) power = 0;
            if (currentAngle < MIN_LIMIT_DEG && power < 0) power = 0;
        }

        double finalPower = INVERT_SERVO_OUTPUT ? -power : power;
        turretOne.setPower(finalPower);
        turretTwo.setPower(finalPower);
    }

    private double calculatePID(double target, double current) {
        double error = target - current;
        double pOut = error * kP;
        double derivative = (error - lastError);
        double dOut = derivative * kD;
        lastError = error;
        return Range.clip(pOut + dOut, -MAX_POWER, MAX_POWER);
    }
}