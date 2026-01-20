package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;

@Config
public class Turret implements Subsystem {

    public static final Turret INSTANCE = new Turret();
    public Turret() {}

    public enum TurretState {
        OFF,
        MANUAL,
        ODOMETRY_AIM
    }

    private TurretState state = TurretState.OFF;

    private CRServo turretOne;
    private CRServo turretTwo;

    private DcMotorEx encoderPortContainer;

    public static String ENCODER_PORT_NAME = "leftRear";

    public static double TICKS_PER_REV_ENCODER = 4096;
    public static double TURRET_GEAR_RATIO = 112.0 / 19.0;

    // SENSOR SETTINGS
    public static boolean ENCODER_REVERSE = true;

    // LIMIT SETTINGS (Asymmetric)
    // Make sure MIN is less than MAX. Example: -45 and 135
    public static double MIN_LIMIT_DEG = -85.0;
    public static double MAX_LIMIT_DEG = 180;

    // OUTPUT SETTINGS
    // Toggle this if the turret spins the wrong way relative to the encoder
    public static boolean INVERT_SERVO_OUTPUT = true;

    // PID SETTINGS
    public static double kP = 0.02;
    public static double kI = 0.0;
    public static double kD = 0.0027;
    public static double MAX_POWER = 1.0;

    private double turretSetpointDeg = 0.0;
    private double lastError = 0.0;
    private double currentOffset = 0.0;

    public static double angle_tester = 0.0;

    @Override
    public void initialize() {
        turretOne = ActiveOpMode.hardwareMap().get(CRServo.class, "turret_one");
        turretTwo = ActiveOpMode.hardwareMap().get(CRServo.class, "turret_two");

        // Set base directions. If the servos fight each other, change ONE of these.
        // If they move together but wrong way, change INVERT_SERVO_OUTPUT.
        turretOne.setDirection(CRServo.Direction.REVERSE);
        turretTwo.setDirection(CRServo.Direction.REVERSE);

        encoderPortContainer = ActiveOpMode.hardwareMap().get(DcMotorEx.class, ENCODER_PORT_NAME);

        resetEncoderLogic();

        turretSetpointDeg = 0.0;
        state = TurretState.MANUAL;
    }


    public void resetEncoderLogic() {
        if(encoderPortContainer != null) {
            double raw = encoderPortContainer.getCurrentPosition();
            if(ENCODER_REVERSE) raw = -raw;
            currentOffset = raw;
        }
    }

    public double getMeasuredAngleDeg() {
        if (encoderPortContainer == null) return 0.0;

        double raw = encoderPortContainer.getCurrentPosition();
        if(ENCODER_REVERSE) raw = -raw;

        double relativeTicks = raw - currentOffset;
        double ticksPerDeg = (TICKS_PER_REV_ENCODER * TURRET_GEAR_RATIO) / 360.0;

        return relativeTicks / ticksPerDeg;
    }

    private double wrapDeg(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    public void setManualAngle(double angle) {
        state = TurretState.MANUAL;
        // Clip incoming manual command to new limits
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

            case MANUAL:
                // Ensure the tester variable also respects limits
                angle_tester = Range.clip(angle_tester, MIN_LIMIT_DEG, MAX_LIMIT_DEG);
                turretSetpointDeg = angle_tester;
                power = calculatePID(turretSetpointDeg, currentAngle);
                break;

            case ODOMETRY_AIM:
                Pose robotPose = PedroComponent.follower().getPose();
                if (robotPose != null) {
                    double fieldAngleToGoal = VisionDistanceHelper.getAngleToGoalRad(robotPose);
                    double robotHeading = robotPose.getHeading();

                    // Standard Math calculates this (Left = Positive)
                    double targetRad = fieldAngleToGoal - robotHeading;

                    // --- THE FIX ---
                    // Since we set ENCODER_REVERSE = true, your Turret thinks "Right is Positive".
                    // Standard Math thinks "Left is Positive".
                    // We must FLIP the math result to match the Turret.
                    double targetDeg = -Math.toDegrees(targetRad);

                    targetDeg = wrapDeg(targetDeg);

                    // Clip target to the new asymmetric limits
                    turretSetpointDeg = Range.clip(targetDeg, MIN_LIMIT_DEG, MAX_LIMIT_DEG);
                    angle_tester = turretSetpointDeg;

                    power = calculatePID(turretSetpointDeg, currentAngle);
                }
                break;
        }

        // --- SAFETY LIMIT LOGIC ---
        // If we are past the MAX limit and trying to go further positive (Power > 0), stop.
        if (currentAngle > MAX_LIMIT_DEG && power > 0) power = 0;

        // If we are past the MIN limit and trying to go further negative (Power < 0), stop.
        if (currentAngle < MIN_LIMIT_DEG && power < 0) power = 0;


        // --- OUTPUT APPLICATION ---
        // Apply the global direction inversion here
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