package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;

@Config
public class Turret implements Subsystem {

    public static final Turret INSTANCE = new Turret();
    public Turret() {}

    public enum TurretState {
        OFF,
        MANUAL,    // angle setpoint controlled by driver commands / dashboard
        LIMELIGHT  // angle setpoint adjusted by Limelight outer loop
    }

    private TurretState state = TurretState.OFF;

    private CRServo turretOne;
    private CRServo turretTwo;
    public AnalogInput turretFeedback;
    private IMUEx imu;
    private Limelight3A limelight;

    public static double MIN_VOLTAGE = 3.25;
    public static double MAX_VOLTAGE = 0.05;

    public static double SERVO_RANGE_DEG   = 355.0;
    public static double TURRET_GEAR_RATIO = 0.815;

    public static double MIN_ANGLE_DEG = -145.0;
    public static double MAX_ANGLE_DEG =  145.0;

    public static double angle_tester = 0.0;

    public static double kP_angle = 0.017;
    public static double kI_angle = 0.0001;
    public static double kD_angle = 0.00057;

    public static double MAX_TURRET_POWER   = 1;
    public static double MAX_ANGLE_INTEGRAL = 50.0;

    public static double HOLD_TOL_DEG = 2;

    private double turretSetpointDeg = 0.0;
    private double angleIntegral     = 0.0;
    private double lastAngleError    = 0.0;

    private static final double DT_SEC = 0.02;

    public static double kAIM                 = -0.2;
    public static double MAX_AIM_STEP_DEG     = 3.0;
    public static double TX_FILTER_ALPHA      = 0.6;
    public static double LIMELIGHT_X_OFFSET_DEG = -3.0;
    public static double DEADZONE_DEG         = 1;

    public static double filteredTx = 0.0;

    private boolean hasGoalFieldHeading     = false;
    private double  lastGoalFieldHeadingDeg = 0.0;

    @Override
    public void initialize() {
        turretOne = ActiveOpMode.hardwareMap().get(CRServo.class, "turret_one");
        turretTwo = ActiveOpMode.hardwareMap().get(CRServo.class, "turret_two");

        turretOne.setDirection(CRServo.Direction.REVERSE);
        turretTwo.setDirection(CRServo.Direction.REVERSE);

        turretFeedback = ActiveOpMode.hardwareMap().get(AnalogInput.class, "turret");

        imu = new IMUEx("imu", Direction.RIGHT, Direction.FORWARD);
        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
        limelight.start();

        turretSetpointDeg = 0.0;
        state = TurretState.MANUAL;
    }


    public double getRobotHeadingDeg() {
        return wrapDeg(imu.get().inDeg);
    }

    private static double wrapDeg(double a) {
        while (a > 180) a -= 360;
        while (a < -180) a += 360;
        return a;
    }

    public double getMeasuredAngleDeg() {
        double v = turretFeedback.getVoltage();

        double minV = Math.min(MIN_VOLTAGE, MAX_VOLTAGE);
        double maxV = Math.max(MIN_VOLTAGE, MAX_VOLTAGE);

        if (Math.abs(maxV - minV) < 1e-4) {
            return 0.0;
        }

        double norm = (v - minV) / (maxV - minV);
        norm = Range.clip(norm, 0.0, 1.0);

        double servoDeg = (norm * SERVO_RANGE_DEG) - (SERVO_RANGE_DEG / 2.0);

        double turretDeg = servoDeg * TURRET_GEAR_RATIO;

        return Range.clip(turretDeg, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
    }



    public void off() {
        state = TurretState.OFF;
        turretOne.setPower(0.0);
        turretTwo.setPower(0.0);
    }

    public void manual() {
        state = TurretState.MANUAL;
    }

    public void enableLimelightAim() {
        state = TurretState.LIMELIGHT;
        filteredTx = 0.0;
    }

    public void disableLimelightAim() {
        state = TurretState.MANUAL;
        filteredTx = 0.0;
    }

    public void enableAutoAim(boolean enabled) {
        if (enabled) enableLimelightAim();
        else disableLimelightAim();
    }


    public void setSetpointDeg(double angleDeg) {
        turretSetpointDeg = Range.clip(angleDeg, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
    }

    public void setManualAngle(double angleDeg) {
        manual();
        setSetpointDeg(angleDeg);
    }

    public void nudgeManual(double deltaDeg) {
        if (state != TurretState.MANUAL) return;
        setSetpointDeg(turretSetpointDeg + deltaDeg);
    }


    private double anglePidStep(double dtSec) {
        double measured = getMeasuredAngleDeg();
        double error    = turretSetpointDeg - measured;
        double absError = Math.abs(error);

        if (absError < HOLD_TOL_DEG) {
            angleIntegral  = 0.0;
            lastAngleError = 0.0;
            return 0.0;
        }

        angleIntegral += error * dtSec;
        angleIntegral = Range.clip(angleIntegral, -MAX_ANGLE_INTEGRAL, MAX_ANGLE_INTEGRAL);

        double deriv = (error - lastAngleError) / dtSec;
        lastAngleError = error;

        double output = kP_angle * error + kI_angle * angleIntegral + kD_angle * deriv;
        output = Range.clip(output, -MAX_TURRET_POWER, MAX_TURRET_POWER);

        return output;
    }


    private void updateLimelightAim() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return;
        }

        double rawTx = result.getTx();
        double tx    = rawTx - LIMELIGHT_X_OFFSET_DEG;

        filteredTx = TX_FILTER_ALPHA * filteredTx
                + (1.0 - TX_FILTER_ALPHA) * tx;

        if (Math.abs(filteredTx) < DEADZONE_DEG) {
            double robotHeading  = getRobotHeadingDeg();
            double measuredAngle = getMeasuredAngleDeg();
            lastGoalFieldHeadingDeg = wrapDeg(robotHeading - measuredAngle);
            hasGoalFieldHeading = true;
            return;
        }

        double aimStep = kAIM * filteredTx;
        aimStep = Range.clip(aimStep, -MAX_AIM_STEP_DEG, MAX_AIM_STEP_DEG);

        setSetpointDeg(turretSetpointDeg + aimStep);
    }

    public void snapToRememberedGoalAndEnable() {
        enableLimelightAim();

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            return;
        }

        if (!hasGoalFieldHeading) {
            return;
        }

        double currentHeading   = getRobotHeadingDeg();
        double desiredTurretDeg = wrapDeg(currentHeading - lastGoalFieldHeadingDeg);

        setSetpointDeg(desiredTurretDeg);
    }


    @Override
    public void periodic() {
        switch (state) {
            case OFF:
                turretOne.setPower(0.0);
                turretTwo.setPower(0.0);
                return;

            case MANUAL:
                setSetpointDeg(angle_tester);
                break;

            case LIMELIGHT:
                updateLimelightAim();
                break;
        }

        if (state != TurretState.OFF) {
            double power = anglePidStep(DT_SEC);



            turretOne.setPower(power);
            turretTwo.setPower(power);
        }
    }
}
