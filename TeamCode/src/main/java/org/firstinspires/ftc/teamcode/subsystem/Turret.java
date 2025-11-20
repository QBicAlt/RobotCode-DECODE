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

    // --- Singleton ---
    public static final Turret INSTANCE = new Turret();
    public Turret() {}

    // --- States ---
    public enum TurretState {
        OFF,
        MANUAL,        // angle setpoint from dashboard/driver
        LIMELIGHT,     // pure LL PID on tx -> power
        SNAP_TO_GOAL   // short angle PID to remembered goal angle
    }

    private TurretState state = TurretState.OFF;

    // --- Hardware ---
    private CRServo turretOne;
    private CRServo turretTwo;
    public AnalogInput turretFeedback;
    private IMUEx imu;
    private Limelight3A limelight;

    // --- Analog mapping / geometry ---
    // Tune MIN/MAX_VOLTAGE in Dashboard so measured angle roughly matches reality.
    public static double MIN_VOLTAGE = 3.25;
    public static double MAX_VOLTAGE = 0.05;

    public static double SERVO_RANGE_DEG   = 355.0;   // analog span
    public static double TURRET_GEAR_RATIO = 0.815;   // turretDeg = servoDeg * ratio

    public static double MIN_ANGLE_DEG = -145.0;      // turret physical/safe range
    public static double MAX_ANGLE_DEG =  145.0;

    // Dashboard tester – for MANUAL mode
    public static double angle_tester = 0.0;

    // --- Inner angle PID (used only in MANUAL + SNAP_TO_GOAL) ---
    public static double kP_angle = 0.017;
    public static double kI_angle = 0.0;        // start at 0, add later if needed
    public static double kD_angle = 0.00057;

    public static double MAX_TURRET_POWER   = 1.0;
    public static double MAX_ANGLE_INTEGRAL = 50.0;

    public static double HOLD_TOL_DEG = 2.0;   // inner loop "close enough"
    public static double SNAP_TOL_DEG = 3.0;   // when snapping to remembered angle

    private double turretSetpointDeg = 0.0;
    private double angleIntegral     = 0.0;
    private double lastAngleError    = 0.0;

    // --- Limelight PID around tx (pure LL control in LIMELIGHT state) ---
    public static double kLL_P = -0.04;   // sign based on turret direction
    public static double kLL_I = 0.0;
    public static double kLL_D = 0.0;

    public static double MAX_LL_POWER = 1.0; // clamp turret power in LL mode

    private double llIntegral  = 0.0;
    private double llLastError = 0.0;
    private double limelightPower = 0.0;

    private static final double DT_SEC = 0.02; // 20 ms loop

    // --- Angle filtering (simple EMA) ---
    public static double FILTER_ALPHA = 0.6;  // 0..1, higher = smoother

    private double filteredAngleDeg   = 0.0;
    private boolean filterInitialized = false;

    // --- Limelight filtering / alignment ---
    public static double TX_FILTER_ALPHA      = 0.6;
    public static double LIMELIGHT_X_OFFSET_DEG = -3.0;
    public static double DEADZONE_DEG         = 1.0;

    public static double filteredTx = 0.0;

    // --- Remembered goal heading (field coordinates) ---
    private boolean hasGoalFieldHeading     = false;
    private double  lastGoalFieldHeadingDeg = 0.0;

    // --- Lifecycle ---

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

        filterInitialized = false;
        llIntegral = 0.0;
        llLastError = 0.0;

        // Start wherever we are
        double startAngle = getMeasuredAngleDeg();
        turretSetpointDeg = startAngle;
        angle_tester      = startAngle;
        state             = TurretState.MANUAL;
    }

    // --- Helpers ---

    private static double wrapDeg(double a) {
        while (a > 180) a -= 360;
        while (a < -180) a += 360;
        return a;
    }

    public double getRobotHeadingDeg() {
        return wrapDeg(imu.get().inDeg);
    }

    public LLResult runLimelight() {
        return limelight.getLatestResult();
    }

    // Raw analog angle (turret degrees, clipped)
    public double getMeasuredAngleDeg() {
        double v = turretFeedback.getVoltage();

        double minV = Math.min(MIN_VOLTAGE, MAX_VOLTAGE);
        double maxV = Math.max(MIN_VOLTAGE, MAX_VOLTAGE);

        if (Math.abs(maxV - minV) < 1e-4) {
            return 0.0;
        }

        double norm = (v - minV) / (maxV - minV); // 0..1
        norm = Range.clip(norm, 0.0, 1.0);

        double servoDeg  = norm * SERVO_RANGE_DEG - (SERVO_RANGE_DEG / 2.0); // -range/2..+range/2
        double turretDeg = servoDeg * TURRET_GEAR_RATIO;

        return Range.clip(turretDeg, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
    }

    // Simple EMA filter of angle for smoother PID / limits
    private double getFilteredAngleDeg() {
        double raw = getMeasuredAngleDeg();

        if (!filterInitialized) {
            filteredAngleDeg   = raw;
            filterInitialized  = true;
            return filteredAngleDeg;
        }

        filteredAngleDeg = FILTER_ALPHA * filteredAngleDeg
                + (1.0 - FILTER_ALPHA) * raw;

        return filteredAngleDeg;
    }

    // --- State control API ---

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
        llIntegral = 0.0;
        llLastError = 0.0;
    }

    public void disableLimelightAim() {
        state = TurretState.MANUAL;
        filteredTx = 0.0;
        limelightPower = 0.0;
    }

    public void enableAutoAim(boolean enabled) {
        if (enabled) enableLimelightAim();
        else disableLimelightAim();
    }

    // For telemetry if you want
    public String turretStateString() {
        return state.toString();
    }

    // --- Setpoint helpers for angle PID (MANUAL + SNAP) ---

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

    // --- Inner angle PID (only used in MANUAL + SNAP_TO_GOAL) ---

    private double anglePidStep(double dtSec, double measuredAngle) {
        double error    = turretSetpointDeg - measuredAngle;
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

    // --- Limelight outer loop (pure tx PID -> power in LIMELIGHT state) ---

    private void updateLimelightAim(double dtSec) {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            // No tag: don't drive turret in LL mode
            limelightPower = 0.0;
            return;
        }

        double rawTx = result.getTx();
        double tx    = rawTx - LIMELIGHT_X_OFFSET_DEG;

        // Filter tx
        filteredTx = TX_FILTER_ALPHA * filteredTx
                + (1.0 - TX_FILTER_ALPHA) * tx;

        // Deadzone: close enough to center
        if (Math.abs(filteredTx) < DEADZONE_DEG) {
            limelightPower = 0.0;

            // Update remembered goal heading (field-relative)
            double robotHeading  = getRobotHeadingDeg();
            double measuredAngle = getMeasuredAngleDeg(); // noisy is fine here
            lastGoalFieldHeadingDeg = wrapDeg(robotHeading - measuredAngle);
            hasGoalFieldHeading = true;
            return;
        }

        // PID on tx -> turret power
        double error = filteredTx; // positive if tag right of center
        llIntegral += error * dtSec;
        double deriv = (error - llLastError) / dtSec;
        llLastError  = error;

        double out = kLL_P * error + kLL_I * llIntegral + kLL_D * deriv;
        limelightPower = Range.clip(out, -MAX_LL_POWER, MAX_LL_POWER);
    }

    // --- Snap-to-remembered-goal logic ---

    public void snapToRememberedGoalAndEnable() {
        // If we currently see a tag, just go straight into LL tracking
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            enableLimelightAim();
            return;
        }

        // No tag – fall back to remembered field heading
        if (!hasGoalFieldHeading) {
            // nothing remembered yet
            return;
        }

        double currentHeading   = getRobotHeadingDeg();
        double desiredTurretDeg = wrapDeg(currentHeading - lastGoalFieldHeadingDeg);

        setSetpointDeg(desiredTurretDeg);

        // Enter SNAP_TO_GOAL state (short inner PID move)
        state = TurretState.SNAP_TO_GOAL;

        angleIntegral  = 0.0;
        lastAngleError = 0.0;
    }

    // --- Main periodic ---

    @Override
    public void periodic() {
        double angleNow = getFilteredAngleDeg(); // filtered turret angle
        double power = 0.0;

        switch (state) {
            case OFF:
                turretOne.setPower(0.0);
                turretTwo.setPower(0.0);
                return;

            case MANUAL:
                // Dashboard: angle_tester
                setSetpointDeg(angle_tester);
                power = anglePidStep(DT_SEC, angleNow);
                break;

            case LIMELIGHT:
                updateLimelightAim(DT_SEC);
                power = limelightPower;
                break;

            case SNAP_TO_GOAL:
                power = anglePidStep(DT_SEC, angleNow);

                // Once we're close enough, stop and go back to MANUAL
                if (Math.abs(turretSetpointDeg - angleNow) < SNAP_TOL_DEG) {
                    power = 0.0;
                    state = TurretState.MANUAL;
                }
                break;
        }

        if (state != TurretState.OFF) {
            // Software endstops using turret angle
            double softLimit = 140.0; // inside ±145 analog span

            if (angleNow >= softLimit && power > 0) {
                power = 0.0;
            }
            if (angleNow <= -softLimit && power < 0) {
                power = 0.0;
            }

            turretOne.setPower(power);
            turretTwo.setPower(power);
        }
    }
}
