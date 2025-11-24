package org.firstinspires.ftc.teamcode.subsystem;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.Range;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

import com.acmerobotics.dashboard.config.Config;

@Config
public class LauncherOuttakeFuckingThing implements Subsystem {
    public static final LauncherOuttakeFuckingThing INSTANCE = new LauncherOuttakeFuckingThing();
    private LauncherOuttakeFuckingThing() {}

    // --- Hardware ---
    private MotorEx motorOne;
    private MotorEx motorTwo;
    private ServoEx turretLatch;
    private ServoEx angleServo;

    // --- Encoder info ---
    // You said: 28 ticks per revolution.
    public static double TICKS_PER_REV = 28.0;

    // --- Geometry constants (same as you had) ---
    private static final double NEUTRAL_HOOD_ANGLE_DEG = 15.0;   // hood angle = 15° at servo pos 0
    private static final double SERVO_RANGE_DEG        = 355.0;  // servo spec
    private static final double SERVO_PER_HOOD_DEG     = 380.0 / 38.0; // 10.0 servoDeg per hoodDeg

    // Dashboard-tunable hood target (degrees)
    public static double shooterAngle = 15.0;   // 15–40 valid

    // (These conversions are now only for debugging/telemetry if you ever want them)
    private static double rpmToRadPerSec(double rpm) {
        return rpm * (2.0 * Math.PI) / 60.0;
    }

    private static double radPerSecToRpm(double rad) {
        return rad * 60.0 / (2.0 * Math.PI);
    }

    // Turret latch positions
    public static double turret_Closed  = 0.6;
    public static double turret_Open    = 0.07;

    // Preset RPMs
    public static double SLOW_RPM   = 3000;
    public static double LAUNCH_RPM = 3600;

    public static PIDCoefficients COEFFS = new PIDCoefficients(
            0.0017,    // kP  (much smaller!)
            0.000000,  // kI  (tiny)
            0.0        // kD
    );


    // --- FEEDFORWARD GAINS (RPM-based) ---
    // Units:
    //   kS: power (0..1)
    //   kV: power per RPM
    //   kA: power per (RPM/s)
    public static double kS = 0.0175;
    public static double kV = 0.0001815;  // starting guess; tune this in Dashboard
    public static double kA = 0.0;      // start at 0; add later if needed

    private ControlSystem velocityPID;

    // We now track target in RPM (not rad/s).
    private double targetRpm = 0.0;

    // For optional acceleration feedforward
    private static final double LOOP_DT = 0.02; // 20 ms loop; adjust if your loop timing is different
    private double lastTargetRpm = 0.0;

    public double getTargetRpm() {
        return targetRpm;
    }

    @Override
    public void initialize() {
        motorOne = new MotorEx("launcher_one");
        motorTwo = new MotorEx("launcher_two");

        turretLatch = new ServoEx("latch_servo");
        angleServo  = new ServoEx("angle_servo");

        // Build PID ONCE with COEFFS. Dashboard mutates COEFFS in-place -> live tuning.
        velocityPID = ControlSystem.builder()
                .velPid(    COEFFS)
                .build();

        setTargetRpm(0.0);

        // Start hood at neutral/low angle
        setTurretAngle(shooterAngle);
    }

    /**
     * Set hood angle in DEGREES.
     * Keeps exactly your previous behavior, just cleaned a bit.
     */
    public void setTurretAngle(double angleDeg) {
        shooterAngle = angleDeg;

        // How far from neutral (in hood degrees)?
        double hoodOffsetDeg = shooterAngle - NEUTRAL_HOOD_ANGLE_DEG;

        // Convert hood degrees -> servo degrees using gear ratio (10:1)
        double servoDeg = hoodOffsetDeg * SERVO_PER_HOOD_DEG;

        // Convert servo degrees -> servo position [0..1]
        // assuming servoPos = 0.0 at neutral hood (15°)
        double servoPos = servoDeg / SERVO_RANGE_DEG;

        // Keep it in bounds just in case you send something dumb
        servoPos = Range.clip(servoPos, 0.0, 1.0);

        angleServo.setPosition(servoPos);
    }

    public void setTurretLatch(double pos){
        turretLatch.setPosition(pos);
    }

    /**
     * Set target in RPM (what you wanted to work in).
     */
    public void setTargetRpm(double rpm) {
        targetRpm = rpm;

        // PID "goal velocity" is in RPM now; ControlSystem doesn't care about units
        velocityPID.setGoal(new KineticState(
                0.0,      // position (unused)
                targetRpm // velocity (RPM)
        ));
    }

    // Helper: ticks/sec -> RPM
    private double ticksPerSecToRpm(double ticksPerSec) {
        return (ticksPerSec / TICKS_PER_REV) * 60.0;
    }

    /**
     * Current RPM of the launcher.
     * This replaces the old weird rad/sec math and actually returns RPM.
     */
    public double getCurrentRpm(){
        // Keep your sign convention (you had a minus before).
        double ticksPerSec = motorOne.getVelocity();  // assuming getVelocity() is ticks/sec
        return ticksPerSecToRpm(ticksPerSec);
    }

    @Override
    public void periodic() {
        Pose pos = PedroComponent.follower().getPose();
        // You'll use pos later for shooter table stuff; leaving it alone for now.

        // Keep hood servo tracking shooterAngle from Dashboard (same behavior as before)
        setTurretAngle(shooterAngle);

        // --- MEASURE RPM ---
        double ticksPerSec = motorOne.getVelocity();  // keep your sign convention
        double measRpm     = ticksPerSecToRpm(ticksPerSec);

        // --- PID IN RPM SPACE ---
        double pidOut = velocityPID.calculate(new KineticState(
                0.0,
                measRpm
        ));

        // --- TARGET-SIDE ACCEL (for kA if you want it) ---
        double targetAccelRpmPerSec = (targetRpm - lastTargetRpm) / LOOP_DT;
        lastTargetRpm = targetRpm;

        // --- FEEDFORWARD (RPM-BASED) ---
        double ff = 0.0;

        if (Math.abs(targetRpm) > 1e-3) {
            ff += kS * Math.signum(targetRpm);
        }

        ff += kV * targetRpm;
        ff += kA * targetAccelRpmPerSec; // does nothing while kA = 0

        double power = Range.clip(pidOut + ff, -1.0, 1.0);

        motorOne.setPower(power);
        motorTwo.setPower(power);
    }
}
