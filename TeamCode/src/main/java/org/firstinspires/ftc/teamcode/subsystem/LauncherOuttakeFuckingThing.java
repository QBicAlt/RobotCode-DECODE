package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.util.Range;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import com.acmerobotics.dashboard.config.Config;




@Config
public class LauncherOuttakeFuckingThing implements Subsystem {
    public static final LauncherOuttakeFuckingThing INSTANCE = new LauncherOuttakeFuckingThing();
    private LauncherOuttakeFuckingThing() {}

    // --- Hardware ---
    private MotorEx motorOne;
    private MotorEx motorTwo;

    private static double rpmToRadPerSec(double rpm) { return rpm * (2.0 * Math.PI) / 60.0; }
    private static double radPerSecToRpm(double rad) { return rad * 60.0 / (2.0 * Math.PI); }

    // --- Dashboard-tunable targets (still in RPM for convenience) ---
    public static double SLOW_RPM   = 2000.0;
    public static double LAUNCH_RPM = 6000.0;

    // --- Velocity PID (loop runs in rad/s) ---
    public static PIDCoefficients COEFFS = new PIDCoefficients(
            0.0008,  // kP (tune)
            0.0000,  // kI
            0.0000   // kD
    );

    // Feedforward tuned in rad/s (NOT rpm)
    public static double kS = 0.0;
    public static double kV_rad = 0.000027; // ~ kV_rpm * (2*pi/60). Start small; tune on-bot.
    public static double kA_rad = 0.0;

    private ControlSystem velocityPID;

    // store targets in rad/s internally
    private double targetRadPerSec = 0.0;

    public double getTargetRpm() {
        return radPerSecToRpm(targetRadPerSec); }

    @Override
    public void initialize() {
        motorOne = new MotorEx("launcher_one");
        motorTwo = new MotorEx("launcher_two");

        velocityPID = ControlSystem.builder()
                .velPid(COEFFS)
                .build();

        setTargetRpm(0.0);
    }

    public void setTargetRpm(double rpm) {
        targetRadPerSec = rpmToRadPerSec(rpm);
        velocityPID.setGoal(new KineticState(0.0, targetRadPerSec));
    }

    public double getCurrentRpm(){
        double velRadPerSec = (-motorOne.getVelocity() / 25) * 60; return velRadPerSec; }

    @Override
    public void periodic() {
        // Measured velocity from MotorEx is rad/s
        double measRad = motorOne.getVelocity();

        // PID (in rad/s)
        double pidOut = velocityPID.calculate(new KineticState(0.0, measRad));

        // Simple FF (in rad/s units)
        double ff = /*kS * Math.signum(targetRadPerSec) +*/ kV_rad * targetRadPerSec;

        double power = Range.clip(pidOut + ff, -1.0, 1.0);

        motorOne.setPower(power);
        motorTwo.setPower(power);

        // (Optional) Telemetry to dashboard/logcat
        // show both units so you can sanity-check
        // targetRPM ~6000, measRPM should climb smoothly if gearing/voltage allow
        // telemetry.addData("tgtRPM", getTargetRpm());
        // telemetry.addData("measRPM", radPerSecToRpm(measRad));
        // telemetry.addData("pidOut", pidOut);
        // telemetry.addData("ff", ff);
        // telemetry.addData("power", power);
        // telemetry.update();
    }}