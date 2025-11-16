package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.util.Range;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
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

    private static double rpmToRadPerSec(double rpm) {
        return rpm * (2.0 * Math.PI) / 60.0;
    }

    private static double radPerSecToRpm(double rad) {
        return rad * 60.0 / (2.0 * Math.PI);
    }

    public static double SLOW_RPM   = 2000.0;
    public static double LAUNCH_RPM = 6000.0;

    public static PIDCoefficients COEFFS = new PIDCoefficients(
            0.0008,  // kP (tune)
            0.0000,  // kI
            0.0000   // kD
    );

    public static double kS = 0.0;
    public static double kV = 0.000027;
    public static double kA = 0.0;

    private ControlSystem velocityPID;

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
        double velRadPerSec = (-motorOne.getVelocity() / 25) * 60;
        return velRadPerSec;
    }

    @Override
    public void periodic() {
        double measRad = motorOne.getVelocity();
        double pidOut = velocityPID.calculate(new KineticState(0.0, measRad));
        double ff = /*kS * Math.signum(targetRadPerSec) +*/ kV * targetRadPerSec;

        double power = Range.clip(pidOut + ff, -1.0, 1.0);

        motorOne.setPower(power);
        motorTwo.setPower(power);
    }
}