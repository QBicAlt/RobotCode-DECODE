package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class LauncherOuttakeFuckingThing implements Subsystem {
    public static final LauncherOuttakeFuckingThing INSTANCE = new LauncherOuttakeFuckingThing();
    private LauncherOuttakeFuckingThing() {}

    public final MotorEx fuckingMotorOne = new MotorEx("launcher_one");
    public final MotorEx fuckingMotorTwo = new MotorEx("launcher_two");

    public static double KP = 0.0008, KI = 0.0, KD = 0.0;
    public static double kS = 0.0, kV = 0.0, kA = 0.0;

    private final ControlSystem ctrl1 = ControlSystem.builder()
            .velPid(KP, KI, KD)
            //.basicFF(kS, kV, kA)
            .build();

    private final ControlSystem ctrl2 = ControlSystem.builder()
            .velPid(KP, KI, KD)
            //.basicFF(kS, kV, kA)
            .build();

    private volatile double targetRpm = 10.0;

    public void setTargetRpm(double rpm) {
        targetRpm = rpm;
        ctrl1.setGoal(new KineticState(0.0, rpm));
        ctrl2.setGoal(new KineticState(0.0, rpm));
    }

    @Override
    public void periodic() {
        double v1_rads = fuckingMotorOne.getVelocity();
        double v2_rads = fuckingMotorTwo.getVelocity();
        double v1_rpm  = radsPerSecToRpm(v1_rads);
        double v2_rpm  = radsPerSecToRpm(v2_rads);

        double pwr1 = ctrl1.calculate(new KineticState(0.0, v1_rpm));
        double pwr2 = ctrl2.calculate(new KineticState(0.0, v2_rpm));

        fuckingMotorOne.setPower(Range.clip(pwr1, -1.0, 1.0));
        fuckingMotorTwo.setPower(Range.clip(pwr2, -1.0, 1.0));
    }

    private static double radsPerSecToRpm(double radPerSec) {
        return (radPerSec * 60.0) / (2.0 * Math.PI);
    }
}
