package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.util.Range;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class LauncherOuttakeFuckingThing implements Subsystem {
    public static final LauncherOuttakeFuckingThing INSTANCE = new LauncherOuttakeFuckingThing();
    private LauncherOuttakeFuckingThing() {}

    public final MotorEx fuckingMotorOne = new MotorEx("launcher_one");
    public final MotorEx fuckingMotorTwo = new MotorEx("launcher_two");

    public final Command slow = new LambdaCommand().setStart(() -> setTargetRpm(2000));
    public final Command launch = new LambdaCommand().setStart(() -> setTargetRpm(4800));

    public static double KP = 0.0008, KI = 0.0, KD = 0.0;
    public static double kS = 0.0, kV = 0.0, kA = 0.0;

    private final ControlSystem velocityPID = ControlSystem.builder()
            .velPid(KP, KI, KD)
            //.basicFF(kS, kV, kA)
            .build();

    private double targetRpm = 10.0;

    public void setTargetRpm(double rpm) {
        targetRpm = rpm;
        velocityPID.setGoal(new KineticState(0.0, rpm));
    }

    @Override
    public void periodic() {
        double velocity = fuckingMotorOne.getVelocity();
        double rpm  = getRpm(velocity);

        double power = velocityPID.calculate(new KineticState(0.0, rpm));

        fuckingMotorOne.setPower(Range.clip(power, -1.0, 1.0));
        fuckingMotorTwo.setPower(Range.clip(power, -1.0, 1.0));
    }

    private static double getRpm(double radPerSec) {
        return (radPerSec * 60.0) / (2.0 * Math.PI);
    }
}
