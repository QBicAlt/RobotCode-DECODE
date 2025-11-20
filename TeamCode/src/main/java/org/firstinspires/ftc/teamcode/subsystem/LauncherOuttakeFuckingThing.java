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
    public static final LauncherOuttakeFuckingThing INSTANCE = new  LauncherOuttakeFuckingThing();
    private LauncherOuttakeFuckingThing() {}

    private MotorEx motorOne;
    private MotorEx motorTwo;
    private ServoEx turretLatch;

    private static double rpmToRadPerSec(double rpm) {
        return rpm * (2.0 * Math.PI) / 60.0;
    }

    private static double radPerSecToRpm(double rad) {
        return rad * 60.0 / (2.0 * Math.PI);
    }

    public static double turret_Closed  = .6;
    public static double turret_Open = .07;


    public static double SLOW_RPM   = 3000;
    public static double LAUNCH_RPM = 3600;

    public static PIDCoefficients COEFFS = new PIDCoefficients(
            0.0008,  // kP (tune)
            0.0000,  // kI
            0.0000   // kD
    );

    public static double kS = 0.0;
    public static double kV = 0.0045;
    public static double kA = 0.01;
    private ControlSystem velocityPID;

    private double targetRadPerSec = 0.0;

    public double getTargetRpm() {
        return radPerSecToRpm(targetRadPerSec);
    }

    @Override
    public void initialize() {
        motorOne = new MotorEx("launcher_one");
        motorTwo = new MotorEx("launcher_two");

        turretLatch = new ServoEx("latch_servo");

        velocityPID = ControlSystem.builder()
                .velPid(COEFFS)
                .build();

        setTargetRpm(0.0);
    }

    public void setTurretLatch(double pos){
        turretLatch.setPosition(pos);
    }


    public void setTargetRpm(double rpm) {
        targetRadPerSec = rpmToRadPerSec(rpm);
        velocityPID.setGoal(new KineticState(0.0, targetRadPerSec));
    }


    public double getCurrentRpm(){
        double velRadPerSec = (-motorOne.getVelocity() / 28) * 60;
        return velRadPerSec;
    }

    @Override
    public void periodic() {
        Pose pos = PedroComponent.follower().getPose();
  //      double x = pos.getX() * 2;
//double y = pos.getY() * 2;
//double ceilY = Math.ceil(y);
//double ceilX = Math.ceil(x);
//double fx = x - Math.floor(x);
      //  double fy = y - Math.floor(y);

    //    double[] dataOne = Data.LAUNCHER_POSES[(int) y][(int) x];
     //   double[] dataTwo = Data.LAUNCHER_POSES[(int) ceilY][(int) ceilX];
       // double[] dataThree = Data.LAUNCHER_POSES[(int) y][(int) ceilX];
        //double[] dataFour = Data.LAUNCHER_POSES[(int) ceilY][(int) x];

       // double rpm = (1 - fy) * ((1 - fx) * dataOne[0] + fx * dataThree[0]) + fy * ((1 - fx) * dataTwo[0] + fx * dataFour[0]);
  //      double angle = (1 - fy) * ((1 - fx) * dataOne[1] + fx * dataThree[1]) + fy * ((1 - fx) * dataTwo[1] + fx * dataFour[1]);

    //    setTargetRpm(rpm);

        double measRad = motorOne.getVelocity();
        double pidOut = velocityPID.calculate(new KineticState(0.0, measRad));
        double ff = /*kS * Math.signum(targetRadPerSec) +*/ kV * targetRadPerSec;

        double power = Range.clip(pidOut + ff, -1.0, 1.0);

        motorOne.setPower(power);
        motorTwo.setPower(power);
    }
}