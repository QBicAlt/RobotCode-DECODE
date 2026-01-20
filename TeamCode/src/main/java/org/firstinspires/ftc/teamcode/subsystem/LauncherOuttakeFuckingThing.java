package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.util.Data;

@Config
public class LauncherOuttakeFuckingThing implements Subsystem {
    public static final LauncherOuttakeFuckingThing INSTANCE = new LauncherOuttakeFuckingThing();
    private LauncherOuttakeFuckingThing() {}

    // --- Hardware ---
    private MotorEx motorOne;
    private MotorEx motorTwo;
    private ServoEx turretLatch;
    private ServoEx angleServo;
    private DcMotorEx rawMotorOne;

    // Standard I2C Sensor
    private RevColorSensorV3 colorSensor;

    // --- Control Flags ---
    public static boolean autoCalculate = true;

    // --- Constants ---
    public static double TICKS_PER_REV = 33.83;
    private static final double NEUTRAL_HOOD_ANGLE_DEG = 25.267884;
    private static final double SERVO_RANGE_DEG        = 355.0;
    private static final double SERVO_PER_HOOD_DEG = (168.0 / 19.0) * (20.0 / 40.0);

    // Turret latch positions
    public static double turret_Closed  = .85;
    public static double turret_Open    = 0.56;

    // Default/Fallback
    public static double shooterAngle = 25.267884 ;
    public static double targetRpm = 0.0;

    // Preset RPMs
    public static double SLOW_RPM   = 3000;
    public static double LAUNCH_RPM = 3600;

    public static PIDCoefficients COEFFS = new PIDCoefficients(0.0045, 0.0, 0.0);
    private ControlSystem velocityPID;

    public static double kS = 0.0175;
    public static double kV = 0.0001815;
    public static double kA = 0.0;
    private double lastTargetRpm = 0.0;

    // Fallback Manual
    public static double MANUAL_RPM = 2700;
    public static double MANUAL_ANGLE = 32.0;

    // DISTANCE THRESHOLD (Adjust this if needed)
    // 4.0 CM is usually a good "Sweet spot" for a ball in a cup
    public static double BALL_DETECT_CM = 4.0;

    public double getTargetRpm() { return targetRpm; }

    @Override
    public void initialize() {
        motorOne = new MotorEx("launcher_one");
        motorTwo = new MotorEx("launcher_two");

        // STANDARD REV V3 INIT
        // This MUST be plugged into an I2C port (0-3) on the hub.
        colorSensor = ActiveOpMode.hardwareMap().get(RevColorSensorV3.class, "color");

        turretLatch = new ServoEx("latch_servo");
        angleServo  = new ServoEx("angle_servo");
        rawMotorOne = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "launcher_one");
        velocityPID = ControlSystem.builder().velPid(COEFFS).build();
     //   setTargetRpm(0.0);
        setAngle(shooterAngle);
        autoCalculate = false;
    }

    public double getMotorCurrent() {
        if (rawMotorOne != null) {
            return rawMotorOne.getCurrent(CurrentUnit.AMPS);
        }
        return 0.0;
    }

    public void setManualShooter(double rpm, double angleDeg) {
        autoCalculate = false;
        setTargetRpm(rpm);
        setAngle(angleDeg);
    }

    public void enableAutoCalculation() {
        autoCalculate = true;
    }

    // --- STANDARD I2C DETECTION LOGIC ---
    public boolean hasBall() {
        if (colorSensor == null) return false;

        // Check if object is closer than 4 CM
        double dist = colorSensor.getDistance(DistanceUnit.CM);

        // Safety check: NaN is sometimes returned if nothing is seen at all
        if (Double.isNaN(dist)) return false;

        return dist < BALL_DETECT_CM;
    }

    public void setAngle(double angleDeg) {
        shooterAngle = angleDeg;
        double hoodOffsetDeg = shooterAngle - NEUTRAL_HOOD_ANGLE_DEG;
        double servoDeg = hoodOffsetDeg * SERVO_PER_HOOD_DEG;
        double servoPos = servoDeg / SERVO_RANGE_DEG;
        servoPos = Range.clip(servoPos, 0.0, 1.0);
        angleServo.setPosition(servoPos);
    }

    public void setTurretLatch(double pos){
        turretLatch.setPosition(pos);
    }

    public void setTargetRpm(double rpm) {
        targetRpm = rpm;
        velocityPID.setGoal(new KineticState(0.0, targetRpm));
    }

    private double ticksPerSecToRpm(double ticksPerSec) {
        return (ticksPerSec / TICKS_PER_REV) * 60.0;
    }

    public double getCurrentRpm(){
        return ticksPerSecToRpm(motorOne.getVelocity());
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    @Override
    public void periodic() {
        // --- AUTO CALCULATION VIA PEDRO POSE ---
        if (autoCalculate) {
            Pose currentPose = PedroComponent.follower().getPose();
            double distIn = VisionDistanceHelper.distanceToGoalInches(currentPose);

            if (distIn > 0) {
                double firstDist = Data.LAUNCHER_POSES[0][0];
                double lastDist  = Data.LAUNCHER_POSES[Data.LAUNCHER_POSES.length - 1][0];
                distIn = Range.clip(distIn, firstDist, lastDist);

                int i1 = 1;
                while (i1 < Data.LAUNCHER_POSES.length && Data.LAUNCHER_POSES[i1][0] < distIn) {
                    i1++;
                }
                int i0 = i1 - 1;
                double d0 = Data.LAUNCHER_POSES[i0][0];
                double d1 = Data.LAUNCHER_POSES[i1][0];
                double t = (Math.abs(d1 - d0) < 1e-6) ? 0.0 : (distIn - d0) / (d1 - d0);
                double speed = lerp(Data.LAUNCHER_POSES[i0][1], Data.LAUNCHER_POSES[i1][1], t);
                double angle = lerp(Data.LAUNCHER_POSES[i0][2], Data.LAUNCHER_POSES[i1][2], t);

                setTargetRpm(speed);
                setAngle(angle);
            }
        }

        double measRpm = getCurrentRpm();
        double pidOut = velocityPID.calculate(new KineticState(0.0, measRpm));
        double targetAccelRpmPerSec = (targetRpm - lastTargetRpm) / 0.02;
        lastTargetRpm = targetRpm;

        double ff = (Math.abs(targetRpm) > 1e-3 ? kS * Math.signum(targetRpm) : 0.0)
                + kV * targetRpm
                + kA * targetAccelRpmPerSec;

        double power = Range.clip(pidOut + ff, -1.0, 1.0);
        motorOne.setPower(power);
        motorTwo.setPower(power);
    }
}