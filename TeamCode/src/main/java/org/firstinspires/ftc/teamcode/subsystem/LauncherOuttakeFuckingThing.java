package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
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
import com.pedropathing.math.Vector;
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

    // --- Sensors ---
    private RevColorSensorV3 colorSensor;
    private RevColorSensorV3 colorSensor2;

    // --- Control Flags ---
    public static boolean autoCalculate = true;
    public static boolean useVelocityCompensation = true;

    // --- Constants ---
    public static double TICKS_PER_REV = 33.83;
    private static final double NEUTRAL_HOOD_ANGLE_DEG = 25.267884;
    private static final double SERVO_RANGE_DEG        = 355.0;
    private static final double SERVO_PER_HOOD_DEG = (168.0 / 19.0) * (20.0 / 40.0);

    // --- NEW HARDWARE LIMITS ---
    public static double MIN_HOOD_ANGLE = 25.0;
    public static double MAX_HOOD_ANGLE = 59.5;

    // --- Physics Constants ---
    public static double FLYWHEEL_RADIUS_IN = 1.417;
    public static double GRAVITY_IN_SEC2 = 386.1;

    // Shared result for Turret to read
    public static double calculatedTurretOffset = 0.0;

    // Turret latch positions
    public static double turret_Closed  = .7;
    public static double turret_Open    = 0.87;

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

    public static double FLYWHEEL_EFFICIENCY = 0.43;
    public static double TURRET_COMPENSATION_GAIN = .8;

    // --- DISTANCE CUTOFF ---
    public static double MAX_COMPENSATION_DIST_IN = 107.0; // 10 Feet

    public static double MANUAL_RPM = 2700;
    public static double MANUAL_ANGLE = 32.0;
    public static double BALL_DETECT_CM = 4.0;

    public double getTargetRpm() { return targetRpm; }

    @Override
    public void initialize() {
        motorOne = new MotorEx("launcher_one");
        motorTwo = new MotorEx("launcher_two");
        colorSensor = ActiveOpMode.hardwareMap().get(RevColorSensorV3.class, "color");
        try {
            colorSensor2 = ActiveOpMode.hardwareMap().get(RevColorSensorV3.class, "color2");
        } catch (Exception e) {
            colorSensor2 = null;
        }
        turretLatch = new ServoEx("latch_servo");
        angleServo  = new ServoEx("angle_servo");
        rawMotorOne = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "launcher_one");
        velocityPID = ControlSystem.builder().velPid(COEFFS).build();

        setAngle(shooterAngle);
        autoCalculate = false;
    }

    public double getMotorCurrent() {
        if (rawMotorOne != null) {
            return rawMotorOne.getCurrent(CurrentUnit.AMPS);
        }
        return 0.0;
    }

    // --- TOGGLE METHOD FOR TELEOP ---
    public void toggleCompensation() {
        useVelocityCompensation = !useVelocityCompensation;
    }

    public boolean hasBall() {
        boolean sensor1HasBall = checkSensor(colorSensor, BALL_DETECT_CM);
        boolean sensor2HasBall = checkSensor(colorSensor2, 8);
        return sensor1HasBall || sensor2HasBall;
    }

    private boolean checkSensor(DistanceSensor sensor, double limitCm) {
        if (sensor == null) return false;
        double dist = sensor.getDistance(DistanceUnit.CM);
        if (Double.isNaN(dist)) return false;
        return dist < limitCm;
    }

    public void setManualShooter(double rpm, double angleDeg) {
        autoCalculate = false;
        setTargetRpm(rpm);
        setAngle(angleDeg);
        calculatedTurretOffset = 0.0;
    }

    public void enableAutoCalculation() {
        autoCalculate = true;
    }
    public void disableCompensation() {
        useVelocityCompensation = false;
    }

    public void setAngle(double angleDeg) {
        shooterAngle = Range.clip(angleDeg, MIN_HOOD_ANGLE, MAX_HOOD_ANGLE);
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
        return ticksPerSecToRpm(motorTwo.getVelocity());
    }

    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    private double rpmToInchesPerSec(double rpm) {
        return rpm * (2 * Math.PI / 60.0) * FLYWHEEL_RADIUS_IN * FLYWHEEL_EFFICIENCY;
    }

    private double inchesPerSecToRpm(double velocity) {
        if (velocity == 0) return 0;
        return velocity / (FLYWHEEL_EFFICIENCY * FLYWHEEL_RADIUS_IN * (2 * Math.PI / 60.0));
    }
    @Override
    public void periodic() {
        if (autoCalculate) {
            Pose currentPose = getPose();
            double distIn = VisionDistanceHelper.distanceToGoalInches(currentPose);

            if (distIn > 0) {
                // 1. LOOKUP TABLE
                double firstDist = Data.LAUNCHER_POSES[0][0];
                double lastDist  = Data.LAUNCHER_POSES[Data.LAUNCHER_POSES.length - 1][0];
                double clampedDist = Range.clip(distIn, firstDist, lastDist);

                int i1 = 1;
                while (i1 < Data.LAUNCHER_POSES.length && Data.LAUNCHER_POSES[i1][0] < clampedDist) {
                    i1++;
                }
                int i0 = i1 - 1;
                double d0 = Data.LAUNCHER_POSES[i0][0];
                double d1 = Data.LAUNCHER_POSES[i1][0];
                double t = (Math.abs(d1 - d0) < 1e-6) ? 0.0 : (clampedDist - d0) / (d1 - d0);

                double baseRpm = lerp(Data.LAUNCHER_POSES[i0][1], Data.LAUNCHER_POSES[i1][1], t);
                double baseHoodAngle = lerp(Data.LAUNCHER_POSES[i0][2], Data.LAUNCHER_POSES[i1][2], t);

                double finalRpm = baseRpm;
                double finalHoodAngle = baseHoodAngle;
                calculatedTurretOffset = 0.0;

                // 2. VELOCITY COMPENSATION
                Vector robotVel = PedroComponent.follower().getVelocity();

                // Check: Enabled? AND Close Enough? AND Moving?
                if (useVelocityCompensation
                        && distIn < MAX_COMPENSATION_DIST_IN
                        && robotVel != null
                        && robotVel.getMagnitude() > 0.5) {

                    double angleToGoal = VisionDistanceHelper.getAngleToGoalRad(currentPose);
                    double robotVelAngle = Math.atan2(robotVel.getYComponent(), robotVel.getXComponent());
                    double robotSpeed = robotVel.getMagnitude();
                    double relativeAngle = robotVelAngle - angleToGoal;

                    double vRadial = robotSpeed * Math.cos(relativeAngle);
                    double vTangential = robotSpeed * Math.sin(relativeAngle);

                    double physicsAngle = 90.0 - baseHoodAngle;
                    double vTotalBase = rpmToInchesPerSec(baseRpm);
                    double vVerticalBase = vTotalBase * Math.sin(Math.toRadians(physicsAngle));
                    double vHorizontalBase = vTotalBase * Math.cos(Math.toRadians(physicsAngle));

                    double timeOfFlight = distIn / vHorizontalBase;
                    double vHorizontalNeeded = (distIn / timeOfFlight) - vRadial;
                    double vTangentialNeeded = -vTangential;

                    double vHorizontalNew = Math.hypot(vHorizontalNeeded, vTangentialNeeded);
                    double rawOffset = Math.toDegrees(Math.atan2(vTangentialNeeded, vHorizontalNeeded));
                    calculatedTurretOffset = rawOffset * TURRET_COMPENSATION_GAIN;

                    double idealPhysicsAngle = Math.toDegrees(Math.atan2(vVerticalBase, vHorizontalNew));
                    double idealHoodAngle = 90.0 - idealPhysicsAngle;
                    double clampedHoodAngle = Range.clip(idealHoodAngle, MIN_HOOD_ANGLE, MAX_HOOD_ANGLE);
                    finalHoodAngle = clampedHoodAngle;

                    if (Math.abs(idealHoodAngle - clampedHoodAngle) > 0.1) {
                        double physicsRad = Math.toRadians(90.0 - clampedHoodAngle);
                        double vRequired = Math.sqrt( (distIn * GRAVITY_IN_SEC2) / Math.sin(2 * physicsRad) );
                        finalRpm = inchesPerSecToRpm(vRequired);
                    } else {
                        double vTotalNew = Math.hypot(vHorizontalNew, vVerticalBase);
                        finalRpm = inchesPerSecToRpm(vTotalNew);
                    }
                }

                setAngle(finalHoodAngle);
                setTargetRpm(finalRpm);
            }
        }

        // --- PID Update ---
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

    private static Pose getPose() {
        Pose currentPose = PedroComponent.follower().getPose();
        Vector robotVel = PedroComponent.follower().getVelocity();
        double LOOKAHEAD_TIME = 0.15;
        if (robotVel != null) {
            double predictedX = currentPose.getX() + (robotVel.getXComponent() * LOOKAHEAD_TIME);
            double predictedY = currentPose.getY() + (robotVel.getYComponent() * LOOKAHEAD_TIME);
            currentPose = new Pose(predictedX, predictedY, currentPose.getHeading());
        }
        return currentPose;
    }
}