package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.core.FeatureRegistrar;
import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import dev.frozenmilk.mercurial.subsystems.SubsystemObjectCell;

public class Drive implements Subsystem {
    public static final Drive INSTANCE = new Drive();

    private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(Attach.class));

    private final SubsystemObjectCell<DcMotorEx> frontLeft = subsystemCell(() -> FeatureRegistrar.getActiveOpMode().hardwareMap.get(DcMotorEx.class, "front_left"));
    private final SubsystemObjectCell<DcMotorEx> frontRight = subsystemCell(() -> FeatureRegistrar.getActiveOpMode().hardwareMap.get(DcMotorEx.class, "front_right"));
    private final SubsystemObjectCell<DcMotorEx> backLeft = subsystemCell(() -> FeatureRegistrar.getActiveOpMode().hardwareMap.get(DcMotorEx.class, "back_left"));
    private final SubsystemObjectCell<DcMotorEx> backRight = subsystemCell(() -> FeatureRegistrar.getActiveOpMode().hardwareMap.get(DcMotorEx.class, "back_right"));
    private final SubsystemObjectCell<Follower> follower = subsystemCell(() -> new Follower(
            new FollowerConstants(),
            new PinpointLocalizer(
                    FeatureRegistrar.getActiveOpMode().hardwareMap,
                    new PinpointConstants()
            ),
            new Mecanum(
                    FeatureRegistrar.getActiveOpMode().hardwareMap,
                    buildConstants()
            )
    ));

    private MecanumConstants buildConstants() {
        MecanumConstants mecanumConstants = new MecanumConstants();
        mecanumConstants.setLeftRearMotorDirection(DcMotorSimple.Direction.REVERSE);
        mecanumConstants.setRightRearMotorDirection(DcMotorSimple.Direction.REVERSE);
        return mecanumConstants;
    }

    public static void drive(double x, double y, double r) {
        getFollower().setTeleOpDrive(y, x, r, true);
    }

    @NonNull
    @Override
    public Dependency<?> getDependency() {
        return dependency;
    }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) {
        this.dependency = dependency;
    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target(ElementType.TYPE)
    @Inherited
    public @interface Attach{}

    public static DcMotorEx getFrontLeft() {
        return INSTANCE.frontLeft.get();
    }

    public static DcMotorEx getFrontRight() {
        return INSTANCE.frontRight.get();
    }

    public static DcMotorEx getBackLeft() {
        return INSTANCE.backLeft.get();
    }

    public static DcMotorEx getBackRight() {
        return INSTANCE.backRight.get();
    }

    public static Follower getFollower() {
        return INSTANCE.follower.get();
    }
}
