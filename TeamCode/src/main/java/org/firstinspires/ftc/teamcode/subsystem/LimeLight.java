package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import dev.frozenmilk.dairy.core.FeatureRegistrar;
import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import dev.frozenmilk.mercurial.subsystems.SubsystemObjectCell;

public class LimeLight implements Subsystem {
    public static final LimeLight INSTANCE = new LimeLight();

    private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(Attach.class));

    private final SubsystemObjectCell<Limelight3A> limeLight = subsystemCell(() -> FeatureRegistrar.getActiveOpMode().hardwareMap.get(Limelight3A.class, "lime_light"));
    private final SubsystemObjectCell<BNO055IMU> imu = subsystemCell(() -> FeatureRegistrar.getActiveOpMode().hardwareMap.get(BNO055IMU.class, "imu"));

    public static double getX() {
        return getLimeLight().getLatestResult().getTx();
    }

    public static LLResult getLatest() {
        return getLimeLight().getLatestResult();
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

    public static Limelight3A getLimeLight() {
        return INSTANCE.limeLight.get();
    }

    public static BNO055IMU getIMU() {
        return INSTANCE.imu.get();
    }

    @Override
    public void postUserInitHook(@NonNull Wrapper opMode) {
        getLimeLight().pipelineSwitch(0);
        getLimeLight().start();
    }

    @Override
    public void preUserLoopHook(@NonNull Wrapper opMode) {
        Orientation orientation = getIMU().getAngularOrientation();
        getLimeLight().updateRobotOrientation(orientation.firstAngle);
        LLResult result = getLimeLight().getLatestResult();
        if (result != null) {
            if (result.isValid()) {
                Pose3D botpose = result.getBotpose();
                Telemetry telemetry = opMode.getOpMode().telemetry;
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("Botpose", botpose.toString());
            }
        }
    }
}
